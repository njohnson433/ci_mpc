#!/usr/bin/env python3
# Cartesian stiction of the real FR3 at the probe pose -> C3 model arm_friction and the friction-compensation
# feedforward (peg_insert_demo.py --c3-arm-friction / --c3-stiction-ff). Real-robot version of
# `peg_insert_demo.py --breakaway` (Isaac): no sim clock, no MPC node, wall time, safety stops.
#
# Needs: franka_hardware + cartesian_impedance_controller active (config/real_robot_controllers.yaml), the arm in
# free space at the pose to measure (e.g. 10 mm above the hole block, peg in hand), a person on the enabling device.
# Procedure per direction and repeat: set-point fixed at the start pose, integral off, contact stiffness; the
# feedforward force (or torque) ramps along the direction until the TCP has moved --move along it. The spring is
# ~0 at that point, so the feedforward is the static friction plus a constant model bias (gravity / payload error).
# Opposite directions separate the two: stiction = (f+ + f-) / 2, bias = (f+ - f-) / 2.
#
# Safety: ramp stops at --max-force / --max-torque, the run aborts (feedforward 0, hold) if the TCP leaves a
# --abort-mm ball around the start pose or moves faster than --abort-speed; without a command for 50 ms the
# controller holds the pose it is at (mpc_timeout), so killing this script is safe.
#
#   python3 c3_breakaway_real.py --dry-run            # read-only: pose, joint_states rate, controller params
#   python3 c3_breakaway_real.py --out ~/ci_mpc_runs/real/breakaway/r1
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--dirs", default="+x,-x,+y,-y,+z,-z", help="directions; rotations +rx,-rx,+ry,-ry also possible")
p.add_argument("--repeats", type=int, default=3)
p.add_argument("--stiffness", default="1500,1500,500,260,260,90", help="K during the ramp (the demo's contact K)")
p.add_argument("--rate", type=float, default=0.5, help="force ramp [N/s] (Isaac used 1.0)")
p.add_argument("--torque-rate", type=float, default=0.1, help="torque ramp [Nm/s] for rx / ry")
p.add_argument("--move", type=float, default=0.1, help="breakaway threshold along the direction [mm]")
p.add_argument("--move-rot", type=float, default=1.0, help="breakaway threshold for rx / ry [mrad]")
p.add_argument("--max-force", type=float, default=8.0, help="stop the ramp at this force [N]")
p.add_argument("--max-torque", type=float, default=2.0, help="stop the ramp at this torque [Nm]")
p.add_argument("--abort-mm", type=float, default=5.0, help="abort if the TCP is this far from the start pose [mm]")
p.add_argument("--abort-speed", type=float, default=0.03, help="abort above this TCP speed [m/s]")
p.add_argument("--settle", type=float, default=1.5, help="hold at the start pose between ramps [s]")
p.add_argument("--joint-states", default="/joint_states")
p.add_argument("--controller", default="/cartesian_impedance_controller")
p.add_argument("--urdf", default=os.path.expanduser("~/ci_mpc_assets/fr3.urdf"))
p.add_argument("--out", default="", help="output dir (breakaway.json)")
p.add_argument("--dry-run", action="store_true", help="read only, publish nothing")
p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
args = p.parse_args()

full = pin.buildModelFromUrdf(args.urdf)
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
ee_id = model.getFrameId("fr3_hand_tcp")


def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].copy()


rclpy.init()
node = Node("c3_breakaway_real")
state = {"q": None, "n": 0, "wrench": np.zeros(6)}


def on_js(msg):
    if all(n in msg.name for n in ARM):
        state["q"] = np.array([msg.position[msg.name.index(n)] for n in ARM])
        state["n"] += 1


def on_wrench(m):
    w = m.wrench
    state["wrench"] = np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])


node.create_subscription(JointState, args.joint_states, on_js, qos_profile_sensor_data)
node.create_subscription(WrenchStamped, args.controller + "/external_wrench", on_wrench, 10)
cmd_pub = node.create_publisher(MpcCommand, args.controller + "/mpc_command", qos_profile_sensor_data)


def spin(seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.002)


def param(name):
    r = subprocess.run(["ros2", "param", "get", args.controller, name], capture_output=True, text=True, timeout=20)
    return (r.stdout + r.stderr).strip()


t_wait = time.monotonic()
while state["q"] is None and time.monotonic() - t_wait < 5.0:
    rclpy.spin_once(node, timeout_sec=0.05)
if state["q"] is None:
    sys.exit(f"no {args.joint_states} with the 7 FR3 joints within 5 s")
n0 = state["n"]
spin(1.0)
x0 = fk(state["q"])
params = {k: param(k) for k in ("model_source", "friction_compensation", "friction_scale", "mpc_timeout")}
print(f"joint_states {state['n'] - n0:.0f} Hz, TCP {np.round(x0.translation, 4).tolist()} m (Pinocchio, "
      f"{os.path.basename(args.urdf)}), external wrench {np.round(state['wrench'], 2).tolist()}")
for k, v in params.items():
    print(f"  {k}: {v}")
if args.dry_run:
    sys.exit(0)
if not args.yes and input(f"ramp {args.dirs} x{args.repeats} up to {args.max_force} N at this pose, enabling device "
                          f"in hand? type 'go': ").strip() != "go":
    sys.exit("not confirmed")

K = [float(v) for v in args.stiffness.split(",")]
quat = pin.Quaternion(x0.rotation)
cmd = {"ff": np.zeros(6)}


def publish():
    m = MpcCommand()
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, x0.translation)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat.x, quat.y, quat.z, quat.w
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, cmd["ff"][:3])
    m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = map(float, cmd["ff"][3:])
    m.stiffness = K
    m.integral_enabled = False
    cmd_pub.publish(m)


node.create_timer(0.005, publish)


def rot_err(R):  # small-angle rotation of R relative to the start orientation, base frame [rad]
    return pin.log3(R @ x0.rotation.T)


def abort(msg):
    cmd["ff"] = np.zeros(6)
    spin(0.5)
    print("ABORT: " + msg, flush=True)
    finish(2)


AXES = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4}
out = {"x0": x0.translation.tolist(), "R0": x0.rotation.tolist(), "stiffness": K, "rate": args.rate,
       "torque_rate": args.torque_rate, "move_mm": args.move, "move_mrad": args.move_rot, "controller": params,
       "runs": []}


def finish(code):
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump(out, open(os.path.join(args.out, "breakaway.json"), "w"), indent=1)
        print(f"wrote {os.path.join(args.out, 'breakaway.json')}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


spin(args.settle)
for rep in range(args.repeats):
    for name in args.dirs.split(","):
        sign, ax = (1.0 if name[0] == "+" else -1.0), AXES[name[1:]]
        rot = ax >= 3
        rate, lim = (args.torque_rate, args.max_torque) if rot else (args.rate, args.max_force)
        thr = args.move_rot / 1000 if rot else args.move / 1000
        cmd["ff"] = np.zeros(6)
        spin(args.settle)
        xs = fk(state["q"])
        drift = float(np.linalg.norm(xs.translation - x0.translation) * 1000)
        if drift > args.abort_mm:
            abort(f"start pose drifted {drift:.2f} mm")
        t0, x_prev, t_prev, trace, f_brk = time.monotonic(), xs.translation.copy(), time.monotonic(), [], None
        while True:
            spin(0.005)
            t = time.monotonic()
            f = rate * (t - t0)
            cmd["ff"] = np.zeros(6)
            cmd["ff"][ax] = sign * f
            x = fk(state["q"])
            d = (x.translation - xs.translation) if not rot else rot_err(x.rotation) - rot_err(xs.rotation)
            s = sign * float(d[ax % 3])
            v = float(np.linalg.norm(x.translation - x_prev) / max(t - t_prev, 1e-3))
            x_prev, t_prev = x.translation.copy(), t
            trace.append([round(t - t0, 4), round(f, 4), *np.round((x.translation - x0.translation) * 1000, 4).tolist(),
                          *np.round(rot_err(x.rotation) * 1000, 3).tolist(), *np.round(state["wrench"], 3).tolist()])
            if np.linalg.norm(x.translation - x0.translation) * 1000 > args.abort_mm:
                abort(f"{name}: TCP {np.linalg.norm(x.translation - x0.translation) * 1000:.2f} mm from the start pose")
            if v > args.abort_speed:
                abort(f"{name}: TCP speed {v:.3f} m/s")
            if s > thr:
                f_brk = f
                break
            if f > lim:
                break
        cmd["ff"] = np.zeros(6)
        unit = "Nm" if rot else "N"
        out["runs"].append({"dir": name, "repeat": rep, "f_breakaway": f_brk, "start_drift_mm": drift, "trace": trace[::4]})
        print(f"  [{rep}] {name}: " + (f"{f_brk:.3f} {unit}" if f_brk is not None else f"none up to {lim} {unit}")
              + f" (start {drift:.2f} mm from x0)", flush=True)

# summary: per axis stiction = (f+ + f-)/2 and bias = (f+ - f-)/2 over the repeat medians
med = {}
for name in args.dirs.split(","):
    fs = [r["f_breakaway"] for r in out["runs"] if r["dir"] == name and r["f_breakaway"] is not None]
    med[name] = float(np.median(fs)) if fs else None
out["median"] = med
summ = {}
for a in AXES:
    fp, fm = med.get("+" + a), med.get("-" + a)
    if fp is not None and fm is not None:
        summ[a] = {"stiction": (fp + fm) / 2, "bias": (fp - fm) / 2}
out["axes"] = summ
print("median breakaway: " + ", ".join(f"{k} {v:.3f}" if v is not None else f"{k} none" for k, v in med.items()))
for a, v in summ.items():
    print(f"  {a}: stiction {v['stiction']:.3f}, bias {v['bias']:+.3f} (positive: the arm drifts toward -{a})")
lat = [summ[a]["stiction"] for a in ("x", "y") if a in summ]
down = med.get("-z")
if lat and down is not None:
    # offline (c3_stall_diag.py, sticky plant 3 / 3 / 5 N): compensation 2 / 3 N with the true stiction in the model
    # inserted 16/16; the same ratios (2/3 lateral, 3/5 down) scaled to the measured values are the starting point,
    # to be checked offline on a plant with these values before the robot runs C3
    af = [summ.get("x", {"stiction": np.mean(lat)})["stiction"], summ.get("y", {"stiction": np.mean(lat)})["stiction"],
          down, summ["rx"]["stiction"] if "rx" in summ else 0.05, summ["ry"]["stiction"] if "ry" in summ else 0.05]
    ff = [2.0 / 3.0 * float(np.mean(lat)), 3.0 / 5.0 * down]
    out["suggested"] = {"c3_arm_friction": af, "c3_stiction_ff": ff}
    print(f"suggested: --c3-arm-friction {','.join(f'{v:.2f}' for v in af)} --c3-stiction-ff {ff[0]:.2f},{ff[1]:.2f}")
spin(0.5)
finish(0)
