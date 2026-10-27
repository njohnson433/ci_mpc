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
# Frames: on the robot the controller (model_source franka) tracks franka::Frame::kEndEffector, i.e. the TCP of
# the Desk end-effector profile; pass the same profile with --ee-profile (real: desk_ee_hand_camera_mount_v0.1,
# TCP 111.9 mm from the flange vs 103.4 mm for the URDF fr3_hand_tcp), or every command is 8.5 mm off. --goto is
# given in URDF fr3_hand_tcp coordinates (Isaac's numbers -> the same arm posture). A start-up guard aborts if the
# arm moves > 2 mm when the hold starts (frame mismatch).
#
#   python3 c3_breakaway_real.py --dry-run            # read-only: pose, joint_states rate, controller params
#   python3 c3_breakaway_real.py --goto 0.5,0.1,0.28 --out ...   # move there first (Isaac's probe pose, tip 10 mm
#                                                       # above the hole), orientation kept, then measure
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

try:  # real robot only: the robot's mode (a reflex stops the arm while joint_states keep coming)
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None
MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped", 6: "automatic error recovery"}

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--dirs", default="+x,-x,+y,-y,+z,-z", help="directions; rotations +rx,-rx,+ry,-ry also possible")
p.add_argument("--repeats", type=int, default=3)
p.add_argument("--stiffness", default="1500,1500,500,130,130,45", help="K during the ramp (translational: the demo's contact K; rotational: the controller default; 260/260/90 oscillated at home on the real arm, 2026-10-27)")
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
p.add_argument("--ee-profile", default="", help="Desk end-effector profile JSON (tcp translation / rpy relative "
               "to the flange): the controller's EE frame on the robot; empty = URDF fr3_hand_tcp (Isaac)")
p.add_argument("--goto", default="", help="x,y,z [m]: before measuring, move the set-point there in a straight line "
               "(orientation kept) at --goto-speed, then measure at the reached pose. Isaac poses: home "
               "0.307,0,0.487; probe pose 0.5,0.1,0.28")
p.add_argument("--goto-speed", type=float, default=0.015, help="--goto set-point speed [m/s]")
p.add_argument("--goto-stiffness", default="1500,1500,1500,130,130,45", help="K during --goto")
p.add_argument("--goto-abort-mm", type=float, default=15.0, help="--goto: abort if the TCP lags the set-point this much")
p.add_argument("--zero-spring", action="store_true",
               help="start each ramp from the settled pose (spring force 0). Right for a bias-free model (Isaac); on the "
                    "real arm the model / gravity bias (~1 N) then drifts the arm by itself in one direction and that "
                    "side reads ~0 N (clipped pair); default: set-point stays at x0, the spring holds the bias")
p.add_argument("--dry-run", action="store_true", help="read only, publish nothing")
p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
args = p.parse_args()

full = pin.buildModelFromUrdf(args.urdf)
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
tcp_id, flange_id = model.getFrameId("fr3_hand_tcp"), model.getFrameId("fr3_link8")
F_T_EE = None
if args.ee_profile:
    tcp = json.load(open(args.ee_profile))["tcp"]
    F_T_EE = pin.SE3(pin.rpy.rpyToMatrix(*(tcp["rotation"][k] for k in ("roll", "pitch", "yaw"))),
                     np.array([tcp["translation"][k] for k in ("x", "y", "z")]))


def fk_tcp(q):  # URDF fr3_hand_tcp
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[tcp_id].copy()


def fk(q):  # the controller's EE frame
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[flange_id] * F_T_EE if F_T_EE is not None else data.oMf[tcp_id].copy()


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
state["mode"] = None
if FrankaRobotState is not None:
    node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state",
                             lambda m: state.__setitem__("mode", m.robot_mode), 10)
cmd_pub = node.create_publisher(MpcCommand, args.controller + "/mpc_command", qos_profile_sensor_data)


watch = {"on": False, "aborting": False, "prev": None}


def spin(seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.002)
        # watchdog in every phase (hold, settle, ramp; goto has its own lag check): robot mode, TCP speed
        if watch["on"] and not watch["aborting"] and state["mode"] not in (None, 2):
            abort(f"robot left Move mode: {MODES.get(state['mode'], state['mode'])} (reflex: see launch.log; "
                  f"communication_constraints_violation = the 1 kHz loop missed its deadline)")
        if watch["on"] and not watch["aborting"] and state["q"] is not None:
            t, x = time.monotonic(), fk(state["q"]).translation
            if watch["prev"] is not None and t - watch["prev"][0] >= 0.01:
                v = float(np.linalg.norm(x - watch["prev"][1]) / (t - watch["prev"][0]))
                watch["prev"] = (t, x)
                if v > args.abort_speed:
                    abort(f"watchdog: TCP speed {v:.3f} m/s (oscillation?)")
            elif watch["prev"] is None:
                watch["prev"] = (t, x)


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
d_ee = fk_tcp(state["q"]).actInv(x0)
print(f"EE frame: {os.path.basename(args.ee_profile) if args.ee_profile else 'URDF fr3_hand_tcp'}; relative to "
      f"fr3_hand_tcp {np.round(d_ee.translation * 1000, 2).tolist()} mm, {np.round(pin.log3(d_ee.rotation), 4).tolist()} rad")
goal = None
if args.goto:
    goal_tcp = np.array([float(v) for v in args.goto.split(",")])
    goal = goal_tcp + (x0.translation - fk_tcp(state["q"]).translation)  # same posture, the controller's EE frame
    r_xy, dist = float(np.hypot(*goal_tcp[:2])), float(np.linalg.norm(goal - x0.translation))
    if len(goal) != 3 or not (0.25 <= r_xy <= 0.75 and 0.15 <= goal[2] <= 0.7 and dist <= 0.5):
        sys.exit(f"--goto {args.goto} outside the allowed box (xy radius 0.25-0.75 m, z 0.15-0.7 m, at most 0.5 m away)")
    print(f"goto fr3_hand_tcp {goal_tcp.tolist()} = EE {np.round(goal, 4).tolist()}: {dist * 1000:.0f} mm in a straight line at {args.goto_speed * 1000:.0f} mm/s "
          f"({dist / args.goto_speed:.0f} s), orientation kept")
if args.dry_run:
    sys.exit(0)
if not args.yes and input(("move to --goto, then " if goal is not None else "")
                          + f"ramp {args.dirs} x{args.repeats} up to {args.max_force} N at this pose, enabling device "
                          f"in hand? type 'go': ").strip() != "go":
    sys.exit("not confirmed")

K = [float(v) for v in args.stiffness.split(",")]
cmd = {"ff": np.zeros(6), "p": x0.translation.copy(), "q": pin.Quaternion(x0.rotation), "K": K}
raw = []  # every published command with the measured pose: t, x(3), rot(3) [rad, vs x0 at start], p_cmd(3), ff(6), K(6)
t_start = time.monotonic()


def publish():
    m = MpcCommand()
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, cmd["p"])
    q = cmd["q"]
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q.x, q.y, q.z, q.w
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, cmd["ff"][:3])
    m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = map(float, cmd["ff"][3:])
    m.stiffness = cmd["K"]
    m.integral_enabled = False
    cmd_pub.publish(m)
    if state["q"] is not None:
        x = fk(state["q"])
        raw.append([time.monotonic() - t_start, *x.translation, *pin.log3(x.rotation @ R_start.T), *cmd["p"], *cmd["ff"],
                    *cmd["K"]])


node.create_timer(0.005, publish)


def rot_err(R):  # small-angle rotation of R relative to the start orientation, base frame [rad]
    return pin.log3(R @ x0.rotation.T)


R_start = x0.rotation.copy()


def abort(msg):
    watch["aborting"] = True
    cmd["ff"] = np.zeros(6)
    cmd["p"], cmd["q"] = fk(state["q"]).translation.copy(), pin.Quaternion(fk(state["q"]).rotation)  # hold where it is
    spin(0.5)
    print("ABORT: " + msg, flush=True)
    out["abort"] = msg
    finish(2)


out = {"runs": []}


def finish(code):
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump(out, open(os.path.join(args.out, "breakaway.json"), "w"), indent=1)
        print(f"wrote {os.path.join(args.out, 'breakaway.json')}")
        np.savez_compressed(os.path.join(args.out, "raw.npz"), raw=np.array(raw), x0=x0.translation, R_start=R_start,
                            cols="t x y z rx ry rz px py pz fx fy fz tx ty tz kx ky kz krx kry krz")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


def goto(target):
    # straight-line set-point at --goto-speed; the arm follows on the impedance spring (integral off)
    global x0
    cmd["K"] = [float(v) for v in args.goto_stiffness.split(",")]
    start, t0 = cmd["p"].copy(), time.monotonic()
    T = float(np.linalg.norm(target - start)) / args.goto_speed
    while True:
        spin(0.005)
        a = min((time.monotonic() - t0) / max(T, 1e-3), 1.0)
        cmd["p"] = start + a * (target - start)
        lag = float(np.linalg.norm(fk(state["q"]).translation - cmd["p"]) * 1000)
        if lag > args.goto_abort_mm:
            abort(f"goto: TCP {lag:.1f} mm from the set-point")
        if a >= 1.0:
            break
    spin(2.0)
    # measure from where the arm settled (spring force ~0 there), at the measurement stiffness
    x0 = fk(state["q"])
    cmd["p"], cmd["q"], cmd["K"] = x0.translation.copy(), pin.Quaternion(x0.rotation), K
    print(f"reached {np.round(x0.translation, 4).tolist()} m ({np.linalg.norm(x0.translation - target) * 1000:.1f} mm "
          f"from the goal)", flush=True)


# start-up guard: holding the pose the script computed must not move the arm (else the EE frames differ)
spin(1.0)
jump = float(np.linalg.norm(fk(state["q"]).translation - x0.translation) * 1000)
if jump > 2.0:
    abort(f"the arm moved {jump:.1f} mm when the hold started: the script's EE frame does not match the controller's "
          f"(--ee-profile = the Desk end-effector profile on the robot, empty in Isaac)")
watch["on"] = True
if goal is not None:
    watch["on"] = False  # goto has its own lag check; the set-point itself moves
    goto(goal)
    watch["on"], watch["prev"] = True, None
AXES = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4}
out.update({"x0": x0.translation.tolist(), "R0": x0.rotation.tolist(), "goto": args.goto, "stiffness": K,
            "rate": args.rate, "torque_rate": args.torque_rate, "move_mm": args.move, "move_mrad": args.move_rot,
            "controller": params})


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
        # ramp from where the arm settled (friction keeps it ~0.5 mm off x0): spring force 0 at the start
        if args.zero_spring:
            cmd["p"], cmd["q"] = xs.translation.copy(), pin.Quaternion(xs.rotation)
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
        cmd["p"], cmd["q"] = x0.translation.copy(), pin.Quaternion(x0.rotation)
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
