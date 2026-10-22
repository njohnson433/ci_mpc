#!/usr/bin/env python3
# Extended test of cartesian_impedance_controller (ROS side; sim via run_impedance_test.sh with
# CHECKER=this script, or a real robot without --sync-dir, where the wrench tests are skipped).
#   1. steps:     5 cm steps along ±x, ±y, ±z from home, back home after each
#   2. rotations: 0.3 rad about the world x, y, z axes at the home position
#   3. circle:    r = 5 cm in the y-z plane, period 5 s, 2 laps
#   4. stiffness: external force/torque on the hand (sim only), deflection vs the controller's K
#   5. mode 2:    control_mode 2 (low stiffness) under the same push, then back to mode 0
# The controller low-pass filters reference_pose (first order, tau = 0.5 s at filter_params 0.002/ms);
# tracking errors are reported against the commanded and against the filtered reference.
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--ee", default="fr3_hand_tcp")
p.add_argument("--sync-dir", default="")
p.add_argument("--out", default="", help="save samples and per-test windows to this .npz")
p.add_argument("--filter-tau", type=float, default=0.5, help="controller reference filter time constant [s]")
p.add_argument("--tests", default="steps,rotations,circle,stiffness,mode2")
p.add_argument("--step-hold", type=float, default=4.0, help="sim seconds per step before the final error")
args = p.parse_args()
tests = args.tests.split(",")

# controller gains (cartesian_impedance_controller.hpp, mode 0) and mode-2 stiffness
K0 = np.array([2500, 2500, 1000, 130, 130, 45.0])
K2 = np.array([150, 150, 150, 40, 40, 20.0])
I_CLAMP = np.array([2.5, 2.5, 2.5, 1.5, 1.5, 0.5])  # integral-term clamps (anti-windup)
I_WEIGHT = np.array([350, 350, 0, 50, 50, 50.0])


def sync(name):
    return os.path.join(args.sync_dir, name)


def finish(code):
    if args.sync_dir:
        if os.path.exists(sync("wrench.json")):
            os.remove(sync("wrench.json"))
        open(sync("done"), "w").close()
    sys.exit(code)


def run(cmd):
    print("$", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    print((r.stdout + r.stderr).strip(), flush=True)
    return r.returncode == 0 and "rror" not in r.stdout + r.stderr


full = pin.buildModelFromUrdf(args.urdf)
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
ee_id = model.getFrameId(args.ee)


def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].copy()


rclpy.init()
node = Node("impedance_suite")
samples = []  # (t, q)


def on_js(msg):
    if all(n in msg.name for n in ARM):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        samples.append((t, np.array([msg.position[msg.name.index(n)] for n in ARM])))


node.create_subscription(JointState, "/joint_states", on_js, 100)
pose_pub = node.create_publisher(Pose, "/cartesian_impedance_controller/reference_pose", 10)
mode_pub = node.create_publisher(Int16, "/cartesian_impedance_controller/control_mode", 1)


def spin_until(pred, wall_timeout=600):
    t_end = time.time() + wall_timeout
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if pred():
            return True
    return False


def now():
    return samples[-1][0]


def spin_sim(seconds):
    t0 = now()
    spin_until(lambda: now() - t0 >= seconds)


def send(pos, R):
    q = pin.Quaternion(R)
    m = Pose()
    m.position.x, m.position.y, m.position.z = map(float, pos)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q.x, q.y, q.z, q.w
    pose_pub.publish(m)


def window(t0, t1):
    return [(t, fk(q)) for t, q in samples if t0 <= t <= t1]


def filtered(ref_times, ref_vals, ts):
    """First-order filter of a piecewise-constant reference, evaluated at times ts."""
    out, y, k, t_prev = [], ref_vals[0], 0, ts[0]
    for t in ts:
        while k + 1 < len(ref_times) and ref_times[k + 1] <= t:
            k += 1
        y = y + (ref_vals[k] - y) * (1 - np.exp(-(t - t_prev) / args.filter_tau))
        t_prev = t
        out.append(y.copy())
    return np.array(out)


def set_wrench(force=(0, 0, 0), torque=(0, 0, 0)):
    tmp = sync("wrench.json.tmp")
    json.dump({"force": list(force), "torque": list(torque)}, open(tmp, "w"))
    os.replace(tmp, sync("wrench.json"))
    if os.path.exists(sync("wrench_ack")):
        os.remove(sync("wrench_ack"))
    spin_until(lambda: os.path.exists(sync("wrench_ack")), 30)


def clear_wrench():
    if os.path.exists(sync("wrench.json")):
        os.remove(sync("wrench.json"))


# --- bring-up
cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
if not cli.wait_for_service(timeout_sec=300):
    print("FAIL: /controller_manager not available")
    finish(2)
if not run(["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"]):
    finish(2)
spin_until(lambda: len(samples) > 0, 60)
if args.sync_dir and not spin_until(lambda: os.path.exists(sync("home_done")), 300):
    print("FAIL: sim never reported home_done")
    finish(2)
spin_sim(0.2)
T_home = fk(samples[-1][1])
if not run(["ros2", "control", "load_controller", "--set-state", "active", "cartesian_impedance_controller"]):
    finish(2)
spin_sim(1.0)
p_home, R_home = T_home.translation.copy(), T_home.rotation.copy()
print(f"home EE {np.round(p_home, 4)}", flush=True)
results = {}
saved = {}

# --- 1. translational steps
if "steps" in tests:
    rows = []
    for axis, name in enumerate("xyz"):
        for sgn in (1, -1):
            d = np.zeros(3)
            d[axis] = 0.05 * sgn
            target = p_home + d
            t0 = now()
            send(target, R_home)
            spin_sim(args.step_hold)
            w = window(t0, now())
            ts = np.array([t for t, _ in w])
            pos = np.array([T.translation for _, T in w])
            prog = (pos[:, axis] - p_home[axis]) / d[axis]  # 0 -> 1
            err = np.linalg.norm(pos[-1] - target)
            rot = np.linalg.norm(pin.log3(R_home.T @ w[-1][1].rotation))
            t90 = ts[np.argmax(prog >= 0.9)] - t0 if (prog >= 0.9).any() else np.nan
            # the filtered reference alone needs tau*ln(10) = 1.15 s to reach 90 %
            rows.append((f"{'+' if sgn > 0 else '-'}{name}", err * 1000, (prog.max() - 1) * 100, t90, rot))
            saved[f"step_{name}{sgn:+d}"] = np.array([t0, now()])
            send(p_home, R_home)
            spin_sim(3.0)
    print(f"STEPS 5 cm (final error after {args.step_hold:.0f} s, overshoot, 90 % time; filtered reference alone: 1.15 s)")
    for r in rows:
        print(f"  {r[0]}: final err {r[1]:5.2f} mm  overshoot {r[2]:+5.1f} %  t90 {r[3]:4.2f} s  rot drift {r[4]:.4f} rad")
    worst = max(r[1] for r in rows)
    results["steps"] = worst < 5.0 and max(r[4] for r in rows) < 0.02
    print(f"  worst final err {worst:.2f} mm -> {'PASS' if results['steps'] else 'FAIL'} (< 5 mm, rot < 0.02 rad)", flush=True)

# --- 2. rotations
if "rotations" in tests:
    rows = []
    for axis, name in enumerate("xyz"):
        dR = pin.exp3(np.eye(3)[axis] * 0.3)
        R_t = dR @ R_home
        t0 = now()
        send(p_home, R_t)
        spin_sim(4.0)
        T = window(now() - 0.01, now())[-1][1]
        rows.append((name, np.linalg.norm(pin.log3(R_t.T @ T.rotation)), np.linalg.norm(T.translation - p_home) * 1000))
        saved[f"rot_{name}"] = np.array([t0, now()])
        send(p_home, R_home)
        spin_sim(3.0)
    print("ROTATIONS 0.3 rad about world axes (final error after 4 s)")
    for r in rows:
        print(f"  {r[0]}: rot err {r[1]:.4f} rad  pos drift {r[2]:5.2f} mm")
    results["rotations"] = max(r[1] for r in rows) < 0.02
    print(f"  -> {'PASS' if results['rotations'] else 'FAIL'} (< 0.02 rad)", flush=True)

# --- 3. circle
if "circle" in tests:
    r_c, period, laps, rate = 0.05, 5.0, 2, 50.0
    center = p_home - np.array([0, r_c, 0])  # start on the circle at home
    ref_t, ref_p = [], []
    t0 = now()
    t_next = t0
    while now() - t0 < laps * period:
        rclpy.spin_once(node, timeout_sec=0.005)
        if now() >= t_next:
            ph = 2 * np.pi * (now() - t0) / period
            target = center + r_c * np.array([0, np.cos(ph), np.sin(ph)])
            send(target, R_home)
            ref_t.append(now())
            ref_p.append(target)
            t_next += 1.0 / rate
    send(p_home, R_home)
    w = window(t0, now())
    ts = np.array([t for t, _ in w])
    pos = np.array([T.translation for _, T in w])
    ref_t, ref_p = np.array(ref_t), np.array(ref_p)
    cmd = ref_p[np.clip(np.searchsorted(ref_t, ts) - 1, 0, len(ref_t) - 1)]
    filt = filtered(ref_t, ref_p, ts)  # filter state starts at the first reference, like the controller's
    keep = ts >= t0 + period / 2  # skip the first half lap
    ts, pos, cmd, filt = ts[keep], pos[keep], cmd[keep], filt[keep]
    e_cmd = np.linalg.norm(pos - cmd, axis=1)
    e_filt = np.linalg.norm(pos - filt, axis=1)
    saved["circle"] = np.array([t0, now()])
    print(f"CIRCLE r 5 cm, period {period:.0f} s: RMS err vs commanded {np.sqrt(np.mean(e_cmd ** 2)) * 1000:.1f} mm "
          f"(max {e_cmd.max() * 1000:.1f}), vs filtered reference {np.sqrt(np.mean(e_filt ** 2)) * 1000:.2f} mm "
          f"(max {e_filt.max() * 1000:.2f})")
    results["circle"] = np.sqrt(np.mean(e_filt ** 2)) < 0.005
    print(f"  -> {'PASS' if results['circle'] else 'FAIL'} (RMS vs filtered reference < 5 mm)", flush=True)
    spin_sim(3.0)


def push(label, force, torque, K, hold=3.0):
    """Apply a wrench at home, return (deflection at 0.5 s, at the end) along the pushed axis."""
    f6 = np.concatenate([force, torque])
    axis = int(np.argmax(np.abs(f6)))
    t0 = now()
    set_wrench(force, torque)
    spin_sim(hold)
    clear_wrench()
    w = window(t0, now())
    ts = np.array([t for t, _ in w]) - t0

    def defl(T):
        if axis < 3:
            return T.translation[axis] - p_home[axis]
        return pin.log3(T.rotation @ R_home.T)[axis - 3]  # world-frame rotation

    d = np.array([defl(T) for _, T in w])
    F = f6[axis]
    i05 = np.argmin(np.abs(ts - 0.5))
    k_early, k_end = F / d[i05], F / d[-1]
    i_max = min(I_CLAMP[axis], abs(F)) if I_WEIGHT[axis] > 0 else 0.0
    # rotation error is the quaternion vector part, sin(theta/2) ~ theta/2: stiffness per rad is K/2
    k_axis = K[axis] if axis < 3 else K[axis] / 2
    pred_end = (F - np.sign(F) * i_max) / k_axis
    unit = "mm" if axis < 3 else "mrad"
    print(f"  {label}: deflection 0.5 s {d[i05] * 1000:7.2f} {unit} (K_eff {k_early:7.1f}), "
          f"end {d[-1] * 1000:7.2f} {unit} (K_eff {k_end:7.1f}); "
          f"predicted end {pred_end * 1000:7.2f} {unit} (K {K[axis]:.0f}{'/2 per rad' if axis >= 3 else ''}, integral term up to {i_max:.1f})")
    saved[f"push_{label}"] = np.array([t0, now()])
    send(p_home, R_home)
    spin_sim(2.5)
    return d[-1], pred_end


# --- 4. stiffness under external wrench (sim only)
if "stiffness" in tests and args.sync_dir:
    print("STIFFNESS (mode 0) - 10 N / 1 Nm in the world frame at the TCP")
    out = [push("Fx +10 N", [10, 0, 0], [0, 0, 0], K0), push("Fy +10 N", [0, 10, 0], [0, 0, 0], K0),
           push("Fz +10 N", [0, 0, 10], [0, 0, 0], K0), push("Tz +1 Nm", [0, 0, 0], [0, 0, 1], K0)]
    ratio = [d / pr for d, pr in out]
    results["stiffness"] = all(0.75 < r < 1.33 for r in ratio)
    print(f"  measured/predicted end deflection {np.round(ratio, 2)} -> "
          f"{'PASS' if results['stiffness'] else 'FAIL'} (within 0.75..1.33)", flush=True)

# --- 5. control mode 2 (low stiffness), then back to 0
if "mode2" in tests and args.sync_dir:
    print("MODE 2 (low stiffness) - 3 N in x")
    mode_pub.publish(Int16(data=2))
    spin_sim(1.0)
    d, pr = push("Fx +3 N mode 2", [3, 0, 0], [0, 0, 0], K2)
    mode_pub.publish(Int16(data=0))
    spin_sim(2.0)
    results["mode2"] = 0.75 < d / pr < 1.33
    print(f"  measured/predicted {d / pr:.2f} -> {'PASS' if results['mode2'] else 'FAIL'}", flush=True)

if args.out:
    np.savez(args.out, t=np.array([t for t, _ in samples]), q=np.array([q for _, q in samples]), **saved)
print("RESULT " + " ".join(f"{k}={'PASS' if v else 'FAIL'}" for k, v in results.items()), flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if all(results.values()) else 1)
