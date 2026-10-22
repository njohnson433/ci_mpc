#!/usr/bin/env python3
# Test of the controller's MPC interface (ci_mpc_msgs/MpcCommand) in sim, ROS side
# (run via run_impedance_test.sh with CHECKER=this script; the wrench tests need --sync-dir).
#   circle:   r 5 cm / period 5 s in the y-z plane, streamed at 100 Hz, three ways:
#             reference_pose (old path, filtered), MpcCommand pose only, MpcCommand pose + twist
#   timeout:  stream the home pose, stop; the arm must hold where it is
#   ff:       10 N external push in x with and without a -10 N feedforward wrench (integral off)
#   gains:    commanded K 500 N/m in x, integral off, 5 N push -> 10 mm
#   integral: 6 N push (2.4 mm on the spring alone, inside the 3 mm integral window), integral on vs off
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
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", default="")
p.add_argument("--rate", type=float, default=100.0, help="command rate [Hz, sim time]")
p.add_argument("--tests", default="circle,timeout,ff,gains,integral")
p.add_argument("--out", default="")
args = p.parse_args()
tests = args.tests.split(",")

full = pin.buildModelFromUrdf(args.urdf)
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
ee_id = model.getFrameId("fr3_hand_tcp")


def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].copy()


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


rclpy.init()
node = Node("mpc_tracking_test")
samples = []


def on_js(msg):
    if all(n in msg.name for n in ARM):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        samples.append((t, np.array([msg.position[msg.name.index(n)] for n in ARM])))


node.create_subscription(JointState, "/joint_states", on_js, 100)
pose_pub = node.create_publisher(Pose, "/cartesian_impedance_controller/reference_pose", 10)
mpc_pub = node.create_publisher(MpcCommand, "/cartesian_impedance_controller/mpc_command", 10)


def spin_until(pred, wall_timeout=600):
    t_end = time.time() + wall_timeout
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.005)
        if pred():
            return True
    return False


def now():
    return samples[-1][0]


def spin_sim(seconds):
    t0 = now()
    spin_until(lambda: now() - t0 >= seconds)


def window(t0, t1):
    return [(t, fk(q)) for t, q in samples if t0 <= t <= t1]


def pose_msg(pos, R):
    q = pin.Quaternion(R)
    m = Pose()
    m.position.x, m.position.y, m.position.z = map(float, pos)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q.x, q.y, q.z, q.w
    return m


def mpc_msg(pos, R, vel=(0, 0, 0), wrench=(0, 0, 0, 0, 0, 0), stiffness=None, integral=True):
    m = MpcCommand()
    m.header.stamp.sec = int(now())
    m.header.stamp.nanosec = int((now() % 1) * 1e9)
    m.pose = pose_msg(pos, R)
    m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, vel)
    (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
     m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z) = map(float, wrench)
    if stiffness is not None:
        m.stiffness = [float(k) for k in stiffness]
    m.integral_enabled = integral
    return m


def stream(duration, make_msg, publish):
    """Publish make_msg(t_rel) at args.rate (sim time) for duration seconds; returns (times, refs)."""
    t0 = now()
    t_next = t0
    ts, refs = [], []
    while now() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.002)
        if now() >= t_next:
            msg, ref = make_msg(now() - t0)
            publish(msg)
            ts.append(now())
            refs.append(ref)
            t_next += 1.0 / args.rate
    return np.array(ts), np.array(refs)


def set_wrench(force):
    tmp = sync("wrench.json.tmp")
    json.dump({"force": list(force), "torque": [0, 0, 0]}, open(tmp, "w"))
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
if not run(["ros2", "control", "load_controller", "--set-state", "active", "cartesian_impedance_controller"]):
    finish(2)
spin_sim(1.0)
T_home = fk(samples[-1][1])
p_home, R_home = T_home.translation.copy(), T_home.rotation.copy()
print(f"home EE {np.round(p_home, 4)}", flush=True)
results, saved = {}, {}


def go_home(seconds=3.0):
    """Stream the home pose with MPC commands (integral on), then keep streaming briefly."""
    stream(seconds, lambda t: (mpc_msg(p_home, R_home), p_home), mpc_pub.publish)


# --- circle, three ways
if "circle" in tests:
    r_c, period, laps = 0.05, 5.0, 2
    center = p_home - np.array([0, r_c, 0])
    w = 2 * np.pi / period

    def ref(t):
        return center + r_c * np.array([0, np.cos(w * t), np.sin(w * t)])

    def vel(t):
        return r_c * w * np.array([0, -np.sin(w * t), np.cos(w * t)])

    variants = {
        "reference_pose (filtered)": (lambda t: (pose_msg(ref(t), R_home), ref(t)), pose_pub.publish),
        "MpcCommand pose only": (lambda t: (mpc_msg(ref(t), R_home), ref(t)), mpc_pub.publish),
        "MpcCommand pose + twist": (lambda t: (mpc_msg(ref(t), R_home, vel(t)), ref(t)), mpc_pub.publish),
    }
    print(f"CIRCLE r {r_c * 100:.0f} cm, period {period:.0f} s, streamed at {args.rate:.0f} Hz "
          "(error vs commanded, after the first half lap)")
    rows = []
    for label, (make, pub) in variants.items():
        if label.startswith("reference_pose"):
            pose_pub.publish(pose_msg(p_home, R_home))  # MPC timeout returns control to reference_pose
            spin_sim(2.0)
        ts, refs = stream(laps * period, make, pub)
        win = window(ts[0] + period / 2, ts[-1])
        tw = np.array([t for t, _ in win])
        pos = np.array([T.translation for _, T in win])
        cmd = refs[np.clip(np.searchsorted(ts, tw) - 1, 0, len(ts) - 1)]
        e = np.linalg.norm(pos - cmd, axis=1)
        rows.append((label, np.sqrt(np.mean(e ** 2)) * 1000, e.max() * 1000))
        saved[f"circle_{label}"] = np.array([ts[0], ts[-1]])
        go_home()
    for label, rms, mx in rows:
        print(f"  {label:28s}: RMS {rms:6.2f} mm, max {mx:6.2f} mm")
    results["circle"] = rows[-1][1] < 3.0
    print(f"  -> {'PASS' if results['circle'] else 'FAIL'} (pose + twist RMS < 3 mm)", flush=True)

# --- timeout: stop streaming, the arm must hold
if "timeout" in tests:
    go_home(2.0)
    t_stop = now()
    spin_sim(2.0)
    win = window(t_stop, now())
    drift = max(np.linalg.norm(T.translation - win[0][1].translation) for _, T in win)
    results["timeout"] = drift < 1e-3
    print(f"TIMEOUT: drift over 2 s after the last command {drift * 1000:.3f} mm -> "
          f"{'PASS' if results['timeout'] else 'FAIL'} (< 1 mm)", flush=True)


def push_test(label, force, make_msg, duration=3.0):
    go_home(2.0)
    t0 = now()
    set_wrench(force)
    stream(duration, lambda t: (make_msg(), p_home), mpc_pub.publish)
    clear_wrench()
    d = (window(now() - 0.05, now())[-1][1].translation - p_home) * 1000
    saved[f"push_{label}"] = np.array([t0, now()])
    return d


# --- feedforward wrench cancels a known external force
if "ff" in tests and args.sync_dir:
    d0 = push_test("ff off", [10, 0, 0], lambda: mpc_msg(p_home, R_home, integral=False))
    d1 = push_test("ff on", [10, 0, 0], lambda: mpc_msg(p_home, R_home, wrench=(-10, 0, 0, 0, 0, 0), integral=False))
    results["ff"] = abs(d1[0]) < 0.25 * abs(d0[0])
    print(f"FEEDFORWARD: 10 N push in x, integral off: deflection x {d0[0]:.2f} mm without, {d1[0]:.2f} mm with "
          f"-10 N feedforward -> {'PASS' if results['ff'] else 'FAIL'} (< 25 %)", flush=True)

# --- commanded stiffness
if "gains" in tests and args.sync_dir:
    K_cmd = [500, 2500, 1000, 130, 130, 45]
    d = push_test("K 500", [5, 0, 0], lambda: mpc_msg(p_home, R_home, stiffness=K_cmd, integral=False))
    results["gains"] = abs(d[0] - 10.0) < 2.0
    print(f"GAINS: K_x 500 N/m commanded, 5 N push: deflection x {d[0]:.2f} mm (expected 10.0) -> "
          f"{'PASS' if results['gains'] else 'FAIL'} (±2 mm, friction)", flush=True)
    go_home(2.0)
    stream(1.0, lambda t: (mpc_msg(p_home, R_home, stiffness=[2500, 2500, 1000, 130, 130, 45]), p_home), mpc_pub.publish)

# --- integral gating
if "integral" in tests and args.sync_dir:
    d_on = push_test("integral on", [6, 0, 0], lambda: mpc_msg(p_home, R_home, integral=True), duration=4.0)
    d_off = push_test("integral off", [6, 0, 0], lambda: mpc_msg(p_home, R_home, integral=False), duration=4.0)
    results["integral"] = abs(d_off[0]) > 2 * abs(d_on[0])
    print(f"INTEGRAL: 6 N push 4 s: deflection x {d_on[0]:.2f} mm with integral, {d_off[0]:.2f} mm without "
          f"(spring alone: 2.40) -> {'PASS' if results['integral'] else 'FAIL'}", flush=True)

if args.out:
    np.savez(args.out, t=np.array([t for t, _ in samples]), q=np.array([q for _, q in samples]), **saved)
print("RESULT " + " ".join(f"{k}={'PASS' if v else 'FAIL'}" for k, v in results.items()), flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if all(results.values()) else 1)
