#!/usr/bin/env python3
# ROS side of the impedance-controller test (system ROS 2 Humble, Python 3.10).
# Activates joint_state_broadcaster + cartesian_impedance_controller on a running Controller
# Manager, then checks (EE pose from /joint_states via Pinocchio FK, timing from message stamps):
#   1. hold: after activation the EE stays where it was
#   2. reach: after a reference pose, the EE converges to it
# Exits 0 if both pass. --sync-dir: wait for <dir>/home_done before activating, write <dir>/done at exit.
import argparse
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

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--ee", default="fr3_hand_tcp")
p.add_argument("--sync-dir", default="")
p.add_argument("--hold", type=float, default=3.0, help="sim seconds")
p.add_argument("--reach", type=float, default=10.0, help="sim seconds")
p.add_argument("--target", type=float, nargs=3, default=[0.4, 0.1, 0.4])
p.add_argument("--hold-tol", type=float, default=0.005, help="m")
p.add_argument("--reach-tol", type=float, default=0.01, help="m")
p.add_argument("--rot-tol", type=float, default=0.05, help="rad")
p.add_argument("--dump", default="", help="save /joint_states samples + phase markers to this .npz")
args = p.parse_args()


def sync(name):
    return os.path.join(args.sync_dir, name) if args.sync_dir else ""


def finish(code):
    if args.sync_dir:
        open(sync("done"), "w").close()
    sys.exit(code)


def run(cmd, timeout=120):
    print("$", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    print((r.stdout + r.stderr).strip(), flush=True)
    return r.returncode == 0 and "rror" not in r.stdout + r.stderr


# Pinocchio model, fingers locked (same reduction as the controller's PinocchioModel)
full = pin.buildModelFromUrdf(args.urdf)
arm = [f"fr3_joint{i}" for i in range(1, 8)]
lock = [j for j in range(1, full.njoints) if full.names[j] not in arm]
model = pin.buildReducedModel(full, lock, pin.neutral(full))
data = model.createData()
ee_id = model.getFrameId(args.ee)


def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].copy()


rclpy.init()
node = Node("impedance_tracking_check")
samples = []  # (t, q)


def on_js(msg):
    if not all(n in msg.name for n in arm):
        return
    t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
    samples.append((t, np.array([msg.position[msg.name.index(n)] for n in arm])))


node.create_subscription(JointState, "/joint_states", on_js, 50)
pose_pub = node.create_publisher(Pose, "/cartesian_impedance_controller/reference_pose", 1)


def spin_until(pred, wall_timeout):
    t_end = time.time() + wall_timeout
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def spin_sim(seconds, wall_timeout=600):
    t_start = samples[-1][0]
    return spin_until(lambda: samples[-1][0] - t_start >= seconds, wall_timeout)


# 1. Controller Manager up -> joint_state_broadcaster
cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
if not cli.wait_for_service(timeout_sec=300):
    print("FAIL: /controller_manager not available")
    finish(2)
if not run(["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"]):
    finish(2)
if not spin_until(lambda: len(samples) > 0, 60):
    print("FAIL: no /joint_states")
    finish(2)

# 2. wait for the sim side to put the arm at home, then activate the impedance controller
if args.sync_dir and not spin_until(lambda: os.path.exists(sync("home_done")), 300):
    print("FAIL: sim never reported home_done")
    finish(2)
spin_sim(0.2)
T0 = fk(samples[-1][1])
print(f"EE before activation: {np.round(T0.translation, 4)}", flush=True)
if not run(["ros2", "control", "load_controller", "--set-state", "active", "cartesian_impedance_controller"]):
    finish(2)
run(["ros2", "control", "list_controllers"])

# 3. hold
n0 = len(samples)
spin_sim(args.hold)
drift = max(np.linalg.norm(fk(q).translation - T0.translation) for _, q in samples[n0:])
rot_drift = max(np.linalg.norm(pin.log3(T0.rotation.T @ fk(q).rotation)) for _, q in samples[n0:])
dt = np.diff([t for t, _ in samples[n0:]])
print(f"HOLD  {args.hold:.1f} s sim: max drift {drift * 1000:.2f} mm, max rot drift {rot_drift:.4f} rad, "
      f"/joint_states period {np.median(dt) * 1000:.1f} ms", flush=True)
hold_ok = drift < args.hold_tol and rot_drift < args.rot_tol

# 4. reach (gripper pointing down: quaternion x,y,z,w = 1,0,0,0)
target = np.array(args.target)
R_des = np.diag([1.0, -1.0, -1.0])
msg = Pose()
msg.position.x, msg.position.y, msg.position.z = target.tolist()
msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = 1.0, 0.0, 0.0, 0.0
n1 = len(samples)
t1 = samples[-1][0]
pose_pub.publish(msg)
spin_sim(args.reach)
traj = [(t - t1, fk(q)) for t, q in samples[n1:]]
err = np.array([np.linalg.norm(T.translation - target) for _, T in traj])
rot_err = np.array([np.linalg.norm(pin.log3(R_des.T @ T.rotation)) for _, T in traj])
t_1cm = next((t for (t, _), e in zip(traj, err) if e < 0.01), float("nan"))
for frac in (0.25, 0.5, 0.75, 1.0):
    i = min(int(frac * len(traj)) - 1, len(traj) - 1)
    print(f"  t={traj[i][0]:5.2f} s  EE {np.round(traj[i][1].translation, 4)}  err {err[i] * 1000:6.1f} mm  "
          f"rot err {rot_err[i]:.3f} rad", flush=True)
print(f"REACH target {target}: final err {err[-1] * 1000:.2f} mm, rot err {rot_err[-1]:.4f} rad, "
      f"time to <1 cm {t_1cm:.2f} s", flush=True)
reach_ok = err[-1] < args.reach_tol and rot_err[-1] < args.rot_tol

if args.dump:
    np.savez(args.dump, t=np.array([t for t, _ in samples]), q=np.array([q for _, q in samples]),
             hold_start=n0, reach_start=n1, target=target)
    print(f"samples saved to {args.dump}", flush=True)
print(f"RESULT hold={'PASS' if hold_ok else 'FAIL'} reach={'PASS' if reach_ok else 'FAIL'}", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if hold_ok and reach_ok else 1)
