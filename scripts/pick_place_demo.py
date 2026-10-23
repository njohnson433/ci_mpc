#!/usr/bin/env python3
# Pick-and-place demo with the stage-1 MPC in sim (ROS side; run via run_impedance_test.sh with PICK_PLACE=1
# CONTROLLER_CONFIG=.../ci_mpc_sim_gripper/config/isaac_pick_place.yaml CHECKER=this script, GUI=1 to watch).
#
# Perception: the object comes from vision_msgs/Detection3DArray on /perception/detections (here the sim
# stand-in scripts/sim_object_detector.py, started by this script, with --pos-noise / --size-noise /
# --yaw-noise-deg; on the robot: the vision pipeline). The grasp is planned from the detection:
#   position = box center (TCP 5 mm above it, fingertips kept 5 mm above the table),
#   hand yaw = box yaw (the 90-degree multiple closest to the current hand that fits the gripper),
#   opening  = box extent across the fingers + 2 cm margin (max 80 mm); grasp check: closed width = extent ± 8 mm.
# Motion: --mode blended (default) sends three paths on /ci_mpc_planner/path, the MPC blends through the via
# points and stops only where the gripper acts; --mode stops sends one goal pose at a time (previous demo).
# Gripper: sim stand-in ci_mpc_sim_gripper (/gripper/width); on the robot: franka_gripper (docs/REAL_ROBOT.md 4b).
import argparse
import atexit
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3DArray

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
p.add_argument("--mode", default="blended", choices=["blended", "stops"])
p.add_argument("--pos-noise", type=float, default=0.0)
p.add_argument("--size-noise", type=float, default=0.0)
p.add_argument("--yaw-noise-deg", type=float, default=0.0)
p.add_argument("--seed", type=int, default=1)
args = p.parse_args()

PLACE = np.array([0.45, 0.12])
Z_SAFE = 0.33
FINGERTIP_BELOW_TCP = 0.0095  # fingertip bottom below the TCP (scene geometry)
MAX_OPENING, OPEN_MARGIN, GRASP_TOL = 0.08, 0.02, 0.008

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


procs = []


def _kill_children():  # also on a crash: orphaned detector / MPC nodes would keep publishing into the next run
    for pr in procs:
        if pr.poll() is None:
            pr.kill()


atexit.register(_kill_children)


def finish(code):
    for pr in procs:
        if pr.poll() is None:
            pr.send_signal(signal.SIGINT)
            try:
                pr.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pr.kill()
    open(sync("done"), "w").close()
    sys.exit(code)


def run(cmd):
    print("$", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    print((r.stdout + r.stderr).strip(), flush=True)
    return r.returncode == 0 and "rror" not in r.stdout + r.stderr


rclpy.init()
node = Node("pick_place_demo")
state = {"q": None, "t": 0.0, "fingers": None, "det": None}


def on_js(msg):
    if all(n in msg.name for n in ARM):
        state["q"] = np.array([msg.position[msg.name.index(n)] for n in ARM])
        state["t"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if "fr3_finger_joint1" in msg.name:
            state["fingers"] = sum(msg.position[msg.name.index(f"fr3_finger_joint{i}")] for i in (1, 2))


def on_det(msg):
    if msg.detections:
        state["det"] = msg.detections[0]


node.create_subscription(JointState, "/joint_states", on_js, 100)
node.create_subscription(Detection3DArray, "/perception/detections", on_det, 10)
goal_pub = node.create_publisher(Pose, "/ci_mpc_planner/goal_pose", 10)
path_pub = node.create_publisher(PoseArray, "/ci_mpc_planner/path", 10)
width_pub = node.create_publisher(Float64, "/gripper/width", 10)


def spin_until(pred, wall_timeout=300):
    t_end = time.time() + wall_timeout
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if pred():
            return True
    return False


def spin_sim(seconds):
    t0 = state["t"]
    spin_until(lambda: state["t"] - t0 >= seconds)


def cube_truth():
    try:
        o = json.load(open(sync("object.json")))
        return np.array(o["cube"]), o["quat_wxyz"]
    except (OSError, ValueError, KeyError):
        return np.full(3, np.nan), [1, 0, 0, 0]


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def gripper(width, settle=1.0):
    width_pub.publish(Float64(data=float(width)))
    spin_sim(settle)


def pose_msg(xyz, R):
    m = Pose()
    m.position.x, m.position.y, m.position.z = map(float, xyz)
    q = pin.Quaternion(R)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q.x, q.y, q.z, q.w
    return m


def wait_at(xyz, tol, timeout):
    t0 = state["t"]
    spin_until(lambda: np.linalg.norm(fk(state["q"]).translation - xyz) < tol or state["t"] - t0 > timeout)
    return state["t"] - t0, np.linalg.norm(fk(state["q"]).translation - xyz)


def segment(label, poses, tol=1.5e-3, timeout=12.0):
    """Blended: one path through all poses (stop at the last). Stops: one goal pose after the other."""
    t0 = state["t"]
    if args.mode == "blended":
        msg = PoseArray()
        msg.header.frame_id = "base"
        msg.poses = [pose_msg(xyz, R) for xyz, R in poses]
        path_pub.publish(msg)
        wait_at(poses[-1][0], tol, timeout)
    else:
        for xyz, R in poses:
            goal_pub.publish(pose_msg(xyz, R))
            wait_at(xyz, tol, timeout)
            spin_sim(0.3)
    dt = state["t"] - t0
    err = np.linalg.norm(fk(state["q"]).translation - poses[-1][0])
    print(f"  {label:34s} {dt:5.2f} s  TCP error {err * 1000:5.2f} mm  fingers {state['fingers'] * 1000:5.1f} mm  "
          f"object {np.round(cube_truth()[0], 4)}", flush=True)
    return dt


# --- bring-up
cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
if not cli.wait_for_service(timeout_sec=300):
    print("FAIL: /controller_manager not available")
    finish(2)
if not run(["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"]):
    finish(2)
spin_until(lambda: state["q"] is not None, 60)
spin_until(lambda: os.path.exists(sync("home_done")), 300)
spin_sim(0.2)
for c in ("cartesian_impedance_controller", "gripper"):
    if not run(["ros2", "control", "load_controller", "--set-state", "active", c]):
        finish(2)
spin_sim(1.0)
T_home = fk(state["q"])

here = os.path.dirname(os.path.abspath(__file__))
det_log = open(os.path.join(args.log_dir, "sim_object_detector.log"), "w")
procs.append(subprocess.Popen([sys.executable, os.path.join(here, "sim_object_detector.py"), "--sync-dir", args.sync_dir,
                               "--pos-noise", str(args.pos_noise), "--size-noise", str(args.size_noise),
                               "--yaw-noise-deg", str(args.yaw_noise_deg), "--seed", str(args.seed)],
                              stdout=det_log, stderr=subprocess.STDOUT))
exe = os.path.join(get_package_prefix("ci_mpc_planner"), "lib", "ci_mpc_planner", "mpc_node")
cfg = os.path.join(get_package_share_directory("ci_mpc_planner"), "config", "mpc_sim.yaml")
mpc_log = open(os.path.join(args.log_dir, "mpc_node_pick_place.log"), "w")
procs.append(subprocess.Popen([exe, "--ros-args", "-r", "__node:=ci_mpc_planner", "--params-file", cfg, "-p", "task:=hold"],
                              stdout=mpc_log, stderr=subprocess.STDOUT))
if not spin_until(lambda: "start pose" in open(mpc_log.name).read() and state["det"] is not None, 60):
    print("FAIL: mpc_node or detector did not start:\n" + open(mpc_log.name).read()[-1500:] + open(det_log.name).read()[-1500:])
    finish(2)

# --- grasp from the detection
det = state["det"]
c = det.bbox.center
center = np.array([c.position.x, c.position.y, c.position.z])
obj_yaw = yaw_of([c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z])
size = np.array([det.bbox.size.x, det.bbox.size.y, det.bbox.size.z])
# fingers close along the hand's y axis; at home that is world y, so a hand yaw of obj_yaw + k*90 deg puts the
# fingers across the object's local y (k even) or x (k odd). Take the smallest rotation whose extent fits.
candidates = []
for k in range(-2, 3):
    d = (obj_yaw + k * np.pi / 2 + np.pi) % (2 * np.pi) - np.pi
    across = size[1] if k % 2 == 0 else size[0]
    if across + OPEN_MARGIN <= MAX_OPENING:
        candidates.append((abs(d), d, across))
if not candidates:
    print(f"FAIL: object {np.round(size * 1000)} mm does not fit the gripper")
    finish(1)
_, hand_yaw, across = min(candidates)
R_grasp = pin.exp3(np.array([0, 0, hand_yaw])) @ T_home.rotation
table_z = center[2] - size[2] / 2
z_grasp = max(center[2] + 0.005, table_z + FINGERTIP_BELOW_TCP + 0.005)
opening = min(MAX_OPENING, across + OPEN_MARGIN)
truth_p, truth_q = cube_truth()
print(f"DETECTION center {np.round(center, 4)} (truth {np.round(truth_p, 4)}, error {np.linalg.norm(center - truth_p) * 1000:.1f} mm), "
      f"size {np.round(size * 1000, 1)} mm, yaw {np.degrees(obj_yaw):.1f} deg (truth {np.degrees(yaw_of(truth_q)):.1f})", flush=True)
print(f"GRASP hand yaw {np.degrees(hand_yaw):+.1f} deg, across the fingers {across * 1000:.1f} mm, opening {opening * 1000:.1f} mm, "
      f"TCP z {z_grasp:.4f}; motion mode {args.mode}", flush=True)

pick_xy = center[:2]
place_xyz = np.array([*PLACE, z_grasp + 0.002])
t_start = state["t"]
gripper(opening, settle=0.5)
t_a = segment("A home -> above -> grasp", [(np.array([*pick_xy, Z_SAFE]), R_grasp), (np.array([*pick_xy, z_grasp]), R_grasp)])
gripper(0.0, settle=1.0)
closed = state["fingers"]
grasped = abs(closed - across) < GRASP_TOL
print(f"  gripper closed: {closed * 1000:.1f} mm (expected {across * 1000:.1f} ± {GRASP_TOL * 1000:.0f}) -> "
      f"{'grasped' if grasped else 'MISSED'}", flush=True)
z0 = truth_p[2]
t_b = segment("B lift -> above place -> place", [(np.array([*pick_xy, Z_SAFE]), R_grasp),
                                                (np.array([*PLACE, Z_SAFE]), R_grasp), (place_xyz, R_grasp)])
gripper(opening, settle=0.8)
t_c = segment("C up -> home", [(np.array([*PLACE, Z_SAFE]), R_grasp), (T_home.translation, T_home.rotation)])
end_p, end_q = cube_truth()
total = state["t"] - t_start
placed_err = np.linalg.norm(end_p[:2] - PLACE)
yaw_change = np.degrees((yaw_of(end_q) - yaw_of(truth_q) + np.pi) % (2 * np.pi) - np.pi)
on_table = abs(end_p[2] - z0) < 0.005
print(f"RESULT grasp {'OK' if grasped else 'FAIL'}, placed {placed_err * 1000:.1f} mm from the target "
      f"(object yaw changed {yaw_change:+.1f} deg), on the table {'OK' if on_table else 'FAIL'}; "
      f"motion {t_a + t_b + t_c:.1f} s, total {total:.1f} s sim ({args.mode})", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if grasped and placed_err < 0.005 and on_table else 1)
