#!/usr/bin/env python3
# Box stacking demo with the stage-1 MPC in sim (ROS side; run via run_impedance_test.sh with PICK_PLACE=1
# BOXES='x,y,yaw;...' CONTROLLER_CONFIG=.../ci_mpc_sim_gripper/config/isaac_pick_place.yaml CHECKER=this script;
# scripts/demo_stack.sh does that).
#
# The boxes come from vision_msgs/Detection3DArray on /perception/detections (id = box name; sim stand-in
# scripts/sim_object_detector.py with optional estimation error). The base box (--base, default box1) stays;
# the others (--order) are stacked on it one after the other. Every round re-reads the detections:
#   grasp   = as in pick_place_demo.py: box center, hand yaw = box yaw (90-degree multiple closest to home),
#             opening = extent across the fingers + 2 cm;
#   place   = on the box currently on top (detected pose): xy of its center, bottom of the carried box 2 mm above
#             its top, hand yaw turned so the carried box's yaw matches it (mod 90 deg, hand within +-45 deg of home);
#   motion  = blended MPC paths (lift -> above the stack -> place), safe height above the current stack.
# Score: per level xy offset to the box below and to the base, yaw difference, height (standing / fallen).
import argparse
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
p.add_argument("--base", default="box1", help="box that stays on the table")
p.add_argument("--order", default="box0,box2", help="boxes to stack, in this order")
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
node = Node("stack_demo")
state = {"q": None, "t": 0.0, "fingers": None, "det": None, "dets": {}}


def on_js(msg):
    if all(n in msg.name for n in ARM):
        state["q"] = np.array([msg.position[msg.name.index(n)] for n in ARM])
        state["t"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if "fr3_finger_joint1" in msg.name:
            state["fingers"] = sum(msg.position[msg.name.index(f"fr3_finger_joint{i}")] for i in (1, 2))


def on_det(msg):
    if msg.detections:
        state["det"] = msg.detections[0]
        state["dets"] = {d.id: d for d in msg.detections}


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


def truth():
    """Ground truth {name: (center, yaw)} from the sim (scoring only)."""
    try:
        return {o["name"]: (np.array(o["cube"]), yaw_of(o["quat_wxyz"]))
                for o in json.load(open(sync("object.json")))["objects"]}
    except (OSError, ValueError, KeyError):
        return {}


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
    print(f"  {label:34s} {dt:5.2f} s  TCP error {err * 1000:5.2f} mm  fingers {state['fingers'] * 1000:5.1f} mm",
          flush=True)
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
mpc_log = open(os.path.join(args.log_dir, "mpc_node_stack.log"), "w")
procs.append(subprocess.Popen([exe, "--ros-args", "-r", "__node:=ci_mpc_planner", "--params-file", cfg, "-p", "task:=hold"],
                              stdout=mpc_log, stderr=subprocess.STDOUT))
if not spin_until(lambda: "start pose" in open(mpc_log.name).read() and state["det"] is not None, 60):
    print("FAIL: mpc_node or detector did not start:\n" + open(mpc_log.name).read()[-1500:] + open(det_log.name).read()[-1500:])
    finish(2)

# --- stacking
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def box(name):
    d = state["dets"][name]
    c = d.bbox.center
    return (np.array([c.position.x, c.position.y, c.position.z]),
            yaw_of([c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z]),
            np.array([d.bbox.size.x, d.bbox.size.y, d.bbox.size.z]))


order = [n for n in args.order.split(",") if n]
missing = [n for n in [args.base] + order if n not in state["dets"]]
if missing:
    print(f"FAIL: not detected: {missing} (detected {sorted(state['dets'])})")
    finish(2)
home_yaw = 0.0  # hand yaw is measured relative to the home orientation
stack = [args.base]
t_start = state["t"]
t_motion = 0.0
ok = True
truth0 = truth()
for name in order:
    spin_sim(0.3)  # fresh detections
    center, obj_yaw, size = box(name)
    top_c, top_yaw, top_size = box(stack[-1])
    # grasp: smallest hand rotation from home whose extent fits
    candidates = []
    for k in range(-2, 3):
        d = wrap(obj_yaw + k * np.pi / 2)
        across = size[1] if k % 2 == 0 else size[0]
        if across + OPEN_MARGIN <= MAX_OPENING:
            candidates.append((abs(d), d, across))
    if not candidates:
        print(f"FAIL: {name} {np.round(size * 1000)} mm does not fit the gripper")
        finish(1)
    _, hand_yaw, across = min(candidates)
    # place: turn the hand so the carried box's yaw matches the top box (mod 90 deg), hand within +-45 deg
    hand_place = min((wrap(hand_yaw + top_yaw - obj_yaw + k * np.pi / 2) for k in range(-2, 3)), key=abs)
    R_grasp = pin.exp3(np.array([0, 0, hand_yaw])) @ T_home.rotation
    R_place = pin.exp3(np.array([0, 0, hand_place])) @ T_home.rotation
    table_z = center[2] - size[2] / 2
    z_grasp = max(center[2] + 0.005, table_z + FINGERTIP_BELOW_TCP + 0.005)
    tcp_above_center = z_grasp - center[2]
    place_center_z = top_c[2] + top_size[2] / 2 + size[2] / 2 + 0.002
    place_xyz = np.array([top_c[0], top_c[1], place_center_z + tcp_above_center])
    stack_top = top_c[2] + top_size[2] / 2
    z_safe = max(Z_SAFE, stack_top + size[2] + 0.05)
    opening = min(MAX_OPENING, across + OPEN_MARGIN)
    print(f"{name} -> on {stack[-1]}: grasp at {np.round(center, 4)} yaw {np.degrees(obj_yaw):+.1f} deg (hand "
          f"{np.degrees(hand_yaw):+.1f}), place at {np.round(place_xyz, 4)} on top yaw {np.degrees(top_yaw):+.1f} "
          f"(hand {np.degrees(hand_place):+.1f}), safe z {z_safe:.3f}", flush=True)
    pick_xy = center[:2]
    gripper(opening, settle=0.5)
    t_motion += segment(f"A -> above {name} -> grasp", [(np.array([*pick_xy, z_safe]), R_grasp),
                                                   (np.array([*pick_xy, z_grasp]), R_grasp)])
    gripper(0.0, settle=1.0)
    closed = state["fingers"]
    grasped = abs(closed - across) < GRASP_TOL
    print(f"  gripper closed: {closed * 1000:.1f} mm (expected {across * 1000:.1f} ± {GRASP_TOL * 1000:.0f}) -> "
          f"{'grasped' if grasped else 'MISSED'}", flush=True)
    if not grasped:
        ok = False
        break
    t_motion += segment(f"B lift -> above {stack[-1]} -> place", [(np.array([*pick_xy, z_safe]), R_grasp),
                                                             (np.array([*place_xyz[:2], z_safe]), R_place),
                                                             (place_xyz, R_place)])
    gripper(opening, settle=0.8)
    t_motion += segment("C up", [(np.array([*place_xyz[:2], z_safe]), R_place)])
    spin_sim(0.5)
    tr = truth()
    below_c, below_yaw = tr[stack[-1]]
    c, yaw = tr[name]
    off = np.linalg.norm(c[:2] - below_c[:2])
    level_ok = abs(c[2] - (below_c[2] + size[2])) < 0.005 and off < 0.01
    print(f"  placed {name}: {off * 1000:.1f} mm from {stack[-1]}'s center, yaw diff "
          f"{np.degrees(wrap(4 * (yaw - below_yaw)) / 4):+.1f} deg (mod 90), height {c[2]:.4f} -> "
          f"{'OK' if level_ok else 'FAIL'}", flush=True)
    stack.append(name)
    ok = ok and level_ok
t_motion += segment("D home", [(T_home.translation, T_home.rotation)])
spin_sim(1.0)
tr = truth()
base_c = tr[args.base][0]
print("TOWER (truth): " + "; ".join(
    f"{n} dz {(tr[n][0][2] - base_c[2]) * 1000:5.1f} mm, xy off base {np.linalg.norm(tr[n][0][:2] - base_c[:2]) * 1000:4.1f} mm"
    for n in stack), flush=True)
for i in range(1, len(stack)):
    lo, hi = tr[stack[i - 1]][0], tr[stack[i]][0]
    if not (abs(hi[2] - lo[2] - size[2]) < 0.005 and np.linalg.norm(hi[:2] - lo[:2]) < 0.01):
        ok = False
moved_base = np.linalg.norm(base_c[:2] - truth0[args.base][0][:2])
print(f"RESULT stack of {len(stack)} {'OK' if ok else 'FAIL'}; base moved {moved_base * 1000:.1f} mm; "
      f"motion {t_motion:.1f} s, total {state['t'] - t_start:.1f} s sim ({args.mode})", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if ok else 1)
