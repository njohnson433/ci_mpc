#!/usr/bin/env python3
# Pick-and-place demo with the stage-1 MPC in sim (ROS side; run via run_impedance_test.sh with
# PICK_PLACE=1 CHECKER=this script, GUI=1 to watch). Starts mpc_node with config/mpc_sim.yaml and sends goal
# poses on /ci_mpc_planner/goal_pose; the gripper is the sim stand-in controller ci_mpc_sim_gripper
# (/gripper/width, needs CONTROLLER_CONFIG=.../ci_mpc_sim_gripper/config/isaac_pick_place.yaml), the cube pose
# comes from <sync>/object.json.
#   open -> above pick -> down -> close -> lift -> above place -> down -> open -> up -> home
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
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
args = p.parse_args()

PICK, PLACE = np.array([0.45, -0.12]), np.array([0.45, 0.12])
Z_GRASP, Z_SAFE = 0.225, 0.33  # TCP height at the grasp (cube center 0.22) and for transport
CUBE_EDGE = 0.04

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


mpc_proc = None


def finish(code):
    if mpc_proc is not None and mpc_proc.poll() is None:
        mpc_proc.send_signal(signal.SIGINT)
        try:
            mpc_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            mpc_proc.kill()
    open(sync("done"), "w").close()
    sys.exit(code)


def run(cmd):
    print("$", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    print((r.stdout + r.stderr).strip(), flush=True)
    return r.returncode == 0 and "rror" not in r.stdout + r.stderr


rclpy.init()
node = Node("pick_place_demo")
state = {"q": None, "t": 0.0, "fingers": None}


def on_js(msg):
    if all(n in msg.name for n in ARM):
        state["q"] = np.array([msg.position[msg.name.index(n)] for n in ARM])
        state["t"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if "fr3_finger_joint1" in msg.name:
            state["fingers"] = sum(msg.position[msg.name.index(f"fr3_finger_joint{i}")] for i in (1, 2))


node.create_subscription(JointState, "/joint_states", on_js, 100)
goal_pub = node.create_publisher(Pose, "/ci_mpc_planner/goal_pose", 10)
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


def cube():
    try:
        return np.array(json.load(open(sync("object.json")))["cube"])
    except (OSError, ValueError, KeyError):
        return np.full(3, np.nan)


def gripper(width, settle=1.0):
    width_pub.publish(Float64(data=float(width)))
    spin_sim(settle)


def move(label, xyz, tol=1.5e-3, timeout=8.0):
    """Send a goal pose (home orientation) to the MPC and wait until the TCP is within tol (sim time)."""
    m = Pose()
    m.position.x, m.position.y, m.position.z = map(float, xyz)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q_home.x, q_home.y, q_home.z, q_home.w
    goal_pub.publish(m)
    t0 = state["t"]
    reached = spin_until(lambda: np.linalg.norm(fk(state["q"]).translation - xyz) < tol or state["t"] - t0 > timeout)
    err = np.linalg.norm(fk(state["q"]).translation - xyz)
    dt = state["t"] - t0
    spin_sim(0.3)  # let it settle a little
    print(f"  {label:22s} {dt:5.2f} s  TCP error {err * 1000:5.2f} mm  TCP z {fk(state['q']).translation[2]:.4f}  "
          f"fingers {state['fingers'] * 1000:5.1f} mm  cube {np.round(cube(), 4)}", flush=True)
    log.append((label, dt, err))
    return reached


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
if not run(["ros2", "control", "load_controller", "--set-state", "active", "cartesian_impedance_controller"]):
    finish(2)
if not run(["ros2", "control", "load_controller", "--set-state", "active", "gripper"]):
    finish(2)
spin_sim(1.0)
T_home = fk(state["q"])
q_home = pin.Quaternion(T_home.rotation)
print(f"home TCP {np.round(T_home.translation, 4)}, cube {np.round(cube(), 4)}", flush=True)

exe = os.path.join(get_package_prefix("ci_mpc_planner"), "lib", "ci_mpc_planner", "mpc_node")
cfg = os.path.join(get_package_share_directory("ci_mpc_planner"), "config", "mpc_sim.yaml")
mpc_log = open(os.path.join(args.log_dir, "mpc_node_pick_place.log"), "w")
mpc_proc = subprocess.Popen([exe, "--ros-args", "-r", "__node:=ci_mpc_planner", "--params-file", cfg, "-p", "task:=hold"],
                            stdout=mpc_log, stderr=subprocess.STDOUT)
if not spin_until(lambda: "start pose" in open(mpc_log.name).read() or mpc_proc.poll() is not None, 60) or mpc_proc.poll() is not None:
    print("FAIL: mpc_node did not start:\n" + open(mpc_log.name).read()[-2000:])
    finish(2)

log = []
cube0 = cube()
print("PICK AND PLACE (MPC goals; times are sim time to reach the goal within 1.5 mm)", flush=True)
t_start = state["t"]
gripper(0.08, settle=0.5)
move("above pick", [*PICK, Z_SAFE])
move("down to grasp", [*PICK, Z_GRASP])
gripper(0.0, settle=1.0)
w_grasp = state["fingers"]
print(f"  gripper closed: finger opening {w_grasp * 1000:.1f} mm (cube {CUBE_EDGE * 1000:.0f} mm)", flush=True)
move("lift", [*PICK, Z_SAFE])
c_lift = cube()
move("above place", [*PLACE, Z_SAFE])
c_carry = cube()
move("down to place", [*PLACE, Z_GRASP + 0.002])
gripper(0.08, settle=0.8)
move("up", [*PLACE, Z_SAFE])
move("home", T_home.translation)
c_end = cube()
total = state["t"] - t_start

grasped = abs(w_grasp - CUBE_EDGE) < 0.005
lifted = c_lift[2] - cube0[2] > 0.08
carried = np.linalg.norm(c_carry[:2] - PLACE) < 0.01
placed_err = np.linalg.norm(c_end[:2] - PLACE)
on_table = abs(c_end[2] - cube0[2]) < 0.005
print(f"RESULT grasp {'OK' if grasped else 'FAIL'} (opening {w_grasp * 1000:.1f} mm), lifted {'OK' if lifted else 'FAIL'} "
      f"(+{(c_lift[2] - cube0[2]) * 1000:.0f} mm), carried {'OK' if carried else 'FAIL'}, placed {placed_err * 1000:.1f} mm "
      f"from the target, on the table {'OK' if on_table else 'FAIL'}; total {total:.1f} s sim", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if grasped and lifted and carried and placed_err < 0.005 and on_table else 1)
