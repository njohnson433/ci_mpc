#!/usr/bin/env python3
# Factory peg-in-hole with our controller in sim (ROS side; run via run_impedance_test.sh with PEG=1
# CONTROLLER_CONFIG=.../ci_mpc_sim_gripper/config/isaac_pick_place.yaml CHECKER=this script; scripts/demo_peg.sh).
#
# Free space (stage-1 MPC, blended paths): grasp the 8 mm peg from the detection (TCP 10 mm below its top, fingers
# closed), carry it above the detected hole (tip 10 mm above the opening). --hole-offset dx,dy [mm] adds a
# perception error to the hole position.
# Contact phase (this script commands the impedance controller directly, the MPC node is paused via ~/active):
#   descend   at --descend-speed with soft z stiffness (--stiffness, default 1500 / 1500 / 500 N/m) until the tip lags
#             the command by 3 mm (contact with the hole block or the chamfer);
#   search    press down (feedforward --push N + 2 mm spring) and move the command on an Archimedean spiral
#             (--pitch, --search-speed, up to --search-radius) around the detected hole; the lateral compliance lets
#             the chamfer pull the peg in;
#   insert    once the tip is --engage below the opening: keep the xy where it is, soft laterally (--insert-stiffness)
#             so the hole aligns the peg, push down until it stalls at the bottom;
#   release   open the gripper, MPC resumes, lift.
# --dither amp,freq adds a small circular lateral motion in search and insert (breaks joint stiction).
# Integral off in contact. Success (Factory): peg bottom within 1 mm (4 % of the 25 mm depth) of the hole bottom,
# from the sim ground truth. Contact force estimate: K (x_cmd - x) + feedforward (no F/T sensor yet).
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
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Float64
from vision_msgs.msg import Detection3DArray

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
p.add_argument("--mode", default="blended", choices=["blended", "stops"])
p.add_argument("--hole-offset", default="0,0", help="perception error of the hole position dx,dy [mm]")
p.add_argument("--stiffness", default="1500,1500,500", help="translational stiffness in contact [N/m]")
p.add_argument("--insert-stiffness", default="500,500,500",
               help="translational stiffness once the peg is in the hole (soft laterally: the hole aligns the peg)")
p.add_argument("--push", type=float, default=2.0, help="feedforward push in contact [N]")
p.add_argument("--descend-speed", type=float, default=0.01)
p.add_argument("--pitch", type=float, default=0.0006, help="spiral pitch [m]")
p.add_argument("--search-speed", type=float, default=0.004, help="spiral speed [m/s]")
p.add_argument("--search-radius", type=float, default=0.005)
p.add_argument("--dither", default="0,0", help="lateral dither amplitude [m], frequency [Hz] in search and insert "
               "(keeps the joints moving, so the velocity-based friction compensation is active: no stiction)")
p.add_argument("--engage", type=float, default=0.003, help="tip this far below the opening = in the hole [m]")
p.add_argument("--timeout-contact", type=float, default=30.0, help="sim seconds for the contact phase")
p.add_argument("--out", default="")
p.add_argument("--probe", action="store_true", help="after inserting: hold and push with known feedforward forces "
               "(checks the external wrench estimate and the PhysX contact report against each other)")
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
node = Node("peg_insert_demo")
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
wrench_log = []  # (joint_states time, Franka-sign external wrench estimate)
node.create_subscription(WrenchStamped, "/cartesian_impedance_controller/external_wrench",
                         lambda m: wrench_log.append((state["t"], m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)), 100)
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
    """{name: object dict} from the sim (scoring only)."""
    try:
        return {o["name"]: o for o in json.load(open(sync("object.json")))["objects"]}
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
mpc_log = open(os.path.join(args.log_dir, "mpc_node_peg.log"), "w")
procs.append(subprocess.Popen([exe, "--ros-args", "-r", "__node:=ci_mpc_planner", "--params-file", cfg, "-p", "task:=hold"],
                              stdout=mpc_log, stderr=subprocess.STDOUT))
if not spin_until(lambda: "start pose" in open(mpc_log.name).read() and state["det"] is not None, 60):
    print("FAIL: mpc_node or detector did not start:\n" + open(mpc_log.name).read()[-1500:] + open(det_log.name).read()[-1500:])
    finish(2)

# --- peg-in-hole
def parse(d):
    c = d.bbox.center
    return np.array([c.position.x, c.position.y, c.position.z]), np.array([d.bbox.size.x, d.bbox.size.y, d.bbox.size.z])


spin_sim(0.5)
if not {"peg", "hole"} <= set(state["dets"]):
    print(f"FAIL: peg / hole not detected ({sorted(state['dets'])})")
    finish(2)
peg_c, peg_size = parse(state["dets"]["peg"])
hole_top, _ = parse(state["dets"]["hole"])  # center of the opening (top face)
hole_top[2] += 0.0  # the detection's z is the opening
offset = np.array([float(v) for v in args.hole_offset.split(",")] + [0.0]) / 1000.0
hole_est = hole_top + offset
tr0 = truth()
R = T_home.rotation
peg_top = peg_c[2] + peg_size[2] / 2
z_grasp = peg_top - 0.010
tip_below_tcp = z_grasp - (peg_c[2] - peg_size[2] / 2)
print(f"PEG at {np.round(peg_c, 4)}, HOLE opening at {np.round(hole_top, 4)}, perception error "
      f"{np.round(offset[:2] * 1000, 2)} mm -> hole estimate {np.round(hole_est, 4)}; tip {tip_below_tcp * 1000:.1f} mm "
      f"below the TCP", flush=True)
t_start = state["t"]
gripper(0.03, settle=0.5)
segment("A home -> above peg -> grasp", [(np.array([*peg_c[:2], Z_SAFE]), R), (np.array([*peg_c[:2], z_grasp]), R)])
gripper(0.0, settle=1.0)
closed = state["fingers"]
if abs(closed - peg_size[0]) > 0.003:
    print(f"FAIL: grasp, fingers {closed * 1000:.1f} mm (peg {peg_size[0] * 1000:.1f} mm)")
    finish(1)
tr = truth()
in_hand = np.array(tr["peg"]["origin"])[:2] - fk(state["q"]).translation[:2]
print(f"  grasped: fingers {closed * 1000:.1f} mm; peg bottom {np.round(in_hand * 1000, 2)} mm from the TCP in xy "
      f"(truth, unknown to the controller)", flush=True)
above = np.array([*hole_est[:2], hole_est[2] + tip_below_tcp + 0.010])
segment("B lift -> above the hole", [(np.array([*peg_c[:2], Z_SAFE]), R), (np.array([*hole_est[:2], Z_SAFE]), R), (above, R)])
spin_sim(0.3)

# contact phase: command the impedance controller directly
active_pub = node.create_publisher(Bool, "/ci_mpc_planner/active", 10)
cmd_pub = node.create_publisher(MpcCommand, "/cartesian_impedance_controller/mpc_command", 10)
dither = [float(v) for v in args.dither.split(",")]
K_search = [float(v) for v in args.stiffness.split(",")] + [260.0, 260.0, 90.0]
K_insert = [float(v) for v in args.insert_stiffness.split(",")] + [260.0, 260.0, 90.0]
quat = pin.Quaternion(R)
c = {"phase": "descend", "p": fk(state["q"]).translation.copy(), "v": np.zeros(3), "t": state["t"], "t0": state["t"],
     "theta": 0.0, "z_contact": None, "z_hist": [], "log": [], "peak_f": 0.0, "t_contact": None, "t_engage": None, "t_done": None}
c["p"][:2] = hole_est[:2]
hole_tip_z = hole_est[2]


def command():
    now = state["t"]
    dt = min(max(now - c["t"], 0.0), 0.02)
    c["t"] = now
    x = fk(state["q"]).translation
    tip = x[2] - tip_below_tcp
    p_prev = c["p"].copy()
    ff = 0.0
    if c["phase"] == "descend":
        c["p"][2] -= args.descend_speed * dt
        c["z_hist"].append((now, x[2]))
        while c["z_hist"] and now - c["z_hist"][0][0] > 0.2:
            c["z_hist"].pop(0)
        vz = (x[2] - c["z_hist"][0][1]) / max(now - c["z_hist"][0][0], 1e-3)
        # contact: the tip lags by 3 mm and has stopped moving down (a lag alone also comes from joint stiction
        # at the start of the descent with the soft z spring)
        # and the tip is near the expected opening (the arm also stands still in its joint stiction when the
        # descent starts: false contact 10 mm above the hole without this gate)
        if (c["p"][2] < x[2] - 0.003 and abs(vz) < 0.2 * args.descend_speed and now - c["t0"] > 0.3
                and tip < hole_tip_z + 0.005):
            c["phase"], c["t_contact"], c["z_contact"] = "search", now, x[2]
            print(f"  contact at tip z {tip * 1000:.1f} mm ({(tip - hole_tip_z) * 1000:+.1f} mm vs the opening), "
                  f"{now - c['t0']:.2f} s", flush=True)
    if c["phase"] == "search":
        ff = -args.push
        # Archimedean spiral r = b theta at constant speed: dtheta = v / sqrt(r^2 + b^2)
        b = args.pitch / (2 * np.pi)
        r = b * c["theta"]
        c["theta"] += args.search_speed * dt / np.hypot(r, b)
        r = min(b * c["theta"], args.search_radius)
        c["p"][:2] = hole_est[:2] + r * np.array([np.cos(c["theta"]), np.sin(c["theta"])])
        c["p"][2] = min(c["z_contact"], x[2]) - 0.002
        if tip < hole_tip_z - args.engage:
            c["phase"], c["t_engage"], c["z_hist"] = "insert", now, []
            c["p"][:2] = x[:2]
            print(f"  engaged after {now - c['t_contact']:.2f} s of search (spiral r {r * 1000:.2f} mm), "
                  f"xy {np.round((x[:2] - hole_top[:2]) * 1000, 2)} mm from the true opening", flush=True)
        elif b * c["theta"] > args.search_radius + args.pitch:
            c["phase"] = "failed"
            print("  search radius exhausted", flush=True)
    if c["phase"] == "insert":
        ff = -args.push
        c["p"][2] = min(c["p"][2], x[2]) - args.descend_speed * dt
        c["p"][2] = max(c["p"][2], x[2] - 0.006)  # spring at most 6 mm below the TCP
        depth = hole_tip_z - tip
        c["z_hist"].append((now, x[2]))
        while c["z_hist"] and now - c["z_hist"][0][0] > 0.3:
            c["z_hist"].pop(0)
        stalled = now - c["t_engage"] > 0.5 and abs(x[2] - c["z_hist"][0][1]) < 1e-4  # < 0.1 mm in 0.3 s
        if (stalled and depth > 0.015) or now - c["t_engage"] > 8.0:  # at the bottom (or stuck)
            c["phase"], c["t_done"] = "done", now
            print(f"  insert {'stalled at depth' if stalled else 'timed out at depth'} {depth * 1000:.1f} mm "
                  f"after {now - c['t_engage']:.2f} s", flush=True)
    if c["phase"] in ("done", "failed"):
        ff = 0.0
    ff_xy = np.zeros(2)
    if c["phase"] == "probe":
        ff_xy, ff = c["probe_f"][:2], c["probe_f"][2]
    c["v"] = (c["p"] - p_prev) / dt if dt > 0 else np.zeros(3)
    p_out, v_out = c["p"].copy(), c["v"].copy()
    if c["phase"] in ("search", "insert") and dither[0] > 0:
        w = 2 * np.pi * dither[1]
        ph = w * (now - c["t0"])
        p_out[:2] += dither[0] * np.array([np.cos(ph), np.sin(ph)])
        v_out[:2] += dither[0] * w * np.array([-np.sin(ph), np.cos(ph)])
    m = MpcCommand()
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_out)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat.x, quat.y, quat.z, quat.w
    m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v_out)
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = float(ff_xy[0]), float(ff_xy[1]), float(ff)
    K = K_insert if c["phase"] in ("insert", "done") else K_search
    m.stiffness = K
    m.integral_enabled = False
    cmd_pub.publish(m)
    f_est = np.array(K[:3]) * (c["p"] - x) + np.array([*ff_xy, ff])
    c["peak_f"] = max(c["peak_f"], float(np.linalg.norm(f_est)))
    c["log"].append((now, x.copy(), c["p"].copy(), f_est, c["phase"]))


active_pub.publish(Bool(data=False))
spin_sim(0.05)
timer = node.create_timer(0.005, command)
spin_until(lambda: c["phase"] in ("done", "failed") or state["t"] - c["t0"] > args.timeout_contact, 600)
if c["phase"] not in ("done", "failed"):
    c["phase"] = "failed"
    print(f"  contact phase timed out ({c['log'][-1][4] if c['log'] else '?'})", flush=True)
spin_sim(0.3)
if args.probe and c["phase"] == "done":
    c["p"] = fk(state["q"]).translation.copy()  # hold here; the spring adds nothing at rest
    print("PROBE (feedforward force, held 2 s each; times on the joint_states clock):", flush=True)
    for fvec in ([0, 0, -8.0], [4.0, 0, -6.0], [0, -4.0, -6.0], [0, 0, -2.0]):
        c["probe_f"], c["phase"] = np.array(fvec), "probe"
        t_a = state["t"]
        spin_sim(2.0)
        print(f"  probe {fvec} from {t_a + 1.0:.2f} to {state['t']:.2f}", flush=True)
    c["phase"] = "done"
tr = truth()
peg_o, hole_o = np.array(tr["peg"]["origin"]), np.array(tr["hole"]["origin"])  # peg bottom, hole bottom
depth_err = peg_o[2] - hole_o[2]
xy_err = np.linalg.norm(peg_o[:2] - hole_o[:2])
gripper(0.03, settle=0.5)
node.destroy_timer(timer)
active_pub.publish(Bool(data=True))
spin_sim(0.2)
x = fk(state["q"]).translation
segment("C release -> up", [(np.array([*x[:2], Z_SAFE]), R)])
tr = truth()
peg_o = np.array(tr["peg"]["origin"])
stays = abs(peg_o[2] - hole_o[2]) < 0.0015
success = depth_err < 0.001 and stays
t_search = (c["t_engage"] - c["t_contact"]) if c["t_engage"] and c["t_contact"] else float("nan")
t_insert = (c["t_done"] - c["t_engage"]) if c["t_done"] and c["t_engage"] else float("nan")
res = {"success": bool(success), "hole_offset_mm": [float(v) for v in offset[:2] * 1000],
       "depth_err_mm": float(depth_err * 1000), "xy_err_mm": float(xy_err * 1000),
       "search_s": float(t_search), "insert_s": float(t_insert),
       "contact_phase_s": float((c["t_done"] or state["t"]) - c["t0"]), "peak_force_est_N": c["peak_f"],
       "total_s": float(state["t"] - t_start)}
print("RESULT " + ", ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in res.items()), flush=True)
if args.out:
    json.dump(res, open(args.out, "w"), indent=1)
np.save(os.path.join(args.log_dir, "external_wrench.npy"), np.array(wrench_log))
import shutil
for fname in ("hole_contact.csv", "wrist_ft.csv"):
    if os.path.exists(sync(fname)):
        shutil.copy(sync(fname), os.path.join(args.log_dir, fname))
np.savez(os.path.join(args.log_dir, "peg_contact.npz"), t=[l[0] for l in c["log"]], x=[l[1] for l in c["log"]],
         p=[l[2] for l in c["log"]], f=[l[3] for l in c["log"]], phase=[l[4] for l in c["log"]])
node.destroy_node()
rclpy.shutdown()
finish(0 if success else 1)
