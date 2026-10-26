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
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Float64
from vision_msgs.msg import Detection3DArray

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
p.add_argument("--mode", default="blended", choices=["blended", "stops"])
p.add_argument("--grasp-offset", default="0,0", help="grasp off the nominal point dx (hand x, across the finger axis), "
               "dz (deeper +) [mm]; the controller keeps assuming the nominal grasp (held-out variation)")
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
p.add_argument("--max-probes", type=int, default=12)
p.add_argument("--engage-lateral", type=float, default=1.2,
               help="probe: a lateral force above this also counts as engaged [N] (inf: tip sink only; with a "
                    "calibrated grasp height ~22 %% of flat-face probes read > 1.2 N and started a GPU phase)")
p.add_argument("--lateral-centering", action="store_true",
               help="probe --entry gpu: a lateral push without tip sink centers (as plain probe) instead of starting "
                    "the GPU phase; only a sink hands over to the GPU MPC (B4: pushes started GPU phases, 0/6 at 5 mm; "
                    "B6, pushes ignored: out of probes, the push is the cue that walks plain probe to the hole)")
p.add_argument("--track-height", type=int, default=0,
               help="probe: re-estimate the grasp height offset as the median raw tip height of the last N flat-face "
                    "probes (0 = off; the peg creeps ~0.005-0.01 mm per probe in the fingers, long searches reached the "
                    "0.3 mm sink threshold on the flat face)")
p.add_argument("--belief-model", default="",
               help="probe: learned probe likelihood (.npz from scripts/train_probe_likelihood.py) instead of the hand "
                    "measurement model in the hole belief")
p.add_argument("--belief-scale", type=float, default=1.0, help="temper the learned log-likelihood (< 1: softer)")
p.add_argument("--collect", type=int, default=0,
               help="probe: data collection for the learned probe likelihood: N probes at sampled positions around the "
                    "TRUE hole (distance 0-9 mm, half of them in the 2-6 mm chamfer band), random hold depth and settle "
                    "time, no insertion; logged as probe lines (scripts/probe_dataset.py)")
p.add_argument("--collect-seed", type=int, default=0)
p.add_argument("--centering-bound", type=float, default=0.0,
               help="probe: stop centering, and do not take a drop as 'in the hole', beyond this distance from the "
                    "estimate [m] (0 = off; hole errors are <= 5.4 mm; a misread push drove centering off the block "
                    "edge twice, B9 rep3 / A4 rep1 0,4)")
p.add_argument("--force-debias", type=int, default=0,
               help="learned belief: subtract the median lateral force of the last N flat probes of the run before "
                    "the likelihood (0 = off; the estimate's lateral bias is ~constant within a run, ~0.6-0.9 N, vs a "
                    "0.17 N chamfer signal, and the belief treats probes as independent: A4 rep1 0,4 drifted)")
p.add_argument("--stuck-dither", type=float, default=0.0,
               help="probe: when the last 3 probes landed within 0.4 mm of each other, offset the next target by this "
                    "much in a random direction [m] (0 = off; A6 rep3 -5,0: the belief locked on at std 0 with the peg "
                    "held by stiction on the chamfer edge, every probe returned to the same spot until time ran out)")
p.add_argument("--jam-recovery", type=int, default=0,
               help="probe: retries when the push to the bottom stalls (lift 1 mm, re-press shifted along the lateral "
                    "contact force; 0 = off; H1 friction 1.0 0.63,1.57 wedged 13 mm short twice, H3 7 of 12)")
p.add_argument("--jam-shift", type=float, default=0.0003, help="lateral shift per jam retry [m]")
p.add_argument("--jam-lift", type=float, default=0.001, help="jam: lift before pressing again [m]")
p.add_argument("--jam-force", type=float, default=3.0, help="jam: lateral force (1 s mean) that counts as wedged [N]")
p.add_argument("--gpu-std-max", type=float, default=float("inf"),
               help="probe --entry gpu: start the GPU phase only when the belief std is below this [m]; otherwise center "
                    "as plain probe (the MPC drives toward the belief estimate: with a wide belief it moved the peg out of "
                    "a chamfer the centering would have followed in, RB_v3 0.74,1.19)")
p.add_argument("--calib-touch", type=float, default=None,
               help="probe: touch the flat top face this far from the hole estimate (diagonal) to calibrate the grasp "
                    "height before probing [m] (default 0.0095 for the Factory peg, 0 = off, i.e. calibrate on probe 0 "
                    "only if it lands flat; with --geo: off)")
p.add_argument("--entry", default="follow", choices=["follow", "gpu"],
               help="probe strategy, engaged phase: follow = force-following centering; gpu = GPU sampling MPC "
                    "(scripts/gpu_mpc_server.py in the Isaac venv, started by this script)")
p.add_argument("--strategy", default="spiral", choices=["spiral", "estimate", "c3", "probe"],
               help="contact phase: spiral search (hand-written); estimate = hole localization from contact "
                    "(particle filter on the measured wrench) + straight down; c3 = localization + C3+ CI-MPC; "
                    "probe = 3D hole belief (x and y) + information-driven probing + chamfer centering")
p.add_argument("--particles", type=int, default=30)
p.add_argument("--follow-force", type=float, default=0.0,
               help="[m/N per step] admittance along the measured lateral contact force (the round rim pushes the "
                    "peg towards the hole axis); 0 = off")
p.add_argument("--lateral-stiction-ff", type=float, default=2.0,
               help="[N] feedforward along the measured lateral contact force when following it")
p.add_argument("--stiction-ff", type=float, default=3.0,
               help="[N] feedforward force along the descent that cancels the arm's static friction in z (estimate "
                    "strategy; 3 N of spring alone did not move the arm in free space)")
p.add_argument("--max-contact-force", type=float, default=4.0,
               help="[N] stop pressing further when the measured contact force exceeds this (the peg slid in the "
                    "fingers at 6-8 N)")
p.add_argument("--plan-period", type=float, default=0.1, help="[s] sim time per planning step")
p.add_argument("--probe", action="store_true", help="after inserting: hold and push with known feedforward forces "
               "(checks the external wrench estimate and the PhysX contact report against each other)")
p.add_argument("--pos-noise", type=float, default=0.0)
p.add_argument("--size-noise", type=float, default=0.0)
p.add_argument("--yaw-noise-deg", type=float, default=0.0)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--grasp-depth", type=float, default=0.010, help="TCP this far below the object top [m] (gear: 0.004, "
               "the finger pads must stay on the hub, above the teeth)")
p.add_argument("--gpu-refine-margin", type=float, default=None, help="--entry gpu: sample, then C3 refine "
               "with this margin (offline choice: -1); default: the sampler alone")
p.add_argument("--open-width", type=float, default=0.03, help="gripper opening before the grasp [m] (gear: 0.05)")
p.add_argument("--geo", default="", help="contact model geometry R,c,depth,a,pc [m] (default: the Factory 8 mm peg; "
               "gear on its shaft: 0.005,0.0005,0.020,0.00475,0.0001)")
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
wrench_log = []  # (joint_states time, Franka-sign external wrench estimate: force xyz, torque xyz)
node.create_subscription(WrenchStamped, "/cartesian_impedance_controller/external_wrench",
                         lambda m: wrench_log.append((state["t"], m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                                                      m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z)), 100)
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
GEO = dict(zip(("R", "c", "depth", "a", "pc"), map(float, args.geo.split(",")))) if args.geo else {}


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
z_grasp = peg_top - args.grasp_depth
tip_below_tcp = z_grasp - (peg_c[2] - peg_size[2] / 2)  # nominal: what the controller assumes
g_off = np.array([float(v) for v in args.grasp_offset.split(",")]) / 1000.0
grasp_xy = peg_c[:2] + np.array([g_off[0], 0.0])
z_grasp_actual = z_grasp - g_off[1]
print(f"PEG at {np.round(peg_c, 4)}, HOLE opening at {np.round(hole_top, 4)}, perception error "
      f"{np.round(offset[:2] * 1000, 2)} mm -> hole estimate {np.round(hole_est, 4)}; tip {tip_below_tcp * 1000:.1f} mm "
      f"below the TCP", flush=True)
t_start = state["t"]
gripper(args.open_width, settle=0.5)
segment("A home -> above peg -> grasp", [(np.array([*grasp_xy, Z_SAFE]), R), (np.array([*grasp_xy, z_grasp_actual]), R)])
gripper(0.0, settle=1.0)
closed = state["fingers"]
if abs(closed - peg_size[0]) > 0.003:
    tr = truth()
    print(f"FAIL: grasp, fingers {closed * 1000:.1f} mm (peg {peg_size[0] * 1000:.1f} mm); TCP "
          f"{np.round(fk(state['q']).translation * 1000, 1).tolist()} mm, peg (truth) "
          f"{np.round(np.array(tr['peg']['cube']) * 1000, 1).tolist() if 'peg' in tr else '?'} mm, quat "
          f"{np.round(tr['peg']['quat_wxyz'], 3).tolist() if 'peg' in tr else '?'}")
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
if args.strategy == "spiral":
    timer = node.create_timer(0.005, command)
    spin_until(lambda: c["phase"] in ("done", "failed") or state["t"] - c["t0"] > args.timeout_contact, 600)
elif args.strategy == "probe":
    # 3D: grid belief over the hole xy (exact axisymmetric hole), probes where entry is most likely
    from ci_mpc_planner.hole_belief import HoleBelief
    from ci_mpc_planner.peg_hole import PegHoleGeometry

    geo = PegHoleGeometry(tip_below_tcp=tip_below_tcp, **GEO)
    O = np.array([hole_est[0], hole_est[1], hole_tip_z])
    # lateral force evidence off: in Isaac the estimate's lateral error while pressing (~1 N) is the size of the
    # chamfer's push; the belief uses the robust part only ("the peg did not drop: the hole is not under the tip")
    # touch tolerance small: ring points over the chamfer must not count as supported (0.4 mm + the 0.3 mm
    # penetration tolerance shrank each probe's exclusion to ~1.3 mm, and the search crawled)
    belief = HoleBelief(geo, f_lateral=1e9, touch=1.5e-4)
    if args.belief_model:
        # from the source tree (the colcon install of ci_mpc_planner is older and is not rebuilt mid-benchmark)
        import importlib.util
        spec = importlib.util.spec_from_file_location("probe_likelihood", os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "ci_mpc_planner", "ci_mpc_planner", "probe_likelihood.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        lik = mod.ProbeLikelihood(os.path.expanduser(args.belief_model), scale=args.belief_scale)

        def learned_update(x, force, b=belief):
            tip, F = x[2] - b.g.tip_below_tcp, np.array(force, float)
            if args.force_debias > 0:
                hist = c.setdefault("lat_flat", [])
                if hist:
                    F[:2] -= np.median(np.array(hist[-args.force_debias:]), axis=0)
                if tip > -3e-4:  # flat-face probe: its lateral reading is mostly estimate bias
                    hist.append(np.array(force[:2], float))
            b.logw += lik(b.h, x[:2], tip, F)
            b.logw -= b.logw.max()

        belief.update = learned_update
        print(f"  belief: learned probe likelihood {args.belief_model} (scale {args.belief_scale})", flush=True)
    c["u_cmd"], c["ff"] = (fk(state["q"]).translation.copy(), R), np.zeros(3)

    c["z_bias"] = 0.0  # tip height offset of the grasp (peg seated deeper / shallower in the fingers than nominal)

    def measure5():
        T = fk(state["q"])
        Rrel = T.rotation @ R.T
        rx, ry = np.arctan2(Rrel[2, 1], Rrel[2, 2]), np.arctan2(Rrel[0, 2], Rrel[2, 2])
        p = T.translation - O
        p[2] -= c["z_bias"]
        return np.array([*p, rx, ry])

    c["tare"] = np.zeros(3)

    def force3(window=0.2, raw=False):
        rows = np.array([r[1:4] for r in wrench_log if r[0] > state["t"] - window])
        if not len(rows):
            return np.zeros(3)
        f = -rows.mean(0)  # Franka sign -> force on the peg
        # tare: the reading while hovering free just before the probe (estimate offsets from the torque sensor
        # bias, ~1 N in y here, and the carried peg weight); without it the bias read as a chamfer push
        return f if raw else f - c["tare"]

    def cmd(xy, z, ff=(0.0, 0.0, 0.0)):  # z in the bias-corrected model frame, like measure5()
        c["u_cmd"] = (np.array([O[0] + xy[0], O[1] + xy[1], O[2] + z + c["z_bias"]]), R)
        c["ff"] = np.asarray(ff, float)

    def command_probe():
        pos, Rc = c["u_cmd"]
        qc = pin.Quaternion(Rc)
        m = MpcCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qc.x, qc.y, qc.z, qc.w
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, c["ff"])
        m.stiffness = K_search
        m.integral_enabled = False
        cmd_pub.publish(m)

    timer = node.create_timer(0.005, command_probe)
    dt = args.plan_period
    c["mpc_ms"] = []
    if args.entry == "gpu":
        import socket
        port = 47000 + int(os.environ.get("ROS_DOMAIN_ID", "42"))
        srv_log = open(os.path.join(args.log_dir, "gpu_mpc_server.log"), "w")
        procs.append(subprocess.Popen([os.path.expanduser("~/IsaacLab/.venv/bin/python"),
                                       os.path.join(os.path.dirname(os.path.abspath(__file__)), "gpu_mpc_server.py"),
                                       "--port", str(port)] + (["--geo", f"{args.geo},{tip_below_tcp}"] if args.geo else [])
                                      + ([f"--refine-margin={args.gpu_refine_margin}"]
                                         if args.gpu_refine_margin is not None else []),
                                      stdout=srv_log, stderr=subprocess.STDOUT,
                                      env={k: v for k, v in os.environ.items() if not k.startswith(("PYTHON", "ROS",
                                                                                                      "AMENT", "LD_"))}))
        t_wait = time.time()
        while "ready" not in open(srv_log.name).read() and time.time() - t_wait < 300:
            time.sleep(1)
        mpc_sock = socket.create_connection(("127.0.0.1", port))
        mpc_io = mpc_sock.makefile("rw")
        print(f"  GPU MPC server up on port {port} ({time.time() - t_wait:.0f} s)", flush=True)
    z_hover = tip_below_tcp + 0.001
    tip_z = lambda x: x[2] - tip_below_tcp  # noqa: E731  (rigid grasp; the press is force-limited)
    probes, target, c["phase"] = [], np.zeros(2), "probe"
    n_probes = 0
    col_rng = np.random.default_rng(args.collect_seed)
    if args.collect:
        c["hold"], c["settle"] = 0.001, 0.3  # probe 0 and the calibration touch: the nominal values

    def touch(target):
        """Hover, move over target, tare, press until contact (or in the hole); returns (x, F) in light contact."""
        x = measure5()
        cmd(x[:2], z_hover)
        spin_sim(0.4)
        for _ in range(25):  # lateral move with a stiction feedforward along the remaining offset
            x = measure5()
            d = target - x[:2]
            if np.linalg.norm(d) < 2e-4:
                break
            cmd(target, z_hover, (*(2.5 * d / np.linalg.norm(d)), 0.0))
            spin_sim(dt)
        cmd(measure5()[:2] if np.linalg.norm(target - measure5()[:2]) > 2e-4 else target, z_hover)
        spin_sim(0.3)
        c["tare"] = force3(0.25, raw=True)
        x = measure5()
        uz = x[2]
        for _ in range(40):  # press (stiction feedforward), until contact or in the hole
            x = measure5()
            uz = max(uz - 5e-4, x[2] - 0.006)
            cmd(target, uz, (0.0, 0.0, -args.stiction_ff))
            spin_sim(dt)
            F = force3(0.1)
            if F[2] > 1.0 or tip_z(measure5()) < -args.engage:
                break
        # light: just keep the contact (collect mode randomizes the hold depth and settle time)
        cmd(target, measure5()[2] - c.get("hold", 0.001), (0.0, 0.0, -args.stiction_ff))
        spin_sim(c.get("settle", 0.3))
        return measure5(), force3(0.3)

    calib_touch = args.calib_touch if args.calib_touch is not None else (0.0 if args.geo else 0.0095)
    if calib_touch > 0:
        # grasp height on a spot that is surely flat top face (2026-10-25: probe 0 skipped the calibration when its
        # lateral force read > 1.5 N, then the belief never moved; D1 plain probe 14/15 calibrated vs 7/12 not).
        # The peg is held upright, so its tip sits at face height whenever part of its footprint is on the flat face:
        # centre 2-16 mm from the hole (R + chamfer 5.5 mm, face 12.5 mm half wide, ~14 mm on the diagonal). 9.5 mm on
        # the diagonal keeps that for estimate errors up to 5.4 mm (14.5 mm, first try: past the face edge, -8 mm)
        x, F = touch(np.array([-1.0, -1.0]) / np.sqrt(2.0) * calib_touch)
        # no lateral-force gate (an edge contact gives a push, not a height); 2.5 mm: grasp depth errors up to ~1 mm plus
        # a tilted hole reached +1.56 mm in the probe sweeps (1.5 mm rejected it)
        if F[2] > 1.0 and abs(tip_z(x)) < 0.0025:

            c["z_bias"] = tip_z(x)
            print(f"  grasp height offset {c['z_bias'] * 1000:+.2f} mm (calibration touch, {calib_touch * 1000:.1f} mm "
                  f"from the estimate)", flush=True)
        else:
            print(f"  calibration touch failed (tip {tip_z(x) * 1000:+.2f} mm, force {np.round(F, 2)} N)", flush=True)
    for k in range(args.max_probes):
        if state["t"] - c["t0"] > args.timeout_contact:
            break
        x, F = touch(target)
        if k == 0 and c["z_bias"] == 0.0 and F[2] > 1.0 and np.linalg.norm(F[:2]) < 1.5 and abs(tip_z(x)) < 0.0015:
            # first contact on the flat top face (no lateral push): its tip height is the grasp offset (Isaac:
            # +0.55 mm; uncorrected, every hypothesis showed a gap and the belief never moved)
            c["z_bias"] = tip_z(x)
            x = measure5()
            print(f"  grasp height offset {c['z_bias'] * 1000:+.2f} mm (calibrated on the top face)", flush=True)
        belief.update(x, F)
        est, std = belief.estimate()
        n_probes += 1
        engaged = tip_z(x) < -3e-4 or np.linalg.norm(F[:2]) > args.engage_lateral  # sank into the chamfer / a push (centering only)
        on_top = not engaged
        if on_top:
            probes.append(x[:2].copy())
            if args.track_height > 0:  # raw tip heights (bias added back), seeded with the calibration
                c.setdefault("flat_raw", [c["z_bias"]]).append(tip_z(x) + c["z_bias"])
                c["z_bias"] = float(np.median(c["flat_raw"][-args.track_height:]))
        print(f"  probe {k}: TCP {np.round(x[:2] * 1000, 2)} mm (true hole {np.round(-offset[:2] * 1000, 1)}), tip "
              f"{tip_z(x) * 1000:+.2f} mm, force {np.round(F, 2)} N; belief {np.round(est * 1000, 2)} "
              f"(std {std * 1000:.2f}) mm" + (f"; hold {c['hold'] * 1000:.2f} mm settle {c['settle']:.2f} s"
                                               if args.collect else ""), flush=True)
        if args.collect:  # next sampled probe around the true hole; no insertion
            r_s = col_rng.uniform(0.0, 9.0) if col_rng.random() < 0.5 else col_rng.uniform(2.0, 6.0)
            a_s = col_rng.uniform(0.0, 2.0 * np.pi)
            target = -offset[:2] + r_s / 1000.0 * np.array([np.cos(a_s), np.sin(a_s)])
            c["hold"], c["settle"] = col_rng.uniform(0.0005, 0.002), col_rng.uniform(0.2, 0.6)
            if k + 1 >= args.collect:
                break
            continue
        sunk = tip_z(x) < -3e-4
        if engaged and args.entry == "gpu" and (sunk or not args.lateral_centering) and std < args.gpu_std_max:
            # engaged: GPU sampling MPC (model on the belief's hole estimate) until the tip is 5 mm in
            for _ in range(80):
                x, F = measure5(), force3(0.1)
                est_now, _ = belief.estimate()
                wts = belief.weights()
                samples = belief.h[np.random.default_rng(len(c["mpc_ms"])).choice(len(wts), 8, p=wts)]
                mpc_io.write(json.dumps({"x": x.tolist(), "hole_xy": est_now.tolist(), "hole_samples": samples.tolist(),
                                         "wrench": [*F.tolist(), 0.0, 0.0]}) + "\n")
                mpc_io.flush()
                rep = json.loads(mpc_io.readline())
                c["mpc_ms"].append(rep["ms"])
                ug = np.array(rep["u"])
                c["u_cmd"] = (np.array([O[0] + ug[0], O[1] + ug[1], O[2] + ug[2] + c["z_bias"]]), R)
                c["ff"] = np.zeros(3)  # the MPC's set-point already contains the feedforward as an offset
                spin_sim(dt)
                if tip_z(measure5()) < -0.005:
                    break
            # the engaged contact informs the belief once, like a probe (2026-10-24: one update per GPU step counted up
            # to 80 correlated measurements as independent; the belief collapsed to 0.02 mm on a wrong spot and the
            # MPC chased it: track B 4/10 vs probe 24/30)
            belief.update(measure5(), force3(0.3))
            print(f"    GPU MPC: tip {tip_z(measure5()) * 1000:+.2f} mm at {np.round(measure5()[:2] * 1000, 2)} mm, "
                  f"{np.mean(c['mpc_ms']):.0f} ms per decision", flush=True)
        elif engaged:
            # engaged on the chamfer: follow its push towards the axis with a light normal force
            for _ in range(60):
                x, F = measure5(), force3(0.1)
                lat = F[:2]
                if np.linalg.norm(lat) > 0.3:
                    dirn = lat / np.linalg.norm(lat)
                    cmd(x[:2] + 4e-4 * dirn, x[2] - 0.002, (*(2.5 * dirn), -2.0))
                else:
                    cmd(x[:2], x[2] - 0.004, (0.0, 0.0, -args.stiction_ff))
                spin_sim(dt)
                if tip_z(measure5()) < -args.engage:
                    break
                if args.centering_bound > 0 and np.linalg.norm(measure5()[:2]) > args.centering_bound:
                    break
            print(f"    centering: tip {tip_z(measure5()) * 1000:+.2f} mm at {np.round(measure5()[:2] * 1000, 2)} mm",
                  flush=True)
        if tip_z(measure5()) < -args.engage and not (args.centering_bound > 0 and np.linalg.norm(measure5()[:2])
                                                     > args.centering_bound):  # in the hole: push to the bottom
            c["t_engage"] = c["t_contact"] = state["t"]
            depth_hist, retries, anchor = [], 0, None
            for _ in range(150 + 30 * args.jam_recovery):
                x, F = measure5(), force3(0.1)
                uz = x[2] - (0.006 if F[2] < args.max_contact_force else 0.002)
                cmd(x[:2] if anchor is None else anchor, uz, (0.0, 0.0, -args.stiction_ff))
                spin_sim(dt)
                if geo.depth - (-tip_z(measure5())) < 0.001:
                    break
                depth_hist.append(tip_z(measure5()))
                F_lat = force3(1.0)[:2]
                stalled = len(depth_hist) > 10 and depth_hist[-11] - depth_hist[-1] < 0.0002
                wedged = len(depth_hist) > 10 and np.linalg.norm(F_lat) > args.jam_force
                if retries < args.jam_recovery and (stalled or wedged):
                    # wedged: lift (--jam-lift), then press again shifted along the lateral contact force (the force on the
                    # peg points away from the wall that holds it). Stall alone misses it: the TCP keeps descending
                    # while the peg slips in the fingers (H1 friction 1.0: 5 N lateral, 0 N on the hole bottom).
                    retries, depth_hist = retries + 1, []
                    x = measure5()
                    u = F_lat / max(np.linalg.norm(F_lat), 1e-6)
                    for _ in range(5):
                        cmd(x[:2], x[2] + args.jam_lift)
                        spin_sim(dt)
                    anchor = x[:2] + args.jam_shift * u
                    print(f"    jam {retries} ({'lateral force' if wedged else 'stall'}): tip {tip_z(x) * 1000:+.2f} mm, "
                          f"lateral force {np.round(F_lat, 2)} N -> press at {np.round(anchor * 1000, 2)} mm",
                          flush=True)
            c["phase"], c["t_done"] = "done", state["t"]
            break
        target, _ = belief.next_probe(exclude=probes, current=x[:2], min_spacing=0.0015)
        if args.stuck_dither > 0:
            c.setdefault("recent", []).append(x[:2].copy())
            last = c["recent"][-3:]
            if len(last) == 3 and max(np.linalg.norm(a - last[-1]) for a in last) < 4e-4:
                a_d = col_rng.uniform(0.0, 2.0 * np.pi)
                target = target + args.stuck_dither * np.array([np.cos(a_d), np.sin(a_d)])
                print(f"    stuck: next probe dithered {args.stuck_dither * 1000:.1f} mm", flush=True)
    if c["phase"] != "done":
        c["phase"] = "failed"
    print(f"  probing: {c['phase']} after {n_probes} probes, {state['t'] - c['t0']:.1f} s", flush=True)
else:
    # model-based contact phase (planar x-z model along world x; the hole offset is along x in these tests)
    from ci_mpc_planner.c3 import C3Config, PegHoleC3
    from ci_mpc_planner.hole_estimator import HoleEstimator
    from ci_mpc_planner.peg_hole import PegHole, PegHoleGeometry

    Kc = np.array([K_search[0], K_search[2], K_search[4] / 2.0])  # rotational: K/2 per rad (quaternion error)
    comp = 1.0 / Kc
    geo = PegHoleGeometry(tip_below_tcp=tip_below_tcp, **GEO)
    O = np.array([hole_est[0], hole_est[1], hole_tip_z])  # model frame: estimated hole axis, opening plane
    peg_model = PegHole(geo, 0.0)
    arm_fric = np.array([2.0, 3.5, 0.05])  # the arm's stiction in TCP coordinates (z: 3 N of spring did not move it)
    cfg_c3 = C3Config()
    cfg_c3.arm_friction = arm_fric
    est = HoleEstimator(geo, comp, 0.0, n=args.particles, sigma_x=(1e-3, 1e-3, 5e-3), sigma_w=(1.0, 1.0, 0.03),
                        arm_friction=arm_fric)
    mpc = PegHoleC3(peg_model, comp, cfg_c3)

    def measure():
        T = fk(state["q"])
        Rrel = T.rotation @ R.T
        return np.array([T.translation[0] - O[0], T.translation[2] - O[2], np.arctan2(Rrel[0, 2], Rrel[2, 2])]), T.translation

    def contact_wrench(window=0.1):
        rows = np.array([r[1:] for r in wrench_log if r[0] > state["t"] - window])
        if not len(rows):
            return np.zeros(3)
        f = rows.mean(0)
        # Franka sign (robot on environment) -> contact wrench on the peg; the robot also carries the peg's weight
        return np.array([-f[0], -(f[2] - 0.019 * 9.81), -f[4]])

    def to_world(u):
        cphi, sphi = np.cos(u[2]), np.sin(u[2])
        Ry = np.array([[cphi, 0, sphi], [0, 1, 0], [-sphi, 0, cphi]])
        return np.array([O[0] + u[0], O[1], O[2] + u[1]]), Ry @ R

    c["u_cmd"] = to_world(measure()[0])

    c["ff"] = np.zeros(3)

    def command_model():
        pos, Rc = c["u_cmd"]
        qc = pin.Quaternion(Rc)
        m = MpcCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qc.x, qc.y, qc.z, qc.w
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, c["ff"])
        m.stiffness = K_search
        m.integral_enabled = False
        cmd_pub.publish(m)

    timer = node.create_timer(0.005, command_model)
    x_prev, _ = measure()
    u = x_prev.copy()
    goal = np.array([0.0, tip_below_tcp - geo.depth, 0.0])
    c["phase"], c["log2"], solve_ms = "model", [], []
    while state["t"] - c["t0"] < args.timeout_contact:
        spin_sim(args.plan_period)
        xm, p_w = measure()
        wm = contact_wrench()
        # the model's command: spring set-point u plus the feedforward force as an equivalent set-point offset C ff
        u_eff = u + comp * np.array([c["ff"][0], c["ff"][2], 0.0]) if "ff" in c else u
        est.update(x_prev, u_eff, xm, wm)
        h, h_std = est.estimate()
        peg_model.hole_x = goal[0] = h
        t0w = time.perf_counter()
        if args.strategy == "c3":
            u, info = mpc.solve(xm, goal, u)
        else:
            u = u.copy()
            following = args.follow_force > 0 and abs(wm[0]) > 0.3 and wm[1] > 1.0
            if following:  # on the rim: follow its push towards the hole axis with a light normal force
                c["follow_dir"] = np.sign(wm[0])
            elif args.follow_force > 0 and c.get("follow_dir") and wm[1] > 1.0 and abs(wm[0]) <= 0.3:
                following = True  # keep the direction while the lateral reading dips
            if following:
                u[0] += np.clip(args.follow_force * 3.0 * c["follow_dir"], -5e-4, 5e-4)
                ff_x = args.lateral_stiction_ff * c["follow_dir"]
            else:
                if h_std < 0.001:  # steer by the estimator only once it is confident
                    u[0] += np.clip(goal[0] - u[0], -5e-4, 5e-4)
                ff_x = 0.0
            # press at most 6 mm below the current TCP (3 N at k_z 500); limiting it relative to the goal at the hole
            # bottom gave up to 15 N while the peg sat at the opening, and the peg slid up in the fingers (~6-8 N)
            ff_z = -args.stiction_ff
            if following:  # light normal force so the rim friction (mu * F_n) does not lock the sideways motion
                u[1] = xm[1]
                ff_z = -1.5
            elif wm[1] < args.max_contact_force:
                u[1] = max(u[1] - 5e-4, goal[1], xm[1] - 0.006)
            else:  # pressing hard enough: hold the spring where the TCP is (peg must not slide in the fingers)
                u[1] = max(u[1], xm[1] - 0.002)
            u[2] = 0.0
            c["ff"] = np.array([ff_x, 0.0, ff_z])
        solve_ms.append((time.perf_counter() - t0w) * 1000)
        c["u_cmd"] = to_world(u)
        x_prev = xm
        tip_z = p_w[2] - tip_below_tcp
        c["log2"].append((state["t"], *xm, *u, h, h_std, *wm))
        if len(c["log2"]) % 10 == 0:
            print(f"  t {state['t'] - c['t0']:5.1f} s: tip {(xm[0] - geo.tip_below_tcp * np.sin(xm[2])) * 1000:+.2f} / "
                  f"{(tip_z - hole_tip_z) * 1000:+.2f} mm, tilt {np.degrees(xm[2]):+.1f} deg, hole estimate "
                  f"{h * 1000:+.2f} mm (std {h_std * 1000:.2f}; true {-offset[0] * 1000:+.2f}), contact wrench "
                  f"{np.round(wm, 2)}", flush=True)
        if hole_tip_z - tip_z > geo.depth - 0.001:
            c["phase"], c["t_done"] = "done", state["t"]
            c["t_engage"] = c["t_contact"] = c["t0"]
            break
    print(f"  model-based contact phase ({args.strategy}): {c['phase']}, planning {np.mean(solve_ms):.0f} ms mean, "
          f"{np.max(solve_ms):.0f} max (+ estimator)", flush=True)
    np.save(os.path.join(args.log_dir, "peg_model_log.npy"), np.array(c["log2"]))
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
success = depth_err < 0.001 and stays and xy_err < 0.0015  # xy: a peg off the block's side reached the depth
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
