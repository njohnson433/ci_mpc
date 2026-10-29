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

import wrist_csv  # scripts/ (next to this file): the wrist F/T log reader with its clock lag

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
p.add_argument("--force-source", default="observer", choices=["observer", "wrist"],
               help="force for the probes / contact logic: the controller observer (~/external_wrench) or the true force\n               through the hand (simulated wrist F/T, needs WRIST_FT=1; vectorized step 5: the Isaac effort state over-reads\n               in contact, the batched likelihood v5 is trained on true force)")
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
p.add_argument("--stuck-dither", type=float, default=0.0,
               help="probe: when the last 3 probes landed within 0.4 mm of each other, offset the next target by this "
                    "much in a random direction [m] (0 = off; A6 rep3 -5,0: the belief locked on at std 0 with the peg "
                    "held by stiction on the chamfer edge, every probe returned to the same spot until time ran out)")
p.add_argument("--push-follow", type=float, default=0.0,
               help="push to the bottom: move the xy command along the lateral contact force by this gain [m/N] per "
                    "planning step (max 0.1 mm), centring the part on its shaft / hole (0 = off)")
p.add_argument("--push-follow-deadband", type=float, default=0.7, help="push follow: ignore lateral forces below [N]")
p.add_argument("--push-dither", type=float, default=0.0,
               help="push to the bottom: circle the xy command with this radius [m] around the push start (0 = off)")
p.add_argument("--push-dither-hz", type=float, default=1.0, help="push dither frequency [Hz]")
p.add_argument("--yaw-search", type=float, default=0.0,
               help="push to the bottom: when it stalls / presses > --max-contact-force for 1 s, sweep the hand yaw up to "
                    "+-this many degrees for a drop (gear tooth on tooth, H2; 0 = off)")
p.add_argument("--yaw-step", type=float, default=1.0, help="yaw search step [deg]")
p.add_argument("--yaw-drop", type=float, default=0.002,
               help="yaw search: tip drop that counts as meshed [m] (1 mm was fooled by the TCP creeping)")
p.add_argument("--yaw-dwell", type=float, default=0.5, help="yaw search: time per yaw [s]")
p.add_argument("--jam-recovery", type=int, default=0,
               help="probe: retries when the push to the bottom stalls (lift 1 mm, re-press shifted along the lateral "
                    "contact force; 0 = off; H1 friction 1.0 0.63,1.57 wedged 13 mm short twice, H3 7 of 12)")
p.add_argument("--jam-shift", type=float, default=0.0003, help="lateral shift per jam retry [m]")
p.add_argument("--push-wiggle", type=float, default=0.0,
               help="push to the bottom: once it stalls, cone the commanded peg axis at this angle [deg] (--push-wiggle-hz) "
                    "until the bottom; a tight chamferless hole wedges a peg 0.2-0.4 deg off the hole axis (dev set: 2.2 N "
                    "lateral, 0.27 Nm at the wrist, 3 mm short; the observer reads no torque under the rotational stiction)")
p.add_argument("--grasp-settle", type=float, default=0.0,
               help="grasp: wait until the TCP is within this distance [m] of the grasp pose before closing (0 = the "
                    "segment default 1.5 mm), e.g. 0.0002")
p.add_argument("--push-stop", type=float, default=0.001,
               help="push to the bottom: stop when the estimated tip is this close to the seated depth [m]. The default "
                    "1 mm equals the 1 mm success threshold, so a run passes only if the part settles the last mm by itself "
                    "(H2 near-misses 1.03 / 1.09 mm short, 2026-10-29)")
p.add_argument("--push-step", type=float, default=0.006,
               help="push to the bottom: command this far below the measured pose while the force is below "
                    "--max-contact-force (else 2 mm); smaller = softer push (thin pegs jam in tight holes)")
p.add_argument("--push-wiggle-hz", type=float, default=1.0, help="--push-wiggle: cone frequency [Hz]")
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
p.add_argument("--c3-goal-below", type=float, default=0.011,
               help="probe --entry c3: C3 goal this far below the hole bottom [m] (at the bottom itself C3 stalls at "
                    "18 mm offline: stiction trap + force cost; 8-11 mm: 16/16, PROGRESS 2026-10-27)")
p.add_argument("--c3-std-max", type=float, default=float("inf"),
               help="probe --entry c3: hand over only when the belief std is below this [m]; otherwise center")
p.add_argument("--c3-steps", type=int, default=150, help="probe --entry c3: max C3 decisions per engagement")
p.add_argument("--c3-arm-friction", default="2.0,2.0,3.5,0.05,0.05",
               help="probe --entry c3: arm stiction in the C3 model x,y,z [N], rx,ry [Nm] (C33Config default; Isaac's "
                    "measured value: --breakaway)")
p.add_argument("--c3-stiction-ff", default="0,0",
               help="probe --entry c3: friction compensation lateral,z [N]: feedforward along the commanded lateral "
                    "motion and down; the C3 model sees the arm friction reduced by it (offline: c3_stall_diag.py "
                    "--stiction-ff 2,3 with the true stiction in the model: 16/16)")
p.add_argument("--c3-ff-lat-depth", type=float, default=float("inf"),
               help="probe --entry c3: lateral compensation only while the tip is less than this deep [mm]")
p.add_argument("--tilt-probe", type=float, default=0.0,
               help="probe: tilt the peg by this angle [deg] while probing (0 = off, upright as before). A chamferless "
                    "tight hole carries an upright peg on its face (H5 NIST: every probe > 0.1 mm off reads flat); tilted, "
                    "the peg's lowest rim point drops into the hole wherever it is over it. The belief then models a "
                    "sharp-edged hole (--tilt-clearance) and the probes place the lowest point; upright at the estimate once "
                    "the belief std < --tilt-std, then the usual insertion")
p.add_argument("--tilt-axis", default="y", choices=["x", "y"],
               help="--tilt-probe: world axis of the tilt. The fingers close along y: tilted about y the press loads the peg "
                    "along the pads (friction only), about x across them")
p.add_argument("--tilt-upright-at-est", action="store_true",
               help="--tilt-probe: the first upright probe at the belief estimate (else next_probe's uncertain edge, "
                    "~1.2 mm off: a flat read there carries no information the tilted probes did not have)")
p.add_argument("--tilt-engage", type=float, default=0.0,
               help="--tilt-probe: tip depth [m] that counts as in the hole (0 = --engage), e.g. 0.0005: a sharp-edged hole has no "
                    "chamfer to centre on, and lateral centering with the tip in the bore slid the peg 0.9 mm in the fingers")
p.add_argument("--tilt-rect", default="",
               help="--tilt-probe: rectangular peg / hole, the peg's half widths 'ax,ay' [m] along the model x / y (hole: + "
                    "half the clearance per side); default round")
p.add_argument("--tilt-first-upright", action="store_true",
               help="--tilt-probe: one upright probe at the estimate first (after the tilted calibration); tilt only if it "
                    "does not go in (H5: at <= 1 mm the tilted search cost 2-5x the time of a direct hit)")
p.add_argument("--tilt-upright-spiral", type=float, default=0.0,
               help="--tilt-probe: after the tilted search, upright probes on a sunflower spiral around the estimate, "
                    "r = PITCH sqrt(k) (spacing ~1.8 PITCH; e.g. 0.00017 = 0.3 mm), instead of next_probe (1.5 mm "
                    "spacing: a thin peg's capture is its ~0.5 mm end chamfer; dev round 4, 2026-10-29)")
p.add_argument("--tilt-slide", type=float, default=0.0,
               help="--tilt-probe: move between tilted probes in light contact at this speed [m/s] (e.g. 0.003), the "
                    "belief updated along the way, instead of lift / move / press (thin pegs: the drop area is ~8x "
                    "smaller than an 8 mm peg's, discrete presses rarely hit it)")
p.add_argument("--tilt-thin-auto", type=float, default=0.0,
               help="--tilt-probe: for a peg half width (--geo a) below this [m], switch to the thin-peg set: tilt 20 deg, "
                    "--tilt-upright-spiral 0.00017, --tilt-slide 0.003 (dev round 4: 3/3 vs 1/3; on round 8 / 10 the same "
                    "set lost 3 of 8 to in-grasp rotation, so it is gated by size; e.g. 0.003)")
p.add_argument("--tilt-slide-refresh", action="store_true",
               help="--tilt-slide: slide samples the belief predicts flat also update the flat reference (as presses do)")
p.add_argument("--tilt-std", type=float, default=0.0004, help="--tilt-probe: go upright below this belief std [m]")
p.add_argument("--tilt-max-probes", type=int, default=12, help="--tilt-probe: go upright after this many tilted probes")
p.add_argument("--tilt-clearance", type=float, default=0.0001,
               help="--tilt-probe: diametral clearance of the sharp-edged hole model [m]")
p.add_argument("--c3-integral", action="store_true",
               help="probe --entry c3: controller integrator on during the C3 phase (else off: the arm stalls where the "
                    "spring force equals the stiction, vectorized step 4c)")
p.add_argument("--entry", default="follow", choices=["follow", "gpu", "c3"],
               help="probe strategy, engaged phase: follow = force-following centering; gpu = GPU sampling MPC "
                    "(scripts/gpu_mpc_server.py in the Isaac venv, started by this script)")
p.add_argument("--strategy", default="spiral", choices=["spiral", "c3", "probe"],
               help="contact phase: spiral search (hand-written); c3 = hole localization from contact (particle "
                    "filter on the measured wrench) + planar C3+ CI-MPC; probe = 3D hole belief (x and y) + "
                    "information-driven probing + chamfer centering")
p.add_argument("--particles", type=int, default=30)
p.add_argument("--stiction-ff", type=float, default=3.0,
               help="[N] feedforward force along the descent that cancels the arm's static friction in z (3 N of "
                    "spring alone did not move the arm in free space)")
p.add_argument("--max-contact-force", type=float, default=4.0,
               help="[N] stop pressing further when the measured contact force exceeds this (the peg slid in the "
                    "fingers at 6-8 N)")
p.add_argument("--plan-period", type=float, default=0.1, help="[s] sim time per planning step")
p.add_argument("--probe", action="store_true", help="after inserting: hold and push with known feedforward forces "
               "(checks the external wrench estimate and the PhysX contact report against each other)")
p.add_argument("--pos-noise", type=float, default=0.0)
p.add_argument("--pos-noise-z", type=float, default=-1.0, help="detector z position noise std [m] (default: as --pos-noise)")
p.add_argument("--prior-sigma", type=float, default=0.0, help="probe: Gaussian prior N(estimate, sigma) [m] on the hole belief\n               (stage A: the camera error, e.g. 0.002); 0 = uniform over the grid as before")
p.add_argument("--smooth-approach", type=float, default=0.0, help="probe, stage A: first approach from above the hole as a\n               decelerating set-point ramp (speed gap / 0.15 s, capped at this value [m/s], at least 2 mm/s) to the hover\n               height instead of a jump; observer contact (Fz > 1 N) stops it; logs speed at contact and peak jerk")
p.add_argument("--known-grasp", action="store_true", help="stage A: the controller gets the true in-hand pose of the part after the\n               grasp (sim truth: tip height and xy offset of the part bottom from the TCP) and skips the grasp-height calibration touch")
p.add_argument("--size-noise", type=float, default=0.0)
p.add_argument("--yaw-noise-deg", type=float, default=0.0)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--grasp-depth", type=float, default=0.010, help="TCP this far below the object top [m] (gear: 0.004, "
               "the finger pads must stay on the hub, above the teeth)")
p.add_argument("--gpu-refine-margin", type=float, default=None, help="--entry gpu: sample, then C3 refine "
               "with this margin (offline choice: -1); default: the sampler alone")
p.add_argument("--gpu-no-dual-rescale", action="store_true", help="--gpu-refine-margin: C3 with the old ADMM "
               "(no scaled-dual rescale, before 2026-10-26)")
p.add_argument("--open-width", type=float, default=0.03, help="gripper opening before the grasp [m] (gear: 0.05)")
p.add_argument("--geo", default="", help="contact model geometry R,c,depth,a,pc [m] (default: the Factory 8 mm peg; "
               "gear on its shaft: 0.005,0.0005,0.020,0.00475,0.0001)")
p.add_argument("--breakaway", default="", help="instead of the contact phase: at the pose above the hole (free space, "
               "peg in hand, contact stiffness, set-point fixed) ramp the feedforward force along each listed direction "
               "(+x,-x,+y,-y,-z) and report the force where the TCP starts to move = the arm's Cartesian stiction "
               "(C3 arm_friction); results in <log-dir>/breakaway.json")
p.add_argument("--breakaway-rate", type=float, default=1.0, help="--breakaway: force ramp [N/s]")
p.add_argument("--breakaway-torque-rate", type=float, default=0.1, help="--breakaway +rx..-ry: torque ramp [Nm/s]")
p.add_argument("--breakaway-max", type=float, default=10.0, help="--breakaway: give up at this force [N]")
p.add_argument("--breakaway-move", type=float, default=0.1, help="--breakaway: motion threshold along the direction [mm]")
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
        if state.get("rec") is not None:  # --smooth-approach: joint trajectory for the smoothness metric
            state["rec"].append((state["t"], state["q"].copy()))
        if "fr3_finger_joint1" in msg.name:
            state["fingers"] = sum(msg.position[msg.name.index(f"fr3_finger_joint{i}")] for i in (1, 2))


def on_det(msg):
    if msg.detections:
        state["det"] = msg.detections[0]
        state["dets"] = {d.id: d for d in msg.detections}


node.create_subscription(JointState, "/joint_states", on_js, 100)
node.create_subscription(Detection3DArray, "/perception/detections", on_det, 10)
wrench_log = []  # (joint_states time, Franka-sign external wrench estimate: force xyz, torque xyz)
if args.force_source == "observer":
    node.create_subscription(WrenchStamped, "/cartesian_impedance_controller/external_wrench",
                             lambda m: wrench_log.append((state["t"], m.wrench.force.x, m.wrench.force.y,
                                                          m.wrench.force.z, m.wrench.torque.x, m.wrench.torque.y,
                                                          m.wrench.torque.z)), 100)
else:
    _wrist = {"fh": None, "buf": ""}

    def _poll_wrist():
        """Tail <sync>/wrist_ft.csv (sim time, raw hand-frame wrench, hand quaternion wxyz) into wrench_log, rotated to
        the world: R_hand raw = Franka sign, as the observer (PROGRESS "Ground truth for contact")."""
        if _wrist["fh"] is None:
            if not os.path.exists(sync("wrist_ft.csv")):
                return
            _wrist["fh"] = open(sync("wrist_ft.csv"))
        _wrist["buf"] += _wrist["fh"].read()
        *lines, _wrist["buf"] = _wrist["buf"].split("\n")
        for line in lines:
            v = line.split(",")
            if len(v) != 11 or v[0] == "t":
                continue
            row = np.array(v, float)
            Rh = wrist_csv.hand_rotation(row[None, 7:11])[0]
            wrench_log.append((row[0] - wrist_csv.WRIST_LAG, *(Rh @ row[1:4]), *(Rh @ row[4:7])))

    node.create_timer(0.005, _poll_wrist)
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
    # the planner's subscriptions must be matched first: a path published before discovery is lost (volatile QoS) and
    # the arm stays home (seen with two Isaac instances starting at once, 2026-10-27)
    spin_until(lambda: path_pub.get_subscription_count() > 0 and goal_pub.get_subscription_count() > 0, 30)
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
                               "--pos-noise", str(args.pos_noise), "--pos-noise-z", str(args.pos_noise_z),
                               "--size-noise", str(args.size_noise),
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
if args.tilt_probe > 0 and args.tilt_thin_auto > 0 and GEO.get("a", 1.0) < args.tilt_thin_auto:
    args.tilt_probe, args.tilt_upright_spiral, args.tilt_slide = 20.0, 0.00017, 0.003
    print(f"  thin peg (half width {GEO['a'] * 1000:.2f} mm < {args.tilt_thin_auto * 1000:.1f}): tilt 20 deg, upright "
          f"spiral, sliding scan", flush=True)


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
# --grasp-settle: wait until the TCP is this close to the grasp pose before closing (default: the segment's 1.5 mm).
# The hand arrived ~1.45 mm off in y, the first finger tipped the standing peg ~0.4 deg before the second clamped:
# a systematic -0.3 mm in-hand y offset of the peg bottom in every scene (src-35, 2026-10-29)
segment("A home -> above peg -> grasp", [(np.array([*grasp_xy, Z_SAFE]), R), (np.array([*grasp_xy, z_grasp_actual]), R)],
        **({"tol": args.grasp_settle, "timeout": 20.0} if args.grasp_settle > 0 else {}))
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
grasp_xy_off = np.zeros(2)  # part bottom - TCP in xy (world), known to the controller only with --known-grasp
if args.known_grasp:
    d_ = np.array(tr["peg"]["origin"]) - fk(state["q"]).translation
    print(f"  known grasp (stage A): tip {-d_[2] * 1000:.2f} mm below the TCP (nominal {tip_below_tcp * 1000:.2f}), "
          f"xy offset {np.round(d_[:2] * 1000, 2)} mm", flush=True)
    tip_below_tcp, grasp_xy_off = -d_[2], d_[:2]


def peg_in_hand():
    """Peg bottom in the hand (TCP) frame [m] (truth, diagnostic only): did the peg slide along the finger pads?"""
    tr_ = truth()
    if "peg" not in tr_:
        return np.full(3, np.nan)
    T = fk(state["q"])
    return T.rotation.T @ (np.array(tr_["peg"]["origin"]) - T.translation)


def peg_tilt_hand():
    """Angle [deg] between the peg axis and the hand axis (truth, diagnostic: rotation in the grasp)."""
    tp = truth().get("peg", {})
    if "quat_wxyz" not in tp:
        return float("nan")
    w_, x_, y_, z_ = tp["quat_wxyz"]
    pz = np.array([2 * (x_ * z_ + w_ * y_), 2 * (y_ * z_ - w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)])
    return float(np.degrees(np.arccos(np.clip(abs(pz @ fk(state["q"]).rotation[:, 2]), -1, 1))))


def peg_yaw_hand():
    """Plug yaw minus hand yaw about the world z [deg] (truth, diagnostic: a rectangular plug turned in the grasp)."""
    tp = truth().get("peg", {})
    if "quat_wxyz" not in tp:
        return float("nan")
    Rh = fk(state["q"]).rotation
    d = np.degrees(yaw_of(tp["quat_wxyz"]) - np.arctan2(Rh[1, 0], Rh[0, 0]))
    return float((d + 90.0) % 180.0 - 90.0)  # mod 180: the hand x axis is defined up to a half turn here


c_in_hand0, c_tilt0, c_yaw0 = peg_in_hand(), peg_tilt_hand(), peg_yaw_hand()
above = np.array([*(hole_est[:2] - grasp_xy_off), hole_est[2] + tip_below_tcp + 0.010])
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
if args.breakaway:
    # Cartesian stiction of the arm: set-point held at the start pose, feedforward ramped along one direction until
    # the TCP has moved --breakaway-move; the spring is ~0 at breakaway, so the feedforward is the static friction
    # rotations (+rx..-ry: torque about the TCP, base axes) use --breakaway-torque-rate and a 1 mrad threshold, as
    # scripts/real/c3_breakaway_real.py; motion = rotation vector of R R_start^T along the axis
    dirs = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1),
            "+rx": (1, 0, 0), "-rx": (-1, 0, 0), "+ry": (0, 1, 0), "-ry": (0, -1, 0)}
    T0 = fk(state["q"])
    x0 = T0.translation.copy()
    bk = {"p": x0.copy(), "ff": np.zeros(3), "tq": np.zeros(3)}

    def command_breakaway():
        m = MpcCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, bk["p"])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat.x, quat.y, quat.z, quat.w
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, bk["ff"])
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = map(float, bk["tq"])
        m.stiffness = K_search
        m.integral_enabled = False
        cmd_pub.publish(m)

    timer = node.create_timer(0.005, command_breakaway)
    out = {"x0": x0.tolist(), "stiffness": K_search, "rate": args.breakaway_rate, "move_mm": args.breakaway_move,
           "torque_rate": args.breakaway_torque_rate, "move_mrad": 1.0, "dirs": {}}
    for name in args.breakaway.split(","):
        d = np.array(dirs[name], float)
        rot = "r" in name
        rate, thr, fmax = ((args.breakaway_torque_rate, 1e-3, 2.0) if rot else
                           (args.breakaway_rate, args.breakaway_move / 1000, args.breakaway_max))
        bk["ff"], bk["tq"] = np.zeros(3), np.zeros(3)
        bk["p"] = x0.copy()
        spin_sim(1.0)  # back to the start pose and settled
        Ts, t0, trace = fk(state["q"]), state["t"], []
        xs, Rs = Ts.translation.copy(), Ts.rotation.copy()
        f_brk = None
        while True:
            spin_sim(0.01)
            f = rate * (state["t"] - t0)
            bk["tq" if rot else "ff"] = f * d
            T = fk(state["q"])
            mv = pin.log3(T.rotation @ Rs.T) if rot else T.translation - xs
            s = float(d @ mv)
            trace.append((state["t"] - t0, f, s * 1000, *(mv * 1000)))
            if s > thr:
                f_brk = f
                break
            if f > fmax:
                break
        bk["ff"], bk["tq"] = np.zeros(3), np.zeros(3)
        side = float(np.linalg.norm(mv - s * d) * 1000)
        unit, mu = ("Nm", "mrad") if rot else ("N", "mm")
        out["dirs"][name] = {"f_breakaway": f_brk, "start_drift_mm": float(np.linalg.norm(xs - x0) * 1000),
                             "off_axis": side, "trace": trace[::5]}
        print(f"  breakaway {name}: " + (f"{f_brk:.2f} {unit}" if f_brk is not None else f"none up to {fmax} {unit}")
              + f" (moved {s * 1000:.2f} {mu} along, {side:.2f} {mu} off-axis; start {np.linalg.norm(xs - x0) * 1000:.2f} mm "
              f"from x0)", flush=True)
    bk["ff"], bk["tq"] = np.zeros(3), np.zeros(3)
    spin_sim(0.5)
    json.dump(out, open(os.path.join(args.log_dir, "breakaway.json"), "w"))
    print("RESULT breakaway " + " ".join(f"{k} {v['f_breakaway']}" for k, v in out["dirs"].items()), flush=True)
    finish(0)
if args.strategy == "spiral":
    timer = node.create_timer(0.005, command)
    spin_until(lambda: c["phase"] in ("done", "failed") or state["t"] - c["t0"] > args.timeout_contact, 600)
elif args.strategy == "probe":
    # 3D: grid belief over the hole xy (exact axisymmetric hole), probes where entry is most likely
    from ci_mpc_planner.hole_belief import HoleBelief
    from ci_mpc_planner.peg_hole import PegHoleGeometry

    geo = PegHoleGeometry(tip_below_tcp=tip_below_tcp, **GEO)
    O = np.array([hole_est[0] - grasp_xy_off[0], hole_est[1] - grasp_xy_off[1], hole_tip_z])  # TCP xy that puts the
    # part bottom over the estimate (grasp_xy_off = 0 unless --known-grasp)
    # lateral force evidence off: in Isaac the estimate's lateral error while pressing (~1 N) is the size of the
    # chamfer's push; the belief uses the robust part only ("the peg did not drop: the hole is not under the tip")
    # touch tolerance small: ring points over the chamfer must not count as supported (0.4 mm + the 0.3 mm
    # penetration tolerance shrank each probe's exclusion to ~1.3 mm, and the search crawled)
    belief = HoleBelief(geo, f_lateral=1e9, touch=1.5e-4)
    if args.belief_model:
        from ci_mpc_planner.probe_likelihood import ProbeLikelihood

        lik = ProbeLikelihood(os.path.expanduser(args.belief_model), scale=args.belief_scale)

        def learned_update(x, force, b=belief):
            b.logw += lik(b.h, x[:2], x[2] - b.g.tip_below_tcp, np.array(force, float))
            b.logw -= b.logw.max()

        belief.update = learned_update
        print(f"  belief: learned probe likelihood {args.belief_model} (scale {args.belief_scale})", flush=True)
    from ci_mpc_planner.peg_hole_3d import rotxy as _rotxy
    c["tilt"] = ((0.0, np.radians(args.tilt_probe)) if args.tilt_axis == "y"  # (rx, ry) of the probing peg; (0, 0) = upright
                 else (np.radians(args.tilt_probe), 0.0))
    if args.tilt_probe > 0:
        # sharp-edged hole model (no chamfer), hole radius = peg half width + half the clearance
        tgeo = PegHoleGeometry(tip_below_tcp=tip_below_tcp, **{**GEO, "R": geo.a + args.tilt_clearance / 2, "c": 1e-5})
        rect = [float(v) for v in args.tilt_rect.split(",")] if args.tilt_rect else None
        if rect:  # the round model's a (peg half width) = the smaller half width; clearance per side as round
            tgeo = PegHoleGeometry(tip_below_tcp=tip_below_tcp,
                                   **{**GEO, "a": min(rect), "R": min(rect) + args.tilt_clearance / 2, "c": 1e-5})
        belief = HoleBelief(tgeo, f_lateral=1e9, touch=1.5e-4, capture=geo.a, face_rings=(0.25, 0.5, 0.75),
                            gap_sigma=1e-4, n_ring=48 if rect else 24, rect=rect)
        print(f"  tilt probe {args.tilt_probe:.1f} deg: sharp-edged hole model R {tgeo.R * 1000:.3f} mm"
              + (f", rectangular peg {np.round(np.array(rect) * 1000, 2)} mm half widths" if rect else ""), flush=True)

    def low_point(x):  # lowest peg point (model frame) at TCP pose x
        pw = belief.points_world(x)
        if args.tilt_rect:  # a tilted rectangle's lowest feature is an edge: its midpoint
            return pw[pw[:, 2] < pw[:, 2].min() + 1e-6].mean(axis=0)
        return pw[np.argmin(pw[:, 2])]
    c["u_cmd"], c["ff"] = (fk(state["q"]).translation.copy(), R), np.zeros(3)

    c["yaw"] = 0.0  # commanded hand yaw about the world z [rad] (--yaw-search), 0 = the grasp orientation
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
        cy, sy = np.cos(c["yaw"]), np.sin(c["yaw"])  # yaw search: rotation about the world z on top of R
        c["u_cmd"] = (np.array([O[0] + xy[0], O[1] + xy[1], O[2] + z + c["z_bias"]]),
                      np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @ _rotxy(*c["tilt"]) @ R)
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
        m.integral_enabled = c.get("integral", False)
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
                                         if args.gpu_refine_margin is not None else [])
                                      + (["--no-dual-rescale"] if args.gpu_no_dual_rescale else []),
                                      stdout=srv_log, stderr=subprocess.STDOUT,
                                      env={k: v for k, v in os.environ.items() if not k.startswith(("PYTHON", "ROS",
                                                                                                      "AMENT", "LD_"))}))
        t_wait = time.time()
        while "ready" not in open(srv_log.name).read() and time.time() - t_wait < 300:
            time.sleep(1)
        mpc_sock = socket.create_connection(("127.0.0.1", port))
        mpc_io = mpc_sock.makefile("rw")
        print(f"  GPU MPC server up on port {port} ({time.time() - t_wait:.0f} s)", flush=True)
    if args.entry == "c3":
        from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
        from ci_mpc_planner.peg_hole_3d import PegHole3D, rotxy

        c3_comp = 1.0 / np.array([K_search[0], K_search[1], K_search[2], K_search[3] / 2.0, K_search[4] / 2.0])
        c["c3_viol"] = []
        c3_cfg = C33Config()
        c3_cfg.arm_friction = np.array([float(v) for v in args.c3_arm_friction.split(",")])
        c3_sff = np.array([float(v) for v in args.c3_stiction_ff.split(",")])
        if c3_sff.any():  # the controller cancels part of the stiction: the model sees the rest (as c3_stall_diag.py)
            c3_cfg.arm_friction = np.maximum(c3_cfg.arm_friction - np.array([c3_sff[0], c3_sff[0], c3_sff[1], 0.0, 0.0]),
                                             0.3 * c3_cfg.arm_friction)
        print(f"  C3 model arm friction {np.round(c3_cfg.arm_friction, 2).tolist()}, compensation {c3_sff.tolist()} N",
              flush=True)
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
        zh = z_hover + (0.002 if any(c["tilt"]) else 0.0)  # tilted: the low rim point hangs ~1 mm lower
        first = "approach" not in c
        if first:  # smoothness metric of the first approach + press, with or without --smooth-approach
            state["rec"], c["approach"] = [], {"t0": state["t"], "smooth": args.smooth_approach > 0}
        if args.smooth_approach > 0 and first:
            # stage A: decelerating descent to the hover height (a camera z error can put the surface above it: the
            # observer, tared in free air at the start, stops the ramp at contact)
            c["tare"] = force3(0.25, raw=True)
            t_a, zc, hit = state["t"], x[2], False
            while zc > zh:
                zc = max(zh, zc - min(args.smooth_approach, max(0.002, (zc - zh) / 0.15)) * 0.02)
                cmd(x[:2], zc)
                spin_sim(0.02)
                if force3(0.05)[2] > 1.0:
                    hit = True
                    c["t_hit"] = state["t"]
                    cmd(x[:2], measure5()[2])
                    break
            c["approach"].update({"t1": state["t"], "contact_in_ramp": hit})
            print(f"  smooth approach: {state['t'] - t_a:.2f} s"
                  + (f", contact during the ramp at tip {tip_z(measure5()) * 1000:+.2f} mm" if hit else ""), flush=True)
        else:
            cmd(x[:2], zh)
            spin_sim(0.4)
            if first:
                c["approach"]["t1"] = state["t"]
        for _ in range(25):  # lateral move with a stiction feedforward along the remaining offset
            x = measure5()
            d = target - x[:2]
            if np.linalg.norm(d) < 2e-4:
                break
            cmd(target, zh, (*(2.5 * d / np.linalg.norm(d)), 0.0))
            spin_sim(dt)
        cmd(measure5()[:2] if np.linalg.norm(target - measure5()[:2]) > 2e-4 else target, zh)
        spin_sim(0.3)
        if not c.get("approach", {}).get("contact_in_ramp") or len(probes) > 0 or "t_hover0" in c:
            c["tare"] = force3(0.25, raw=True)
        c.setdefault("t_hover0", state["t"])  # first hover, peg in free air: wrist F/T tare window (last 0.25 s)
        x = measure5()
        uz = x[2]
        for _ in range(40):  # press (stiction feedforward), until contact or in the hole
            x = measure5()
            uz = max(uz - 5e-4, x[2] - 0.006)
            cmd(target, uz, (0.0, 0.0, -args.stiction_ff))
            spin_sim(dt)
            F = force3(0.1)
            if F[2] > 1.0 and "t_hit" not in c:
                c["t_hit"] = state["t"]  # first observer contact (stage A smoothness metric)
            if F[2] > 1.0 or tip_z(measure5()) < -args.engage:
                break
        # light: just keep the contact (collect mode randomizes the hold depth and settle time)
        cmd(target, measure5()[2] - c.get("hold", 0.001), (0.0, 0.0, -args.stiction_ff))
        spin_sim(c.get("settle", 0.3))
        if state.get("rec") is not None:  # first touch done: smoothness of approach + press (TCP z at joint-state rate)
            rec, state["rec"] = state["rec"], None
            ts = np.array([r[0] for r in rec])
            if len(ts) > 30:
                zs = np.array([fk(r[1]).translation[2] for r in rec])
                k_ = 5  # ~20 ms moving average (240 Hz) before each derivative
                sm_ = lambda v: np.convolve(v, np.ones(k_) / k_, "same")  # noqa: E731
                vz = np.gradient(sm_(zs), ts)
                az = np.gradient(sm_(vz), ts)
                jz = np.gradient(sm_(az), ts)
                inner = slice(3 * k_, len(ts) - 3 * k_)  # edges of the filters
                c["approach"].update({"v_peak_mm_s": float(np.abs(vz[inner]).max() * 1000),
                                      "jerk_peak_m_s3": float(np.abs(jz[inner]).max()),
                                      "v_contact_mm_s": float(np.abs(np.interp(c.get("t_hit", ts[-1]), ts, vz)) * 1000)
                                      if "t_hit" in c else None})
                print(f"  approach smoothness: peak speed {c['approach']['v_peak_mm_s']:.1f} mm/s, at contact "
                      f"{c['approach']['v_contact_mm_s'] or float('nan'):.1f} mm/s, peak jerk "
                      f"{c['approach']['jerk_peak_m_s3']:.2f} m/s^3", flush=True)
        return measure5(), force3(0.3)

    def slide(target):
        """--tilt-slide: from light contact, move to target (TCP xy) without lifting, pressing lightly; every 3rd
        0.1 s sample updates the belief (a line of probes per move). Stops early when the low point drops > 0.3 mm
        (the hole under it) or the peg snags (lateral force > 4 N). Returns the last (x, F) for the caller's update."""
        n_up = 0
        u = measure5()[:2]  # the commanded xy advances on its own (a step past the measured pose, ~0.45 N at 1500
        # N/m, stayed below the sliding friction of the pressed peg: the first version never moved, 2026-10-29)
        for i in range(int(0.012 / (args.tilt_slide * dt)) + 1):
            x = measure5()
            d = target - u
            if np.linalg.norm(d) < 2e-4 and np.linalg.norm(target - x[:2]) < 3e-4:
                break
            u = u + min(args.tilt_slide * dt, np.linalg.norm(d)) * d / max(np.linalg.norm(d), 1e-9)
            dd = target - x[:2]
            ff = 2.5 * dd / max(np.linalg.norm(dd), 1e-9)  # lateral stiction feedforward, as touch()'s moves
            cmd(u, x[2] - 0.001, (*ff, -args.stiction_ff))
            spin_sim(dt)
            x, F = measure5(), force3(0.1)
            if i % 3 == 2 and F[2] > 1.0:
                if args.tilt_slide_refresh:
                    # flat reference from the slide too: sliding friction pushes the peg up the fingers (H5 round 4:
                    # -0.34 mm by the handover), so the press-only reference went stale and flats read as drops
                    zf = tip_below_tcp - low_point(np.array([0.0, 0.0, tip_below_tcp, *c["tilt"]]))[2]
                    if belief.predicted_drop(np.array([x[0], x[1], zf, *c["tilt"]])) < 1e-4:
                        c["tilt_raw"].append(low_point(x)[2] + c["z_bias"])
                        c["z_bias"] = max(c["tilt_raw"][-4:])
                        x = measure5()
                belief.update(x, F)
                n_up += 1
            if low_point(x)[2] < -3e-4 or np.linalg.norm(F[:2]) > 4.0:
                break
        spin_sim(0.3)
        x, F = measure5(), force3(0.3)
        print(f"    slide: {n_up} samples, stopped at low point {np.round(low_point(x)[:2] * 1000, 2)} mm, height "
              f"{low_point(x)[2] * 1000:+.2f} mm, lateral {np.linalg.norm(F[:2]):.1f} N", flush=True)
        return x, F

    calib_touch = args.calib_touch if args.calib_touch is not None else (0.0 if args.geo else 0.0095)
    if args.known_grasp:  # stage A: the grasp height is known; the first flat contact still sets the surface height
        calib_touch = 0.0
    if args.prior_sigma > 0:  # stage A: camera prior instead of uniform (hypotheses h are relative to the estimate)
        belief.logw = -(belief.h ** 2).sum(axis=1) / (2.0 * args.prior_sigma ** 2)
        print(f"  belief prior N(estimate, {args.prior_sigma * 1000:.1f} mm) on {len(belief.h)} hypotheses", flush=True)
    tilting = args.tilt_probe > 0
    lo = low_point(np.array([0.0, 0.0, tip_below_tcp, *c["tilt"]]))[:2]  # lowest point relative to the TCP (xy)
    if tilting:
        # tilted calibration touch: the low point 9.5 mm from the estimate on the diagonal (hole radius + error < 9.5 mm),
        # its height is the grasp offset (the tip height of an upright peg is meaningless here)
        d_c = np.array([-1.0, -1.0]) / np.sqrt(2.0) * 0.0095
        x, F = touch(d_c - lo)
        lz = low_point(x)[2]
        if F[2] > 1.0 and abs(lz) < 0.0025:
            c["z_bias"] = lz
            print(f"  grasp height offset {lz * 1000:+.2f} mm (tilted calibration touch, low point "
                  f"{np.round(low_point(measure5())[:2] * 1000, 2)} mm)", flush=True)
        else:
            print(f"  tilted calibration touch failed (low point {lz * 1000:+.2f} mm, force {np.round(F, 2)} N)", flush=True)
        calib_touch = 0.0
        if args.tilt_first_upright:  # first probe upright at the estimate; the tilted search starts after it
            c["tilt_pending"], c["tilt"], tilting = c["tilt"], (0.0, 0.0), False
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
        if tilting and args.tilt_slide > 0 and c.get("in_contact"):
            x, F = slide(target - lo)
        else:
            x, F = touch(target - lo if tilting else target)
        # sliding continues from light contact on the flat face; after a drop (or upright) lift and press again
        c["in_contact"] = tilting and F[2] > 1.0 and low_point(x)[2] > -3e-4
        if not tilting and k == 0 and c["z_bias"] == 0.0 and F[2] > 1.0 and np.linalg.norm(F[:2]) < 1.5 and abs(tip_z(x)) < 0.0015:
            # first contact on the flat top face (no lateral push): its tip height is the grasp offset (Isaac:
            # +0.55 mm; uncorrected, every hypothesis showed a gap and the belief never moved)
            c["z_bias"] = tip_z(x)
            x = measure5()
            print(f"  grasp height offset {c['z_bias'] * 1000:+.2f} mm (calibrated on the top face)", flush=True)
        if tilting:
            # flat reference = the highest of the last 4 raw low-point heights: each press slips the peg up in the
            # fingers, so readings only sink (dev round 8: flats -0.1 -> -0.7 mm over 20 probes, as deep as real drops).
            # Only probes the belief expects flat (< 0.1 mm drop) count: 4 drops in a row made a drop the reference and
            # every later drop read 0.35 mm short (dev round 8 (1.41, 1.41), offline replay: model within 0.05 mm before)
            z_flat_ = tip_below_tcp - low_point(np.array([0.0, 0.0, tip_below_tcp, *c["tilt"]]))[2]
            c.setdefault("tilt_raw", [c["z_bias"]])
            if belief.predicted_drop(np.array([x[0], x[1], z_flat_, *c["tilt"]])) < 1e-4:
                c["tilt_raw"].append(low_point(x)[2] + c["z_bias"])
                c["z_bias"] = max(c["tilt_raw"][-4:])
            x = measure5()
        belief.update(x, F)
        est, std = belief.estimate()
        n_probes += 1
        engaged = tip_z(x) < -3e-4 or np.linalg.norm(F[:2]) > args.engage_lateral  # sank into the chamfer / a push (centering only)
        if tilting:  # tilted probes only inform the belief (a drop is the hole edge under the low point)
            engaged = False
            lp = low_point(x)
            print(f"  tilted probe {k}: low point {np.round(lp[:2] * 1000, 2)} mm (target {np.round(target * 1000, 2)}), "
                  f"height {lp[2] * 1000:+.2f} mm", flush=True)
        on_top = not engaged
        if on_top:
            probes.append(lp[:2].copy() if tilting else x[:2].copy())
            if tilting:  # and the target: the press's lateral force deflects the hand 0.7-1.3 mm back towards the last
                # probe (1-2 N at 1500 N/m), so excluding only the measured points re-picked the same spot (dev 1.41, 1.41)
                probes.append(np.asarray(target, float).copy())
            if args.track_height > 0 and not tilting:  # raw tip heights (bias added back), seeded with the calibration
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
        if tilting and (std < args.tilt_std or k + 1 >= args.tilt_max_probes):
            # localized (or out of tilted probes): upright at the estimate, then the usual probe / centering / push
            tilting, c["tilt"], probes, belief.capture = False, (0.0, 0.0), [], 0.001
            c["upright_first"] = args.tilt_upright_at_est
            if args.tilt_upright_spiral > 0:
                c["spiral"] = [est.copy(), 0]
            tp = truth().get("peg", {})
            if "quat_wxyz" in tp:  # diagnostic (truth): peg axis vs the hand axis, did the tilted presses rotate it?
                w_, x_, y_, z_ = tp["quat_wxyz"]
                pz = np.array([2 * (x_ * z_ + w_ * y_), 2 * (y_ * z_ - w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)])
                hz = fk(state["q"]).rotation[:, 2]
                ang = np.degrees(np.arccos(np.clip(abs(pz @ hz), -1, 1)))
            else:
                ang = float("nan")
            slip = peg_in_hand() - c_in_hand0
            print(f"    tilt search: {'localized' if std < args.tilt_std else 'out of tilted probes'} (std "
                  f"{std * 1000:.2f} mm) after {k + 1} probes, upright at {np.round(est * 1000, 2)} mm; peg axis vs hand "
                  f"axis {ang:.2f} deg, peg moved {np.round(slip * 1000, 2)} mm in the hand frame since the grasp (truth)",
                  flush=True)
        sunk = tip_z(x) < -3e-4
        if engaged and args.entry == "gpu" and (sunk or not args.lateral_centering) and std < args.gpu_std_max:
            # engaged: GPU sampling MPC (model on the belief's hole estimate) until the tip is 5 mm in
            t_a = state["t"]
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
            c.setdefault("phases", []).append(("gpu", t_a, state["t"]))
            # the engaged contact informs the belief once, like a probe (2026-10-24: one update per GPU step counted up
            # to 80 correlated measurements as independent; the belief collapsed to 0.02 mm on a wrong spot and the
            # MPC chased it: track B 4/10 vs probe 24/30)
            belief.update(measure5(), force3(0.3))
            print(f"    GPU MPC: tip {tip_z(measure5()) * 1000:+.2f} mm at {np.round(measure5()[:2] * 1000, 2)} mm, "
                  f"{np.mean(c['mpc_ms']):.0f} ms per decision", flush=True)
        elif engaged and args.entry == "c3" and sunk and std < args.c3_std_max:
            # sunk into the chamfer: 3D C3+ on the belief's hole estimate, down to the bottom (only a sink hands over:
            # flat-face pushes started GPU phases far from the hole, B4)
            mdl = PegHole3D(geo, hole_xy=est, n_ring=8, n_rim=12)
            mpc = PegHoleC3_3D(mdl, c3_comp, c3_cfg)
            goal = np.array([est[0], est[1], tip_below_tcp - geo.depth - args.c3_goal_below, 0.0, 0.0])
            u, t_a = measure5(), state["t"]
            est_err = float(np.linalg.norm(est - (-offset[:2])))
            print(f"    C3 handover: belief {np.round(est * 1000, 2)} mm (std {std * 1000:.2f}), true hole "
                  f"{np.round(-offset[:2] * 1000, 2)} mm: estimate error {est_err * 1000:.2f} mm", flush=True)
            # trace for offline replay (scripts/c3_isaac_replay.py): model-frame state, command, tared force per step
            tr_c3 = {"est": est.tolist(), "std": float(std), "true": (-offset[:2]).tolist(), "goal": goal.tolist(),
                     "comp": c3_comp.tolist(), "tip_below_tcp": float(tip_below_tcp), "steps": []}
            c.setdefault("c3_trace", []).append(tr_c3)
            c["integral"] = args.c3_integral
            for _ in range(args.c3_steps):
                x = measure5()
                t_s = time.perf_counter()
                u, info = mpc.solve(x, goal, u)
                c["mpc_ms"].append((time.perf_counter() - t_s) * 1000)
                c["c3_viol"].append(info.get("viol", np.nan))
                ff = np.zeros(3)
                if c3_sff.any():
                    dl = u[:2] - x[:2]
                    if np.linalg.norm(dl) > 5e-5 and -tip_z(x) * 1000 < args.c3_ff_lat_depth:
                        ff[:2] = c3_sff[0] * dl / np.linalg.norm(dl)
                    ff[2] = -c3_sff[1]
                # trace columns: t, x(5), u(5), F(3), viol, ms, ff(3)
                tr_c3["steps"].append([float(state["t"]), *x.tolist(), *u.tolist(), *force3(0.1).tolist(),
                                       float(info.get("viol", np.nan)), c["mpc_ms"][-1], *ff.tolist()])
                c["u_cmd"] = (np.array([O[0] + u[0], O[1] + u[1], O[2] + u[2] + c["z_bias"]]), rotxy(u[3], u[4]) @ R)
                c["ff"] = ff
                spin_sim(dt)
                if geo.depth - (-tip_z(measure5())) < 0.001:
                    break
            c["ff"], c["integral"] = np.zeros(3), False
            c.setdefault("phases", []).append(("c3", t_a, state["t"]))
            tr_c3["depth_end"] = float(-tip_z(measure5()))
            belief.update(measure5(), force3(0.3))  # once per engagement, like a probe
            print(f"    C3: tip {tip_z(measure5()) * 1000:+.2f} mm at {np.round(measure5()[:2] * 1000, 2)} mm, "
                  f"{np.mean(c['mpc_ms']):.0f} ms per decision, violation {np.nanmean(c['c3_viol']):.3f}", flush=True)
        elif engaged and not (args.tilt_engage > 0 and tip_z(measure5()) < -args.tilt_engage):
            # engaged on the chamfer: follow its push towards the axis with a light normal force
            t_a = state["t"]
            for _ in range(60):
                x, F = measure5(), force3(0.1)
                lat = F[:2]
                if np.linalg.norm(lat) > 0.3:
                    dirn = lat / np.linalg.norm(lat)
                    cmd(x[:2] + 4e-4 * dirn, x[2] - 0.002, (*(2.5 * dirn), -2.0))
                else:
                    cmd(x[:2], x[2] - 0.004, (0.0, 0.0, -args.stiction_ff))
                spin_sim(dt)
                if tip_z(measure5()) < -(args.tilt_engage or args.engage):
                    break
                if args.centering_bound > 0 and np.linalg.norm(measure5()[:2]) > args.centering_bound:
                    break
            c.setdefault("phases", []).append(("center", t_a, state["t"]))
            print(f"    centering: tip {tip_z(measure5()) * 1000:+.2f} mm at {np.round(measure5()[:2] * 1000, 2)} mm",
                  flush=True)
        if not any(c["tilt"]) and tip_z(measure5()) < -(args.tilt_engage or args.engage) and not (
                args.centering_bound > 0 and np.linalg.norm(measure5()[:2]) > args.centering_bound):  # in the hole: push to the bottom
            c["t_engage"] = c["t_contact"] = state["t"]
            depth_hist, retries, anchor, yaw_tries = [], 0, None, 0
            xy_push0, t_push0 = measure5()[:2], state["t"]
            for _ in range(150 + 30 * args.jam_recovery + (60 if args.yaw_search > 0 else 0)
                           + (60 if args.push_wiggle > 0 else 0)):
                x, F = measure5(), force3(0.1)
                uz = x[2] - (args.push_step if F[2] < args.max_contact_force else min(0.002, args.push_step))
                if args.push_follow > 0:
                    # follow the lateral contact force (the bore / hole wall pushes the part toward the axis): keeps
                    # the gear centred on its shaft, where its teeth mesh (H2 stalls all sat 0.23-0.30 mm off,
                    # the bore clearance; successes 0.02-0.16)
                    f_lat = force3(0.3)[:2]
                    if np.linalg.norm(f_lat) > args.push_follow_deadband:
                        anchor = (x[:2] if anchor is None else anchor) + np.clip(args.push_follow * f_lat, -1e-4, 1e-4)
                    c["follow_n"] = c.get("follow_n", 0) + 1
                    if c["follow_n"] % 20 == 0:
                        print(f"      push follow: lateral force {np.round(f_lat, 2)} N, tip {tip_z(x) * 1000:+.2f} mm, "
                              f"xy cmd {np.round((x[:2] if anchor is None else anchor) * 1000, 2)} mm", flush=True)
                xy_c = x[:2] if anchor is None else anchor
                if args.push_wiggle > 0 and "t_wiggle" in c:
                    ph = 2.0 * np.pi * args.push_wiggle_hz * (state["t"] - c["t_wiggle"])
                    c["tilt"] = tuple(np.radians(args.push_wiggle) * np.array([np.cos(ph), np.sin(ph)]))
                if args.push_dither > 0:
                    # circle of radius push_dither around where the push started (or the follow / jam anchor): lets a
                    # bore leaning on its shaft find the axis, where the gear teeth mesh (H2 stalls at the clearance)
                    ph = 2.0 * np.pi * args.push_dither_hz * (state["t"] - t_push0)
                    xy_c = (xy_push0 if anchor is None else anchor) + args.push_dither * np.array([np.cos(ph), np.sin(ph)])
                cmd(xy_c, uz, (0.0, 0.0, -args.stiction_ff))
                spin_sim(dt)
                if geo.depth - (-tip_z(measure5())) < args.push_stop:
                    break
                depth_hist.append(tip_z(measure5()))
                F_lat = force3(1.0)[:2]
                stalled = len(depth_hist) > 10 and depth_hist[-11] - depth_hist[-1] < 0.0002
                wedged = len(depth_hist) > 10 and np.linalg.norm(F_lat) > args.jam_force
                pressing = len(depth_hist) > 10 and force3(1.0)[2] > args.max_contact_force
                if args.push_wiggle > 0 and stalled and "t_wiggle" not in c:
                    c["t_wiggle"] = state["t"]
                    print(f"    push stalled at tip {tip_z(measure5()) * 1000:+.2f} mm: wiggle {args.push_wiggle:.2f} deg "
                          f"at {args.push_wiggle_hz:.1f} Hz", flush=True)
                if args.yaw_search > 0 and yaw_tries < 1 and (stalled or pressing):
                    # tooth on tooth (gear: 40 teeth, 9 deg pitch, meshes within ~2 deg): sweep the hand yaw
                    # 0, +s, -s, +2s, ... up to +-yaw_search while pressing lightly; keep the first yaw where it drops by --yaw-drop
                    yaw_tries, depth_hist = yaw_tries + 1, []
                    tip0, found = tip_z(measure5()), None
                    n = int(round(args.yaw_search / args.yaw_step))
                    for a in [k * sgn * args.yaw_step for k in range(1, n + 1) for sgn in (1, -1)]:
                        c["yaw"], tip_a = np.radians(a), tip_z(measure5())  # drop counted per step: the slow creep
                        # over a whole sweep added up to a false "drop" (H2 yaw 1 deg)
                        for _ in range(max(1, int(round(args.yaw_dwell / dt)))):
                            x = measure5()
                            cmd(x[:2] if anchor is None else anchor, x[2] - 0.003, (0.0, 0.0, -args.stiction_ff))
                            spin_sim(dt)
                        Rr, tp = fk(state["q"]).rotation @ R.T, truth().get("peg", {})
                        gy = np.degrees(yaw_of(tp["quat_wxyz"])) if "quat_wxyz" in tp else float("nan")
                        print(f"      yaw cmd {a:+.1f}: hand {np.degrees(np.arctan2(Rr[1, 0], Rr[0, 0])):+.2f} deg, "
                              f"object {gy:+.2f} deg (truth), tip {tip_z(measure5()) * 1000:+.2f} mm, "
                              f"Fz {force3(0.2)[2]:.1f} N", flush=True)
                        if tip_z(measure5()) < tip_a - args.yaw_drop:
                            found = a
                            break
                    Rr = fk(state["q"]).rotation @ R.T
                    print(f"    yaw search ({'stall' if stalled else 'pressing'} at tip {tip0 * 1000:+.2f} mm): "
                          f"{'dropped at %+.1f deg' % found if found is not None else 'no drop, back to 0'}, hand yaw "
                          f"{np.degrees(np.arctan2(Rr[1, 0], Rr[0, 0])):+.1f} deg, tip {tip_z(measure5()) * 1000:+.2f} mm, "
                          f"Fz {force3(0.3)[2]:.1f} N", flush=True)
                    if found is None:
                        c["yaw"] = 0.0
                    continue
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
            c["phase"], c["t_done"], c["tilt"] = "done", state["t"], (0.0, 0.0)
            c.setdefault("phases", []).append(("push", c["t_engage"], state["t"]))
            break
        if "tilt_pending" in c:  # the upright first probe did not go in: tilted search from here
            tilting, c["tilt"] = True, c.pop("tilt_pending")
            print("    upright first probe: not in, tilted search", flush=True)
        if tilting:  # the low point on the belief's uncertain edge
            z_flat = tip_below_tcp - low_point(np.array([0.0, 0.0, tip_below_tcp, *c["tilt"]]))[2]  # low point at 0
            target, _ = belief.next_probe_info(lambda cxy: np.array([*(cxy - lo), z_flat, *c["tilt"]]), exclude=probes,
                                               current=low_point(x)[:2])
        elif "spiral" in c:  # sunflower spiral around the tilted search's estimate (k = 0: the estimate itself)
            centre, k_s = c["spiral"]
            c["spiral"][1] += 1
            target = centre + args.tilt_upright_spiral * np.sqrt(k_s) * np.array(
                [np.cos(k_s * 2.39996323), np.sin(k_s * 2.39996323)])
        elif c.pop("upright_first", False):
            target = belief.estimate()[0]
        else:
            target, _ = belief.next_probe(exclude=probes, current=x[:2], min_spacing=0.0015)
        if args.stuck_dither > 0 and "spiral" not in c:  # the spiral is dense on purpose
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
        u, info = mpc.solve(xm, goal, u)
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
slip_end = peg_in_hand() - c_in_hand0  # truth diagnostic: peg moved in the hand frame since the grasp


tilt_end, yaw_end = peg_tilt_hand(), peg_yaw_hand()
gripper(max(0.03, args.open_width), settle=0.5)  # 0.03 did not release the gear (grasped 30.1 mm) / AutoMate 00388
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
       "total_s": float(state["t"] - t_start), "peg_slip_hand_mm": [float(v) for v in slip_end * 1000],
       "peg_tilt_hand_deg": tilt_end, "peg_tilt_hand_deg_grasp": c_tilt0,
       "peg_yaw_hand_deg": yaw_end, "peg_yaw_hand_deg_grasp": c_yaw0}
if c.get("approach"):  # --smooth-approach metrics (stage A)
    res["approach"] = {k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in c["approach"].items()
                       if k not in ("t0", "t1")}
    res["approach"]["duration_s"] = float(c["approach"].get("t1", c["approach"]["t0"]) - c["approach"]["t0"])
if c.get("phases"):  # engaged phases with the peak of the observer's force estimate (|F|, untared) in each
    W_ = np.array(wrench_log)
    res["phases"] = [(n, float(a), float(b), float(np.linalg.norm(W_[(W_[:, 0] >= a) & (W_[:, 0] <= b), 1:4], axis=1)
                                                    .max()) if ((W_[:, 0] >= a) & (W_[:, 0] <= b)).any() else 0.0)
                     for n, a, b in c["phases"]]
    res["peak_force_engaged_N"] = max(ph[3] for ph in res["phases"])
    if os.path.exists(sync("wrist_ft.csv")) and c.get("t_hover0"):
        # true force (WRIST_FT=1): wrist F/T |F - tare| in the engaged phases, tared once at the first free-air hover;
        # (the "observer peak stiction-biased by 5-8 N" reading of 2026-10-28 used the unaligned windows: recheck)
        # (wrist_csv: world frame, CSV stamps shifted by the measured 0.235 s lag; before 2026-10-28 evening the
        # windows were 0.235 s off: scripts/recompute_wrist_peaks.py redoes saved runs)
        pk = wrist_csv.phase_peaks(sync("wrist_ft.csv"), c["t_hover0"], c["phases"])
        if pk is not None:
            res["phases_wrist_N"], res["peak_force_wrist_engaged_N"] = pk, max(pk)
            res["wrist_lag_s"], res["wrist_t_hover0"] = wrist_csv.WRIST_LAG, float(c["t_hover0"])  # tare time: offline stats
if c.get("c3_trace"):  # per C3 handover: estimate error, belief std [mm], depth reached [mm]
    res["c3_handovers"] = [(float(np.linalg.norm(np.array(t["est"]) - np.array(t["true"])) * 1000),
                            t["std"] * 1000, t.get("depth_end", float("nan")) * 1000) for t in c["c3_trace"]]
    json.dump(c["c3_trace"], open(os.path.join(args.log_dir, "c3_trace.json"), "w"))
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
