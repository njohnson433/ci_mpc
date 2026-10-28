#!/usr/bin/env python3
# Stage-1 MPC on the real FR3 in free space (no contact): first real test of ci_mpc_planner/mpc_node streaming
# MpcCommand to the cartesian_impedance_controller. Real-robot counterpart of scripts/mpc_stage1_test.py (Isaac).
#
# Needs: franka_hardware + cartesian_impedance_controller active (config/real_robot_controllers.yaml), arm in free
# space near the libfranka home pose, a person on the enabling device. Session script: mpc_free_space_session.sh.
# mpc_node runs with ci_mpc_planner/config/mpc_real.yaml (v 0.1 m/s, a 0.5 m/s^2, box x 0.25-0.55, y +-0.25,
# z 0.2-0.6, rotation <= 0.5 rad from the start) and the Desk end-effector profile as its EE frame.
#
# Tests (--tests, in this order; each one but hold starts at home):
#   hold     10 s at the start pose: drift, MPC solve times, command gaps
#   home     MPC reach to the home pose (TCP 0.307, 0, 0.487 as fr3_hand_tcp) and orientation
#   steps    +-x, +-y, +-z by --step from home and back: time to 1 cm / 1 mm, overshoot, final error, line deviation
#   rotate   +-0.3 rad about world z, +-0.2 rad about world x, and back: orientation error, position drift
#   circle   ~/path circle, radius --radius in the y-z plane through home: arm vs command and vs the ideal circle
# Results: <out>/mpc_free_space.json (per test), raw.npz (joint states, EE, commands, controller observer wrench and
# Franka's O_F_ext_hat_K; monotonic receive time).
#
# Safety: abort if the robot leaves Move mode, the state freezes / /joint_states stop for 0.3 s (reflex), the TCP
# moves faster than --abort-speed, is more than --abort-dev from the newest command, or leaves the MPC box by more
# than 3 cm. Abort = stop mpc_node: after 50 ms without a command the controller holds the pose it is at
# (mpc_timeout), so killing this script (Ctrl-C) is also safe.
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
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import Pose, PoseArray, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:  # real robot only: the robot's mode (a reflex stops the arm while joint_states keep coming)
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None
MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped", 6: "automatic error recovery"}
HERE = os.path.dirname(os.path.abspath(__file__))

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--tests", default="hold,home,steps,rotate,circle")
p.add_argument("--step", type=float, default=0.05, help="step size [m]")
p.add_argument("--radius", type=float, default=0.04, help="circle radius [m]")
p.add_argument("--path-speed", type=float, default=0.06, help="circle speed along the path [m/s]")
p.add_argument("--settle", type=float, default=4.0, help="time per move after the goal is sent [s]")
p.add_argument("--abort-speed", type=float, default=0.2, help="abort above this TCP speed [m/s] (MPC v_max 0.1)")
p.add_argument("--abort-dev", type=float, default=0.02, help="abort if the TCP is this far from the command [m]")
p.add_argument("--home-max", type=float, default=0.35, help="refuse the home reach from farther away [m]")
p.add_argument("--ee-profile", default=os.path.join(HERE, "desk_ee_hand_camera_mount_v0.1.endeffector-profile.json"),
               help="Desk end-effector profile (the controller's EE on the robot); empty = URDF fr3_hand_tcp (Isaac)")
p.add_argument("--mpc-config", default="", help="mpc_node parameter file (default: ci_mpc_planner mpc_real.yaml)")
p.add_argument("--mpc-args", default="", help="extra 'name:=value' for mpc_node, space separated")
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--out", default="")
args = p.parse_args()
tests = args.tests.split(",")
cfg_file = args.mpc_config or os.path.join(get_package_share_directory("ci_mpc_planner"), "config", "mpc_real.yaml")
P_MIN, P_MAX = np.array([0.25, -0.25, 0.2]), np.array([0.55, 0.25, 0.6])  # mpc_real.yaml
for line in open(cfg_file):  # keep the box in sync with the config
    if line.strip().startswith(("p_min:", "p_max:")):
        v = np.array(json.loads(line.split(":", 1)[1].split("#")[0]))
        P_MIN, P_MAX = (v, P_MAX) if line.strip().startswith("p_min") else (P_MIN, v)

# --- EE kinematics: the same frame as mpc_node / the controller
sys.path.insert(0, os.path.join(HERE, "..", "..", "ci_mpc_planner"))
from ci_mpc_planner.ee_model import ARM_JOINTS as ARM, EeModel  # noqa: E402

ee = EeModel(args.urdf, ee_profile=args.ee_profile)
Q_HOME = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])


def fk(q):
    p_, R, _, _ = ee.update(q, np.zeros(7))
    return p_, R


P_HOME, R_HOME = fk(Q_HOME)

rclpy.init()
node = Node("mpc_free_space_real")
state = {"q": None, "mode": None}
raw, cmds = [], []  # (t, q, p) / (t, p_cmd, quat_cmd)


def on_js(msg):
    if all(n in msg.name for n in ARM):
        q = np.array([msg.position[msg.name.index(n)] for n in ARM])
        t = time.monotonic()
        state["q"], state["js_t"] = q, t
        # measured torques are never bit-identical on the real arm; after a reflex the state freezes (2026-10-27)
        tau = tuple(msg.effort[msg.name.index(n)] for n in ARM) if len(msg.effort) == len(msg.name) else None
        if tau is None or tau != state.get("tau"):
            state["tau"], state["tau_t"] = tau, t
        raw.append((t, q))


def on_cmd(m):
    cmds.append((time.monotonic(), np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                 np.array([m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z])))


node.create_subscription(JointState, "/joint_states", on_js, qos_profile_sensor_data)
node.create_subscription(MpcCommand, "/cartesian_impedance_controller/mpc_command", on_cmd, qos_profile_sensor_data)
wrenches = []  # (t, source 0 = controller observer / 1 = Franka O_F_ext_hat_K, wrench 6), base frame


def w6(w):
    return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]


def on_robot_state(m):
    state["mode"] = m.robot_mode
    wrenches.append((time.monotonic(), 1, w6(m.o_f_ext_hat_k.wrench)))


node.create_subscription(WrenchStamped, "/cartesian_impedance_controller/external_wrench",
                         lambda m: wrenches.append((time.monotonic(), 0, w6(m.wrench))), 10)
if FrankaRobotState is not None:
    node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", on_robot_state, 10)
goal_pub = node.create_publisher(Pose, "/ci_mpc_planner/goal_pose", 10)
path_pub = node.create_publisher(PoseArray, "/ci_mpc_planner/path", 10)
active_pub = node.create_publisher(Bool, "/ci_mpc_planner/active", 10)
out = {"tests": {}, "config": cfg_file, "ee_profile": args.ee_profile}
watch = {"on": False, "aborting": False, "prev": None, "sigint": False}
# Ctrl-C: stop at the next spin and save what was measured so far (abort -> finish)
signal.signal(signal.SIGINT, lambda *_: watch.__setitem__("sigint", True))
mpc = {"proc": None, "log": None}


def stop_mpc():
    pr = mpc["proc"]
    if pr is not None and pr.poll() is None:
        active_pub.publish(Bool(data=False))
        pr.send_signal(signal.SIGINT)
        try:
            pr.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pr.kill()


def finish(code):
    stop_mpc()  # the controller holds the current pose 50 ms later
    if mpc["log"]:
        out["mpc_node_report"] = [ln.strip() for ln in open(mpc["log"]) if "solves" in ln][-20:]
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump(out, open(os.path.join(args.out, "mpc_free_space.json"), "w"), indent=1)
        if raw:
            t = np.array([r[0] for r in raw])
            q = np.array([r[1] for r in raw])
            ct = np.array([c[0] for c in cmds]) if cmds else np.zeros(0)
            np.savez_compressed(os.path.join(args.out, "raw.npz"), t=t, q=q, p=np.array([fk(x)[0] for x in q]),
                                cmd_t=ct, cmd_p=np.array([c[1] for c in cmds]).reshape(-1, 3),
                                cmd_quat=np.array([c[2] for c in cmds]).reshape(-1, 4),
                                # controller observer (src 0) vs Franka O_F_ext_hat_K (src 1): free-space bias of both
                                wrench_t=np.array([w[0] for w in wrenches]), wrench_src=np.array([w[1] for w in wrenches]),
                                wrench=np.array([w[2] for w in wrenches]).reshape(-1, 6))
        print(f"results: {args.out}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


def abort(msg):
    watch["aborting"] = True
    print("ABORT: " + msg, flush=True)
    out["abort"] = msg
    finish(2)


def spin(seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.002)
        if watch["sigint"] and not watch["aborting"]:
            abort("stopped by Ctrl-C")
        if not watch["on"] or watch["aborting"] or state["q"] is None:
            continue
        now = time.monotonic()
        if mpc["proc"] is not None and mpc["proc"].poll() is not None:
            abort("stopped by Ctrl-C" if watch["sigint"] else f"mpc_node exited (see {mpc['log']})")
        if state["mode"] is not None and (now - state.get("tau_t", now) > 0.3 or now - state.get("js_t", now) > 0.3):
            abort("robot state frozen / no /joint_states for 0.3 s: the robot stopped (reflex? see launch.log; "
                  "communication_constraints_violation = the 1 kHz loop missed its deadline)")
        if state["mode"] not in (None, 2):
            abort(f"robot left Move mode: {MODES.get(state['mode'], state['mode'])} (see launch.log)")
        x = fk(state["q"])[0]
        if np.any(x < P_MIN - 0.03) or np.any(x > P_MAX + 0.03):
            abort(f"TCP {np.round(x, 3).tolist()} left the MPC box by > 3 cm")
        if cmds and now - cmds[-1][0] < 0.2 and np.linalg.norm(x - cmds[-1][1]) > args.abort_dev:
            abort(f"TCP {np.linalg.norm(x - cmds[-1][1]) * 1000:.1f} mm from the MPC command")
        # speed over >= 50 ms: /joint_states arrival jitter made 10 ms differences read 0.34 m/s at a true 0.1 m/s
        if watch["prev"] is not None and now - watch["prev"][0] >= 0.05:
            v = float(np.linalg.norm(x - watch["prev"][1]) / (now - watch["prev"][0]))
            watch["prev"] = (now, x)
            if v > args.abort_speed:
                abort(f"TCP speed {v:.3f} m/s")
        elif watch["prev"] is None:
            watch["prev"] = (now, x)


def pose_msg(pos, R):
    m = Pose()
    m.position.x, m.position.y, m.position.z = map(float, pos)
    qt = pin.Quaternion(R)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = qt.x, qt.y, qt.z, qt.w
    return m


def ee_window(t0, t1):
    w = [(t, q) for t, q in raw if t0 <= t <= t1]
    return np.array([t for t, _ in w]), [fk(q) for _, q in w]


def cmd_gaps(t0, t1):
    ct = np.array([c[0] for c in cmds if t0 <= c[0] <= t1])
    return float(np.diff(ct).max() * 1000) if len(ct) > 1 else float("nan")


def move(name, pos, R, settle=None):
    """Send a goal pose, record time to 1 cm / 1 mm, overshoot, line deviation, final position / rotation error."""
    t0 = time.monotonic()
    start = fk(state["q"])[0]
    goal_pub.publish(pose_msg(pos, R))
    spin(settle or args.settle)
    ts, poses = ee_window(t0, time.monotonic())
    P = np.array([x for x, _ in poses])
    err = np.linalg.norm(P - pos, axis=1)
    d = pos - start
    L = float(np.linalg.norm(d))
    r = {"goal": np.round(pos, 4).tolist(), "dist_mm": round(L * 1000, 1)}
    for tol, key in ((0.01, "t_1cm_s"), (0.001, "t_1mm_s")):
        k = np.flatnonzero(err < tol)
        r[key] = round(float(ts[k[0]] - t0), 3) if len(k) else None
    rot_err = np.array([np.linalg.norm(pin.log3(Rm @ R.T)) for _, Rm in poses])
    k = np.flatnonzero(rot_err < 0.01)
    r["t_10mrad_s"] = round(float(ts[k[0]] - t0), 3) if len(k) else None
    if L > 1e-3:
        u = d / L
        along = (P - start) @ u
        r["overshoot_mm"] = round(float(max(0.0, along.max() - L)) * 1000, 2)
        r["line_dev_mm"] = round(float(np.linalg.norm((P - start) - np.outer(along, u), axis=1).max()) * 1000, 2)
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(ts), 1e-4)
    r["v_peak"] = round(float(np.percentile(v, 99)), 3) if len(v) else None
    tail = ts > ts[-1] - 0.5
    r["final_mm"] = round(float(err[tail].mean()) * 1000, 2)
    r["final_rot_mrad"] = round(float(rot_err[tail].mean()) * 1000, 2)
    r["cmd_gap_max_ms"] = round(cmd_gaps(t0, time.monotonic()), 1)
    print(f"  {name}: {r}", flush=True)
    return r


def go_home():
    r = move("home", P_HOME, R_HOME)
    if r["final_mm"] > 5 or r["final_rot_mrad"] > 20:
        abort(f"home not reached ({r['final_mm']} mm, {r['final_rot_mrad']} mrad)")
    return r


# --- start
t_end = time.monotonic() + 10
while state["q"] is None and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
if state["q"] is None:
    print("no /joint_states (is franka_hardware running, ROS_DOMAIN_ID?)")
    finish(1)
x_start, R_start = fk(state["q"])
print(f"start TCP {np.round(x_start, 4).tolist()} m (EE: {os.path.basename(args.ee_profile) or 'fr3_hand_tcp'}); "
      f"home {np.round(P_HOME, 4).tolist()}; MPC box {P_MIN.tolist()} - {P_MAX.tolist()}", flush=True)
if np.any(x_start < P_MIN) or np.any(x_start > P_MAX):
    print("start pose outside the MPC box: move the arm near home first, e.g. "
          "c3_breakaway_session.sh hold --goto 0.307,0,0.487")
    finish(1)
if any(t != "hold" for t in tests):
    far, rot = np.linalg.norm(x_start - P_HOME), np.linalg.norm(pin.log3(R_HOME @ R_start.T))
    if far > args.home_max or rot > 0.45:
        print(f"start is {far * 1000:.0f} mm / {rot:.2f} rad from home (limits {args.home_max * 1000:.0f} mm, "
              f"0.45 rad of rot_box 0.5): move the arm near home first")
        finish(1)

exe = os.path.join(get_package_prefix("ci_mpc_planner"), "lib", "ci_mpc_planner", "mpc_node")
cmd = [exe, "--ros-args", "--params-file", cfg_file, "-p", "task:=hold", "-p", f"path_v_max:={args.path_speed}",
       "-p", "path_a_max:=0.2", "-p", "path_blend:=0.01"]
if args.ee_profile:  # (rcl rejects an empty override; empty = the config's ee_profile, if any; the frame check below)
    cmd += ["-p", f"ee_profile:={args.ee_profile}"]
for extra in args.mpc_args.split():
    cmd += ["-p", extra]
mpc["log"] = os.path.join(args.out or "/tmp", "mpc_node.log")
os.makedirs(os.path.dirname(mpc["log"]), exist_ok=True)
print("$ " + " ".join(cmd), flush=True)
mpc["proc"] = subprocess.Popen(cmd, stdout=open(mpc["log"], "w"), stderr=subprocess.STDOUT)
t_end = time.monotonic() + 30
while not cmds and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
    if mpc["proc"].poll() is not None:
        print("mpc_node exited:\n" + open(mpc["log"]).read()[-2000:])
        finish(1)
if not cmds:
    abort("no MpcCommand from mpc_node within 30 s")
# frame check: mpc_node holds the pose it measured in its EE frame; if the controller tracks another frame (Desk
# profile vs fr3_hand_tcp: 8.5 mm along the tool) the arm moves by the difference as soon as the commands arrive
print(f"mpc_node streaming; first command {np.linalg.norm(cmds[-1][1] - fk(state['q'])[0]) * 1000:.2f} mm from the "
      f"TCP; frame check (1.5 s)", flush=True)
t_end = time.monotonic() + 1.5
while time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.002)
    moved = np.linalg.norm(fk(state["q"])[0] - x_start)
    if moved > 0.002:
        abort(f"the arm moved {moved * 1000:.1f} mm when the MPC commands started: EE frame mismatch between "
              f"mpc_node and the controller (--ee-profile / the Desk profile?)")
watch["on"] = True

if "hold" in tests:
    t0 = time.monotonic()
    spin(10.0)
    ts, poses = ee_window(t0, time.monotonic())
    P = np.array([x for x, _ in poses])
    out["tests"]["hold"] = r = {"drift_max_mm": round(float(np.linalg.norm(P - x_start, axis=1).max()) * 1000, 2),
                                "rot_drift_max_mrad": round(float(max(np.linalg.norm(pin.log3(Rm @ R_start.T))
                                                                      for _, Rm in poses)) * 1000, 2),
                                "cmd_gap_max_ms": round(cmd_gaps(t0, time.monotonic()), 1),
                                "js_rate_hz": round(len(ts) / max(ts[-1] - ts[0], 1e-3), 1)}
    print(f"hold: {r}", flush=True)

if any(t in tests for t in ("home", "steps", "rotate", "circle")):
    out["tests"]["home"] = go_home()

if "steps" in tests:
    out["tests"]["steps"] = res = {}
    for ax, name in enumerate("xyz"):
        for sgn in (1, -1):
            d = np.zeros(3)
            d[ax] = sgn * args.step
            lbl = f"{'+' if sgn > 0 else '-'}{name}"
            res[lbl] = move(lbl, P_HOME + d, R_HOME)
            res[lbl + "_back"] = move(lbl + " back", P_HOME, R_HOME)

if "rotate" in tests:
    out["tests"]["rotate"] = res = {}
    for lbl, rv in (("+rz", [0, 0, 0.3]), ("-rz", [0, 0, -0.3]), ("+rx", [0.2, 0, 0]), ("-rx", [-0.2, 0, 0])):
        t0 = time.monotonic()
        res[lbl] = move(lbl, P_HOME, pin.exp3(np.array(rv, dtype=float)) @ R_HOME)
        res[lbl]["pos_drift_max_mm"] = round(float(max(np.linalg.norm(x - P_HOME) for x, _ in
                                                       ee_window(t0, time.monotonic())[1])) * 1000, 2)
        res[lbl + "_back"] = move(lbl + " back", P_HOME, R_HOME)

if "circle" in tests:
    # circle in the y-z plane through home (center home - r y), counter-clockwise, one lap, ending at home
    r_, n = args.radius, 36
    center = P_HOME - np.array([0.0, r_, 0.0])
    ang = np.linspace(0, 2 * np.pi, n + 1)[1:]
    pts = center + r_ * np.stack([np.zeros(n), np.cos(ang), np.sin(ang)], axis=1)
    msg = PoseArray()
    msg.poses = [pose_msg(x, R_HOME) for x in pts]
    t0 = time.monotonic()
    path_pub.publish(msg)
    T = 2 * np.pi * r_ / args.path_speed
    spin(T)
    t_end = t0 + 3 * T + 10  # then until the command is back home and still (sim: slower than real time)
    while time.monotonic() < t_end and not (np.linalg.norm(cmds[-1][1] - P_HOME) < 2e-4 and len(cmds) > 50 and
                                            np.linalg.norm(cmds[-1][1] - cmds[-50][1]) < 1e-5):
        spin(0.1)
    spin(2.0)
    ts, poses = ee_window(t0, time.monotonic())
    P = np.array([x for x, _ in poses])
    ct = np.array([c[0] for c in cmds if c[0] >= t0])
    cp = np.array([c[1] for c in cmds if c[0] >= t0])
    ref = cp[np.clip(np.searchsorted(ct, ts) - 1, 0, len(ct) - 1)]
    e_cmd = np.linalg.norm(P - ref, axis=1)
    rel = P - center
    e_circ = np.abs(np.hypot(rel[:, 1], rel[:, 2]) - r_)  # radial error in the plane
    moving = np.linalg.norm(ref - P_HOME, axis=1) > 0.005
    out["tests"]["circle"] = r = {
        "radius_mm": r_ * 1000, "path_speed": args.path_speed,
        "vs_cmd_rms_mm": round(float(np.sqrt(np.mean(e_cmd ** 2))) * 1000, 2),
        "vs_cmd_max_mm": round(float(e_cmd.max()) * 1000, 2),
        "radial_rms_mm": round(float(np.sqrt(np.mean(e_circ[moving] ** 2))) * 1000, 2) if moving.any() else None,
        "out_of_plane_max_mm": round(float(np.abs(rel[:, 0]).max()) * 1000, 2),
        "final_mm": round(float(np.linalg.norm(P[-20:] - P_HOME, axis=1).mean()) * 1000, 2),
        "cmd_gap_max_ms": round(cmd_gaps(t0, time.monotonic()), 1)}
    print(f"circle: {r}", flush=True)

print("RESULT DONE", flush=True)
finish(0)
