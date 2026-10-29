#!/usr/bin/env python3
# Real FR3, free space, no contact: known set-point trajectories streamed straight to the cartesian_impedance_controller
# (MpcCommand at --rate, no mpc_node), so every controller input is known. Two uses (next session, 2026-10-29 plan):
#   arm       data for the stage-B arm model (B2: Lambda(q) from pinocchio + known controller terms, friction / damping
#             fitted; src-f1): per axis x / y / z / rx / ry / rz a step out and back, then sines at several frequencies
#             (amplitude capped by --v-max / --w-max), each with a 0.5 s ramp in and out. Twist = the set-point
#             derivative (the controller damps twist - actual twist).
#   approach  false-trigger test of stage A's contact detection (peg_insert_demo.py --smooth-approach), mid-air: from
#             --z-top down to --depth below it with the same decelerating ramp (speed gap / 0.15 s, capped at --cap,
#             at least 2 mm/s), --reps times per cap. Detector as in the sim: tare = mean force over 0.25 s before the
#             descent, then 50 ms mean of (-F_ext) z minus the tare > 1 N = "contact". Evaluated offline for both the
#             controller observer and Franka's O_F_ext_hat_K (src-35: O_F_ext_hat_K reads ~1.7 N along v while moving).
#             The arm does not stop on a trigger (there is nothing to hit); triggers are counted.
# Needs: franka_hardware + cartesian_impedance_controller active, arm at the libfranka home pose; session script
# mpc_free_space_session.sh armid | approach.
# Results: <out>/arm_id.json (segments with start/end times, approach trigger counts) and raw.npz:
#   cmd_*  every published command (monotonic time, pose xyz + quat wxyz, twist 6, K 6, D 6, integral flag)
#   rs_*   robot_state at its full rate: monotonic receive time, robot time, mode, q, dq, tau_J, tau_J_d, O_F_ext_hat_K
#   obs_*  the controller observer (~/external_wrench, base frame, Franka sign)
# (the session's background recorder writes robot_state.npz too; rs_robot_t aligns the two)
# Safety: as mpc_free_space_real.py: abort if the robot leaves Move mode, the state freezes for 0.3 s, the TCP is
# faster than --abort-speed or more than --abort-dev from the command, or leaves home by more than --box. Abort =
# stop publishing: after 50 ms the controller holds the pose it is at (mpc_timeout), so Ctrl-C is also safe.
import argparse
import gc
import json
import os
import signal
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from ci_mpc_msgs.msg import MpcCommand
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped", 6: "automatic error recovery"}
HERE = os.path.dirname(os.path.abspath(__file__))
AXES = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("mode", choices=["arm", "approach"])
p.add_argument("--axes", default="x,y,z,rx,ry,rz", help="arm: axes to excite")
p.add_argument("--step", type=float, default=0.01, help="arm: translation step [m]")
p.add_argument("--rot-step", type=float, default=0.05, help="arm: rotation step [rad]")
p.add_argument("--freqs", default="0.3,0.7,1.5,3,5", help="arm: sine frequencies [Hz]")
p.add_argument("--cycles", type=float, default=4, help="arm: sine cycles per frequency (at least 3 s)")
p.add_argument("--amp", type=float, default=0.02, help="arm: max translation amplitude [m]")
p.add_argument("--rot-amp", type=float, default=0.1, help="arm: max rotation amplitude [rad]")
p.add_argument("--v-max", type=float, default=0.08, help="arm: amplitude cap from the peak speed [m/s]")
p.add_argument("--w-max", type=float, default=0.4, help="arm: amplitude cap from the peak angular speed [rad/s]")
p.add_argument("--caps", default="0.02,0.01,0.005", help="approach: speed caps [m/s] (stage A uses 0.02)")
p.add_argument("--reps", type=int, default=5, help="approach: descents per cap")
p.add_argument("--depth", type=float, default=0.04, help="approach: descent [m]")
p.add_argument("--threshold", type=float, default=1.0, help="approach: detector threshold [N] (stage A: 1 N)")
p.add_argument("--integral", type=int, default=-1, help="integral_enabled (default: 0 for arm, 1 for approach)")
p.add_argument("--rate", type=float, default=500.0, help="command rate [Hz]")
p.add_argument("--abort-speed", type=float, default=0.2, help="[m/s]")
p.add_argument("--abort-dev", type=float, default=0.02, help="[m] TCP from the command")
p.add_argument("--box", type=float, default=0.08, help="[m] abort if the TCP is this far from home")
p.add_argument("--ee-profile", default=os.path.join(HERE, "desk_ee_hand_camera_mount_v0.1.endeffector-profile.json"))
p.add_argument("--mpc-config", default="", help="K / D from this mpc_node file (default: ci_mpc_planner mpc_real.yaml)")
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--out", required=True)
args = p.parse_args()
integral = (args.mode == "approach") if args.integral < 0 else bool(args.integral)

# K / D: the same as the MPC sends on the robot (mpc_real.yaml)
from ament_index_python.packages import get_package_share_directory  # noqa: E402

cfg = args.mpc_config or os.path.join(get_package_share_directory("ci_mpc_planner"), "config", "mpc_real.yaml")
K = D = None
for line in open(cfg):
    s = line.strip()
    if s.startswith("stiffness:"):
        K = json.loads(s.split(":", 1)[1].split("#")[0])
    elif s.startswith("damping:"):
        D = json.loads(s.split(":", 1)[1].split("#")[0])
if K is None or D is None:
    sys.exit(f"stiffness / damping not found in {cfg}")

sys.path.insert(0, os.path.join(HERE, "..", "..", "ci_mpc_planner"))
from ci_mpc_planner.ee_model import ARM_JOINTS as ARM, EeModel  # noqa: E402

ee = EeModel(args.urdf, ee_profile=args.ee_profile)
Q_HOME = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])


def fk(q):
    p_, R, _, _ = ee.update(q, np.zeros(7))
    return p_, R


P_HOME, R_HOME = fk(Q_HOME)

rclpy.init()
node = Node("arm_id_real")
state = {"q": None, "mode": None}
rs, obs, sent = [], [], []
seg = []  # (name, t0, t1, extra)


def on_js(m):
    if all(n in m.name for n in ARM):
        state["q"] = np.array([m.position[m.name.index(n)] for n in ARM])
        state["js_t"] = time.monotonic()


def on_rs(m):
    t = time.monotonic()
    js, jd, w = m.measured_joint_state, m.desired_joint_state, m.o_f_ext_hat_k.wrench
    if len(js.position) != 7 or len(js.effort) != 7 or len(jd.effort) != 7:
        return
    tau = tuple(js.effort)
    if tau != state.get("tau"):  # never bit-identical while running; frozen after a reflex
        state["tau"], state["tau_t"] = tau, t
    state["mode"] = m.robot_mode
    rs.append([t, m.time, m.robot_mode, *js.position, *js.velocity, *js.effort, *jd.effort,
               w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])


def on_obs(m):
    w = m.wrench
    obs.append([time.monotonic(), w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])


node.create_subscription(JointState, "/joint_states", on_js, qos_profile_sensor_data)
node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", on_rs, 100)
node.create_subscription(WrenchStamped, "/cartesian_impedance_controller/external_wrench", on_obs, 100)
pub = node.create_publisher(MpcCommand, "/cartesian_impedance_controller/mpc_command", qos_profile_sensor_data)
out = {"mode": args.mode, "args": vars(args), "K": K, "D": D, "integral_enabled": integral, "config": cfg,
       "segments": []}
flags = {"sigint": False, "done": False}
signal.signal(signal.SIGINT, lambda *_: flags.__setitem__("sigint", True))


def finish(code):
    flags["done"] = True  # stop publishing: the controller holds the current pose after mpc_timeout
    os.makedirs(args.out, exist_ok=True)
    out["segments"] = [{"name": n, "t0": t0, "t1": t1, **x} for n, t0, t1, x in seg]
    a = np.array(sent) if sent else np.zeros((0, 21))
    r = np.array(rs) if rs else np.zeros((0, 37))
    o = np.array(obs) if obs else np.zeros((0, 7))
    np.savez_compressed(os.path.join(args.out, "raw.npz"),
                        cmd_t=a[:, 0], cmd_pose=a[:, 1:8], cmd_twist=a[:, 8:14], cmd_K=np.array(K),
                        cmd_D=np.array(D), cmd_integral=integral,
                        rs_t=r[:, 0], rs_robot_t=r[:, 1], rs_mode=r[:, 2], rs_q=r[:, 3:10], rs_dq=r[:, 10:17],
                        rs_tau_J=r[:, 17:24], rs_tau_J_d=r[:, 24:31], rs_O_F_ext=r[:, 31:37],
                        obs_t=o[:, 0], obs_F=o[:, 1:7])
    json.dump(out, open(os.path.join(args.out, "arm_id.json"), "w"), indent=1)
    print(f"results: {args.out} ({len(a)} commands, {len(r)} robot states, {len(o)} observer samples)", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


def abort(msg):
    print("ABORT: " + msg, flush=True)
    out["abort"] = msg
    finish(2)


def publish(pos, R, v, w):
    m = MpcCommand()
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
    qt = pin.Quaternion(R)
    m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = qt.w, qt.x, qt.y, qt.z
    m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v)
    m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = map(float, w)
    m.stiffness = [float(k) for k in K]
    m.damping = [float(d) for d in D]
    m.integral_enabled = integral
    pub.publish(m)
    sent.append([time.monotonic(), *pos, qt.w, qt.x, qt.y, qt.z, *v, *w])


watch = {"prev": None}


def check(x_cmd):
    now = time.monotonic()
    if flags["sigint"]:
        abort("stopped by Ctrl-C")
    if now - state.get("tau_t", now) > 0.3 or now - state.get("js_t", now) > 0.3:
        abort("robot state frozen / no /joint_states for 0.3 s: the robot stopped (reflex? see launch.log)")
    if state["mode"] not in (None, 2):
        abort(f"robot left Move mode: {MODES.get(state['mode'], state['mode'])} (see launch.log)")
    x = fk(state["q"])[0]
    if np.linalg.norm(x - P_HOME) > args.box:
        abort(f"TCP {np.round(x, 3).tolist()} is > {args.box * 1000:.0f} mm from home")
    if np.linalg.norm(x - x_cmd) > args.abort_dev:
        abort(f"TCP {np.linalg.norm(x - x_cmd) * 1000:.1f} mm from the command")
    if watch["prev"] is None or now - watch["prev"][0] >= 0.05:  # speed over >= 50 ms (joint_states jitter)
        if watch["prev"] is not None:
            v = float(np.linalg.norm(x - watch["prev"][1]) / (now - watch["prev"][0]))
            if v > args.abort_speed:
                abort(f"TCP speed {v:.3f} m/s")
        watch["prev"] = (now, x)


def run(name, T, f, **extra):
    """Stream f(t) -> (pos, R, v, w) for T seconds at --rate; record the segment."""
    dt = 1.0 / args.rate
    t0 = time.monotonic()
    t_next = t0
    while True:
        now = time.monotonic()
        t = now - t0
        if t > T:
            break
        if now >= t_next:
            pos, R, v, w = f(min(t, T))
            publish(pos, R, v, w)
            t_next += dt
            check(pos)
        rclpy.spin_once(node, timeout_sec=max(0.0, min(0.001, t_next - time.monotonic())))
    seg.append((name, t0, time.monotonic(), extra))


def offset(d6):
    """Set-point = home displaced by d6 (translation [m], rotation vector [rad] in the world frame)."""
    return P_HOME + d6[:3], pin.exp3(np.asarray(d6[3:], dtype=float)) @ R_HOME


def hold(T, name="hold", pos=None):
    pos = P_HOME if pos is None else pos
    run(name, T, lambda t: (pos, R_HOME, np.zeros(3), np.zeros(3)))


def smooth_step(name, ax, size, T=2.5, t_rise=0.0):
    # a step in the set-point (t_rise 0) excites the arm the most; the twist feed-forward is zero for a step
    def f(t):
        d = np.zeros(6)
        d[ax] = size
        pos, R = offset(d)
        return pos, R, np.zeros(3), np.zeros(3)
    run(name, T, f, axis=ax, size=size)


def sine(name, ax, amp, freq):
    T_sin = max(args.cycles / freq, 3.0)
    T = T_sin + 1.0  # 0.5 s ramp in + out (envelope), so the set-point and its rate start and end at zero

    def f(t):
        env, denv = 1.0, 0.0
        if t < 0.5:
            env, denv = 0.5 - 0.5 * np.cos(np.pi * t / 0.5), 0.5 * np.pi / 0.5 * np.sin(np.pi * t / 0.5)
        elif t > T - 0.5:
            s = T - t
            env, denv = 0.5 - 0.5 * np.cos(np.pi * s / 0.5), -0.5 * np.pi / 0.5 * np.sin(np.pi * s / 0.5)
        ph = 2 * np.pi * freq * t
        d = np.zeros(6)
        dd = np.zeros(6)
        d[ax] = amp * env * np.sin(ph)
        dd[ax] = amp * (denv * np.sin(ph) + env * 2 * np.pi * freq * np.cos(ph))
        pos, R = offset(d)
        return pos, R, dd[:3], dd[3:]
    run(name, T, f, axis=ax, amp=amp, freq=freq)


# --- start
# no cyclic garbage here; the collector's full passes over the growing logs stalled the sender 23 -> 151 ms every ~37 s
# (armid_1639, 2026-10-29), and the controller holds after 50 ms without a command
gc.disable()
t_end = time.monotonic() + 10
while (state["q"] is None or state["mode"] is None) and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
if state["q"] is None or state["mode"] is None:
    print("no /joint_states or robot_state (is franka_hardware running, ROS_DOMAIN_ID?)")
    finish(1)
x0, R0 = fk(state["q"])
far, rot = np.linalg.norm(x0 - P_HOME), np.linalg.norm(pin.log3(R_HOME @ R0.T))
print(f"start TCP {np.round(x0, 4).tolist()}, {far * 1000:.1f} mm / {rot * 1000:.1f} mrad from home; K {K}; D {D}; "
      f"integral {integral}", flush=True)
if far > 0.01 or rot > 0.05:
    print("start is not at home: run it from the session script (move_to_start home first)")
    finish(1)
# glide the set-point from the measured pose to the exact home pose (move_to_start ends within ~1 mm), 2 s
dp, drot = P_HOME - x0, pin.log3(R_HOME @ R0.T)


def glide(t):
    s = min(t / 2.0, 1.0)
    a = 0.5 - 0.5 * np.cos(np.pi * s)
    da = 0.5 * np.pi / 2.0 * np.sin(np.pi * s) if s < 1 else 0.0
    return x0 + a * dp, pin.exp3(a * drot) @ R0, da * dp, da * drot


# frame check: the first commands are the measured pose; if the controller's EE frame differs the arm jumps
run("frame_check", 0.5, lambda t: (x0, R0, np.zeros(3), np.zeros(3)))
moved = np.linalg.norm(fk(state["q"])[0] - x0)
if moved > 0.002:
    abort(f"the arm moved {moved * 1000:.1f} mm on the first commands: EE frame mismatch (--ee-profile / Desk profile?)")
run("glide_home", 2.5, glide)
hold(3.0, "hold_start")

if args.mode == "arm":
    freqs = [float(v) for v in args.freqs.split(",")]
    for name in args.axes.split(","):
        ax = AXES[name]
        rotational = ax >= 3
        st, amp_max, vel_max = ((args.rot_step, args.rot_amp, args.w_max) if rotational
                                else (args.step, args.amp, args.v_max))
        print(f"{name}: steps +-{st}, sines {freqs} Hz", flush=True)
        for sgn in (1, -1):
            smooth_step(f"{name}_step{'+' if sgn > 0 else '-'}", ax, sgn * st)
            smooth_step(f"{name}_step{'+' if sgn > 0 else '-'}_back", ax, 0.0)
        for fr in freqs:
            amp = min(amp_max, vel_max / (2 * np.pi * fr))
            sine(f"{name}_sin{fr:g}", ax, amp, fr)
            hold(1.0, f"{name}_sin{fr:g}_settle")
    print("RESULT DONE", flush=True)
    finish(0)

# approach: decelerating descents in free air, same law as peg_insert_demo.py --smooth-approach
caps = [float(v) for v in args.caps.split(",")]
z_top = P_HOME[2]
z_bot = z_top - args.depth
res = out["approach"] = []
for cap in caps:
    for rep in range(args.reps):
        hold(0.6, f"tare_{cap:g}_{rep}")  # the detector tares over the last 0.25 s of this
        zc = {"z": z_top, "t": None}

        def descend(t):
            if zc["t"] is None:
                zc["t"] = t
            dt = t - zc["t"]
            zc["t"] = t
            v = min(cap, max(0.002, (zc["z"] - z_bot) / 0.15)) if zc["z"] > z_bot else 0.0
            zc["z"] = max(z_bot, zc["z"] - v * dt)
            return np.array([P_HOME[0], P_HOME[1], zc["z"]]), R_HOME, np.array([0.0, 0.0, -v]), np.zeros(3)

        T = args.depth / cap + 1.5
        run(f"descent_{cap:g}_{rep}", T, descend, cap=cap, rep=rep)
        hold(1.0, f"bottom_{cap:g}_{rep}", pos=np.array([P_HOME[0], P_HOME[1], z_bot]))  # (was P_HOME: a 4 cm jump)
        # back up at the cap speed (a smooth ramp; the detector is only evaluated on the way down)
        t_up = args.depth / cap

        def up(t):
            s = min(t / t_up, 1.0)
            a = 0.5 - 0.5 * np.cos(np.pi * s)
            da = 0.5 * np.pi / t_up * np.sin(np.pi * s) if s < 1 else 0.0
            return (np.array([P_HOME[0], P_HOME[1], z_bot + a * args.depth]), R_HOME,
                    np.array([0.0, 0.0, da * args.depth]), np.zeros(3))
        run(f"up_{cap:g}_{rep}", t_up + 0.5, up)


def detect(t, F, tare_win, win):
    """stage A detector on one descent: tare (mean -F z over the last 0.25 s before), 50 ms means > threshold."""
    tt = (t >= tare_win[1] - 0.25) & (t <= tare_win[1])
    if not tt.any():
        return None
    tare = -F[tt, 2].mean()
    m = (t >= win[0]) & (t <= win[1])
    ts, fz = t[m], -F[m, 2] - tare
    if len(ts) < 5:
        return None
    k = np.searchsorted(ts, ts - 0.05)
    c = np.concatenate([[0.0], np.cumsum(fz)])
    idx = np.arange(len(ts))
    mean50 = (c[idx + 1] - c[k]) / np.maximum(idx + 1 - k, 1)
    hit = np.flatnonzero(mean50 > args.threshold)
    return {"tare_N": round(float(tare), 3), "fz50_max_N": round(float(mean50.max()), 3),
            "fz50_min_N": round(float(mean50.min()), 3), "fz50_p50_N": round(float(np.median(mean50)), 3),
            "trigger": bool(len(hit)), "t_trigger_s": round(float(ts[hit[0]] - win[0]), 3) if len(hit) else None}


r = np.array(rs)
o = np.array(obs) if obs else np.zeros((0, 7))
by_name = {n: (t0, t1) for n, t0, t1, _ in seg}
summary = {}
for cap in caps:
    for rep in range(args.reps):
        win, tare = by_name[f"descent_{cap:g}_{rep}"], by_name[f"tare_{cap:g}_{rep}"]
        row = {"cap": cap, "rep": rep,
               "franka": detect(r[:, 0], r[:, 31:37], tare, win),
               "observer": detect(o[:, 0], o[:, 1:7], tare, win) if len(o) else None}
        res.append(row)
        for src in ("franka", "observer"):
            if row[src] is not None:
                s = summary.setdefault(f"{src}_{cap:g}", {"n": 0, "triggers": 0, "fz50_max": []})
                s["n"] += 1
                s["triggers"] += int(row[src]["trigger"])
                s["fz50_max"].append(row[src]["fz50_max_N"])
for k, s in summary.items():
    s["fz50_max"] = [round(float(np.min(s["fz50_max"])), 2), round(float(np.median(s["fz50_max"])), 2),
                     round(float(np.max(s["fz50_max"])), 2)]  # min / median / max over the descents
    print(f"approach {k}: {s['triggers']}/{s['n']} false triggers (> {args.threshold} N); "
          f"peak 50 ms Fz min/median/max {s['fz50_max']} N", flush=True)
out["approach_summary"] = summary
print("RESULT DONE", flush=True)
finish(0)
