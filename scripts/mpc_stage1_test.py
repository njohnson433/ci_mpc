#!/usr/bin/env python3
# Stage-1 MPC test in sim (ROS side; run via run_impedance_test.sh with CHECKER=this script).
# Starts ci_mpc_planner/mpc_node per task and measures the EE from /joint_states:
#   reach:       home -> home + (0.10, 0.10, -0.05) -> home; time to 1 cm / 1 mm, final error, peak speed,
#                deviation from the straight line
#   circle:      r 5 cm, period 5 s, 2 laps (y-z plane); RMS vs the ideal circle after half a lap
#   disturbance: hold at home, 10 N push in x for 2 s (sim only), deflection and recovery
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
from ci_mpc_msgs.msg import MpcCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--sync-dir", default="")
p.add_argument("--tests", default="reach,circle,disturbance")
p.add_argument("--mpc-args", default="", help="extra '-p name:=value' for mpc_node, space separated")
p.add_argument("--log-dir", default="/tmp")
args = p.parse_args()
tests = args.tests.split(",")

full = pin.buildModelFromUrdf(args.urdf)
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM], pin.neutral(full))
data = model.createData()
ee_id = model.getFrameId("fr3_hand_tcp")


def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].translation.copy()


def sync(name):
    return os.path.join(args.sync_dir, name)


procs = []


def finish(code):
    for pr in procs:
        stop(pr)
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
node = Node("mpc_stage1_test")
samples = []


def on_js(msg):
    if all(n in msg.name for n in ARM):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        samples.append((t, np.array([msg.position[msg.name.index(n)] for n in ARM])))


node.create_subscription(JointState, "/joint_states", on_js, 100)
commands = []  # (t, commanded position) = the MPC plan at that time


def on_cmd(msg):
    t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
    commands.append((t, np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])))


node.create_subscription(MpcCommand, "/cartesian_impedance_controller/mpc_command", on_cmd, 100)


def tracking(t0, t1, shift=0.0):
    """Arm vs commanded (plan) position over [t0, t1], command stamps shifted by `shift` s: (RMS, max) in m."""
    ct = np.array([t for t, _ in commands if t0 <= t <= t1]) + shift
    cp = np.array([p for t, p in commands if t0 <= t <= t1])
    if len(ct) < 2:
        return np.nan, np.nan
    w = window(ct[0], ct[-1])
    ts = np.array([t for t, _ in w])
    pos = np.array([pp for _, pp in w])
    ref = cp[np.clip(np.searchsorted(ct, ts) - 1, 0, len(ct) - 1)]
    e = np.linalg.norm(pos - ref, axis=1)
    return np.sqrt(np.mean(e ** 2)), e.max()


def best_shift(t0, t1):
    """Time shift of the command stamps that minimizes the arm-vs-plan RMS (clock offset + arm lag)."""
    shifts = np.arange(-0.5, 0.5001, 0.005)
    rms = [tracking(t0, t1, s)[0] for s in shifts]
    k = int(np.nanargmin(rms))
    return shifts[k], rms[k]


def spin_until(pred, wall_timeout=600):
    t_end = time.time() + wall_timeout
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
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


def start_mpc(label, params):
    """Start mpc_node with the given parameters; returns (process, sim time when its first command arrives)."""
    # the installed executable directly (not `ros2 run`, whose wrapper would sit between us and the node's signals)
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ci_mpc_planner"), "lib", "ci_mpc_planner", "mpc_node")
    cmd = [exe, "--ros-args", "-p", "use_sim_time:=true"]
    for k, v in params.items():
        cmd += ["-p", f"{k}:={v}"]
    for extra in args.mpc_args.split():
        cmd += ["-p", extra]
    log = open(os.path.join(args.log_dir, f"mpc_node_{label}.log"), "w")
    pr = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    procs.append(pr)
    # wait until the node has started streaming (the controller logs nothing we can see here, so watch the log)
    t_end = time.time() + 60
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if "start pose" in open(log.name).read():
            break
        if pr.poll() is not None:
            print(f"FAIL: mpc_node ({label}) exited:\n" + open(log.name).read()[-2000:], flush=True)
            finish(2)
    return pr, now()


def stop(pr):
    if pr.poll() is None:
        pr.send_signal(signal.SIGINT)
        try:
            pr.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pr.kill()


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
p_home = fk(samples[-1][1])
print(f"home EE {np.round(p_home, 4)}", flush=True)
results = {}


def reach(label, goal, hold=6.0):
    pr, t0 = start_mpc(label, {"task": "reach", "goal": "[" + ",".join(f"{g:.5f}" for g in goal) + "]"})
    p_start = fk(samples[-1][1])
    spin_sim(hold)
    stop(pr)
    w = window(t0, now())
    ts = np.array([t for t, _ in w]) - t0
    pos = np.array([pp for _, pp in w])
    err = np.linalg.norm(pos - goal, axis=1)
    t1cm = ts[np.argmax(err < 0.01)] if (err < 0.01).any() else np.nan
    t1mm = ts[np.argmax(err < 0.001)] if (err < 0.001).any() else np.nan
    speed = np.linalg.norm(np.diff(pos, axis=0) / np.diff(ts)[:, None], axis=1)
    line = (goal - p_start) / np.linalg.norm(goal - p_start)
    rel = pos - p_start
    dev = np.linalg.norm(rel - np.outer(rel @ line, line), axis=1)
    along = rel @ line
    overshoot = max(0.0, along.max() - np.linalg.norm(goal - p_start))
    trk_rms, trk_max = tracking(t0, now())
    return dict(t1cm=t1cm, t1mm=t1mm, final=err[-1], vmax=np.percentile(speed, 99.5), dev=dev.max(),
                overshoot=overshoot, trk_rms=trk_rms, trk_max=trk_max)


if "reach" in tests:
    goal = p_home + np.array([0.10, 0.10, -0.05])
    rows = [("out (15 cm)", reach("reach_out", goal)), ("back", reach("reach_back", p_home))]
    print("REACH home <-> home + (0.10, 0.10, -0.05), MPC v_max 0.3 m/s, a_max 2 m/s^2")
    for label, r in rows:
        print(f"  {label:12s}: <1 cm {r['t1cm']:.2f} s, <1 mm {r['t1mm']:.2f} s, final err {r['final'] * 1000:.2f} mm, "
              f"peak speed {r['vmax']:.3f} m/s, max off-line {r['dev'] * 1000:.2f} mm, overshoot {r['overshoot'] * 1000:.2f} mm, "
              f"arm vs plan RMS {r['trk_rms'] * 1000:.2f} mm (max {r['trk_max'] * 1000:.2f})")
    # the MPC limits each axis to v_max, so the total speed can reach sqrt(3) v_max
    results["reach"] = all(r["final"] < 1e-3 and r["overshoot"] < 1e-3 for _, r in rows)
    print(f"  -> {'PASS' if results['reach'] else 'FAIL'} (final < 1 mm, overshoot < 1 mm)", flush=True)

if "circle" in tests:
    r_c, period, laps = 0.05, 5.0, 2
    pr, t0 = start_mpc("circle", {"task": "circle", "circle_radius": float(r_c), "circle_period": float(period),
                                  "circle_laps": float(laps)})
    p_start = fk(samples[-1][1])
    spin_sim(laps * period + 1.0)
    stop(pr)
    center = p_start - np.array([0, r_c, 0])
    w = window(t0 + period / 2, t0 + laps * period)
    ts = np.array([t for t, _ in w]) - t0
    pos = np.array([pp for _, pp in w])
    wv = 2 * np.pi / period
    ideal = center + r_c * np.stack([np.zeros_like(ts), np.cos(wv * ts), np.sin(wv * ts)], axis=1)
    e = np.linalg.norm(pos - ideal, axis=1)
    radial = np.abs(np.linalg.norm((pos - center)[:, 1:], axis=1) - r_c)
    trk_rms, trk_max = tracking(t0 + period / 2, t0 + laps * period)
    sh, sh_rms = best_shift(t0 + period / 2, t0 + laps * period)
    print(f"  arm vs plan with the command stamps shifted by {sh * 1000:+.0f} ms: RMS {sh_rms * 1000:.2f} mm", flush=True)
    print(f"CIRCLE r 5 cm / 5 s via MPC: RMS {np.sqrt(np.mean(e ** 2)) * 1000:.2f} mm (max {e.max() * 1000:.2f}) "
          f"vs the timed reference; radial (shape) error RMS {np.sqrt(np.mean(radial ** 2)) * 1000:.2f} mm; "
          f"arm vs plan RMS {trk_rms * 1000:.2f} mm (max {trk_max * 1000:.2f})")
    results["circle"] = np.sqrt(np.mean(e ** 2)) < 3e-3
    print(f"  -> {'PASS' if results['circle'] else 'FAIL'} (RMS < 3 mm)", flush=True)

if "disturbance" in tests and args.sync_dir:
    pr, t0 = start_mpc("hold", {"task": "hold"})
    p_hold = fk(samples[-1][1])
    spin_sim(1.0)
    t_push = now()
    set_wrench([10, 0, 0])
    spin_sim(2.0)
    clear_wrench()
    t_rel = now()
    spin_sim(3.0)
    stop(pr)
    w = window(t_push, now())
    ts = np.array([t for t, _ in w])
    d = np.array([pp - p_hold for _, pp in w])
    during = np.abs(d[ts <= t_rel, 0]).max()
    after1 = np.linalg.norm(d[np.argmin(np.abs(ts - t_rel - 1.0))])
    final = np.linalg.norm(d[-1])
    print(f"DISTURBANCE 10 N x for 2 s while holding: max deflection {during * 1000:.2f} mm, "
          f"error 1 s after release {after1 * 1000:.2f} mm, 3 s after {final * 1000:.2f} mm")
    results["disturbance"] = final < 1e-3
    print(f"  -> {'PASS' if results['disturbance'] else 'FAIL'} (back within 1 mm)", flush=True)

print("RESULT " + " ".join(f"{k}={'PASS' if v else 'FAIL'}" for k, v in results.items()), flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if all(results.values()) else 1)
