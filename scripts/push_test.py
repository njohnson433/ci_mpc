#!/usr/bin/env python3
# Stage-2 pushing test in sim (ROS side; run via run_impedance_test.sh with PUSH=1 CHECKER=this script,
# GUI=1 to watch). Starts the sim object detector (vision stand-in) and ci_mpc_planner push_node, waits for
# the push to finish and scores it from the push node's log:
#   tracking  slider position vs the reference during the push (RMS, max), final position / yaw error,
#   contact   p_y drift (the contact sliding along the face), largest rod-face gap, lost-contact solves.
#   push_test.py --sync-dir DIR [--task line|arc] [--push-args name:=value ...] [--pos-noise ...] [--out x.json]
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument("--sync-dir", required=True)
p.add_argument("--log-dir", default="/tmp")
p.add_argument("--task", default="line", choices=["line", "arc"])
p.add_argument("--push-args", nargs="*", default=[], help="extra push_node parameters name:=value")
p.add_argument("--pos-noise", type=float, default=0.0)
p.add_argument("--yaw-noise-deg", type=float, default=0.0)
p.add_argument("--det-rate", type=float, default=30.0)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--timeout", type=float, default=120.0, help="sim seconds for the whole push")
p.add_argument("--out", default="")
args = p.parse_args()


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
node = Node("push_test")
state = {"q": None, "t": 0.0}


def on_js(msg):
    if "fr3_joint1" in msg.name:
        state["q"] = True
        state["t"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


node.create_subscription(JointState, "/joint_states", on_js, 100)


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
spin_sim(1.0)

here = os.path.dirname(os.path.abspath(__file__))
det_log = open(os.path.join(args.log_dir, "sim_object_detector_push.log"), "w")
procs.append(subprocess.Popen([sys.executable, os.path.join(here, "sim_object_detector.py"), "--sync-dir", args.sync_dir,
                               "--pos-noise", str(args.pos_noise), "--yaw-noise-deg", str(args.yaw_noise_deg),
                               "--seed", str(args.seed), "--rate", str(args.det_rate)],
                              stdout=det_log, stderr=subprocess.STDOUT))
exe = os.path.join(get_package_prefix("ci_mpc_planner"), "lib", "ci_mpc_planner", "push_node")
npz = os.path.join(args.log_dir, f"push_{args.task}.npz")
if os.path.exists(npz):
    os.remove(npz)
push_log_path = os.path.join(args.log_dir, f"push_node_{args.task}.log")
push_log = open(push_log_path, "w")
cmd = [exe, "--ros-args", "-r", "__node:=ci_mpc_push", "-p", "use_sim_time:=true", "-p", f"task:={args.task}",
       "-p", f"log_path:={npz}"]
for a in args.push_args:
    cmd += ["-p", a]
print("$", " ".join(cmd), flush=True)
procs.append(subprocess.Popen(cmd, stdout=push_log, stderr=subprocess.STDOUT))
t_start = state["t"]


def log_text():
    return open(push_log_path).read()


ok = spin_until(lambda: "FINISHED" in log_text() or "Traceback" in log_text() or procs[-1].poll() is not None
                or state["t"] - t_start > args.timeout, 3600)
text = log_text()
for line in text.splitlines():
    if any(k in line for k in ("slider", "contact", "PUSH DONE", "FINISHED", "MPC", "Error", "Traceback", "task")):
        print("  " + line.split("]: ")[-1] if "]: " in line else "  " + line, flush=True)
if "PUSH DONE" not in text:
    print("FAIL: push did not finish (timeout or crash)\n" + text[-3000:])
    finish(1)
spin_until(lambda: os.path.exists(npz), 10)
d = np.load(npz)
ph = d["phase"]
m = ph == "push"
s, s_ref = d["s"][m], d["s_ref"][m]
e = np.linalg.norm(s[:, :2] - s_ref[:, :2], axis=1)
eth = np.degrees(np.abs((s[:, 2] - s_ref[:, 2] + np.pi) % (2 * np.pi) - np.pi))
py, gap = d["py"][m], d["gap"][m]
truth = json.load(open(sync("object.json")))
res = {
    "task": args.task, "push_args": args.push_args,
    "rms_mm": float(np.sqrt(np.mean(e ** 2)) * 1000), "max_mm": float(e.max() * 1000),
    "final_mm": float(e[-1] * 1000), "final_yaw_deg": float(eth[-1]), "max_yaw_deg": float(eth.max()),
    "py_start_mm": float(py[0] * 1000), "py_end_mm": float(py[-1] * 1000),
    "py_range_mm": float((py.max() - py.min()) * 1000),
    "gap_max_mm": float(gap.max() * 1000), "gap_mean_mm": float(gap.mean() * 1000),
    "lost_solves": int((gap > 0.005).sum()), "solves": int(m.sum()),
    "solve_ms_mean": float(d["solve_ms"][m].mean()), "solve_ms_max": float(d["solve_ms"][m].max()),
    "slider_z": float(truth["cube"][2]), "push_s": float(d["t"][m][-1] - d["t"][m][0]),
}
print("RESULT " + ", ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in res.items()), flush=True)
if args.out:
    json.dump(res, open(args.out, "w"), indent=1)
node.destroy_node()
rclpy.shutdown()
finish(0 if res["final_mm"] < 5 and res["lost_solves"] == 0 and abs(res["slider_z"] - 0.225) < 0.005 else 1)
