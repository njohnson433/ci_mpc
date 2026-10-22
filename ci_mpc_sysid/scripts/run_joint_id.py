#!/usr/bin/env python3
# Runs the joint identification on a running Controller Manager (sim or real robot) and records
# /joint_id_controller/log into an .npz file (array `log`, columns as in joint_id_controller.hpp).
#   run_joint_id.py --out id.npz [--sync-dir DIR] [--timeout 3600]
# --sync-dir: wait for <dir>/home_done before starting, write <dir>/done at exit (Isaac test runner).
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

PHASE_DONE, PHASE_ABORT = 9, -1

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--controller", default="joint_id_controller")
p.add_argument("--sync-dir", default="")
p.add_argument("--timeout", type=float, default=3600.0, help="wall-clock seconds")
args = p.parse_args()


def finish(code):
    if args.sync_dir:
        open(os.path.join(args.sync_dir, "done"), "w").close()
    sys.exit(code)


def run(cmd):
    print("$", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    print((r.stdout + r.stderr).strip(), flush=True)
    return r.returncode == 0 and "rror" not in r.stdout + r.stderr


rclpy.init()
node = Node("joint_id_runner")
rows = []
node.create_subscription(Float64MultiArray, f"/{args.controller}/log", lambda m: rows.append(list(m.data)), 1000)

cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
if not cli.wait_for_service(timeout_sec=300):
    print("FAIL: /controller_manager not available")
    finish(2)
if args.sync_dir:
    t_end = time.time() + 300
    while not os.path.exists(os.path.join(args.sync_dir, "home_done")) and time.time() < t_end:
        time.sleep(0.1)
if not run(["ros2", "control", "load_controller", "--set-state", "active", args.controller]):
    finish(2)

def save():
    log = np.array(rows)
    np.savez(args.out, log=log)
    if len(log) < 2:
        print(f"saved {len(log)} rows to {args.out}", flush=True)
        return
    dt = np.diff(log[:, 0])
    gaps = int(np.sum(dt > 1.5 * np.median(dt)))
    print(f"saved {len(log)} rows to {args.out} (sim t {log[-1, 0]:.1f} s, {gaps} gaps in the log)", flush=True)


t_end = time.time() + args.timeout
last_report = time.time()
state = None
try:
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if rows:
            state = rows[-1][2]
            if state in (PHASE_DONE, PHASE_ABORT):
                break
        if time.time() - last_report > 30 and rows:
            r = rows[-1]
            print(f"t {r[0]:7.1f} s  joint {int(r[1])}  phase {int(r[2])}  rows {len(rows)}", flush=True)
            last_report = time.time()
except BaseException:
    save()  # keep what was recorded if the run is cut off (sim exit, Ctrl-C, shutdown)
    raise

run(["ros2", "control", "set_controller_state", args.controller, "inactive"])
save()
status = {PHASE_DONE: "DONE", PHASE_ABORT: "ABORTED"}.get(state, "TIMEOUT")
print(f"RESULT {status}", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if state == PHASE_DONE else 1)
