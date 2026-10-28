#!/usr/bin/env python3
# Runs the joint identification on a running Controller Manager (sim or real robot) and records
# /joint_id_controller/log into an .npz file (array `log`, columns as in joint_id_controller.hpp).
#   run_joint_id.py --out id.npz [--sync-dir DIR] [--timeout 3600] [--param-file real.yaml]
# --param-file: load the controller with the spawner (type ci_mpc_sysid/JointIdController, these parameters) instead of
#   expecting it in the Controller Manager config (real robot: franka_bringup's controllers.yaml does not list it);
#   the controller is unloaded at the end so the next run can load it again.
# --switch-from NAME (with --param-file): spawn inactive, then switch NAME -> this controller in one call (real robot:
#   the cartesian_impedance_controller holds the arm after the move, there is no unpowered gap).
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

try:  # real robot only: a reflex stops the arm while the controller keeps logging
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None

PHASE_DONE, PHASE_ABORT = 9, -1

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--controller", default="joint_id_controller")
p.add_argument("--sync-dir", default="")
p.add_argument("--param-file", default="", help="spawn with this parameter file (see above)")
p.add_argument("--switch-from", default="", help="active controller to replace in one switch (needs --param-file)")
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
mode = {"m": None, "active": False, "tau": None, "tau_t": time.time()}
if FrankaRobotState is not None:
    node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state",
                             lambda m: mode.__setitem__("m", m.robot_mode), 10)


def on_js(msg):  # measured torques are never bit-identical on the real arm; after a reflex they freeze
    tau = tuple(msg.effort)
    if tau != mode["tau"]:
        mode["tau"], mode["tau_t"] = tau, time.time()


from sensor_msgs.msg import JointState  # noqa: E402
node.create_subscription(JointState, "/joint_states", on_js, 100)

cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
if not cli.wait_for_service(timeout_sec=300):
    print("FAIL: /controller_manager not available")
    finish(2)
if args.sync_dir:
    t_end = time.time() + 300
    while not os.path.exists(os.path.join(args.sync_dir, "home_done")) and time.time() < t_end:
        time.sleep(0.1)
if args.param_file:
    ok = run(["ros2", "run", "controller_manager", "spawner", args.controller, "-c", "/controller_manager",
              "-t", "ci_mpc_sysid/JointIdController", "--param-file", args.param_file]
             + (["--inactive"] if args.switch_from else []))
    if ok and args.switch_from:
        ok = run(["ros2", "control", "switch_controllers", "--deactivate", args.switch_from,
                  "--activate", args.controller, "--strict", "--activate-asap"])
else:
    ok = run(["ros2", "control", "load_controller", "--set-state", "active", args.controller])
if not ok:
    finish(2)
# nothing was spun while the spawner ran (blocking): restart the frozen-state clock, or the first log row triggers a
# false "frozen" abort (real robot, 2026-10-28)
mode["tau_t"] = time.time()

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
            if mode["m"] is not None and time.time() - mode["tau_t"] > 0.3:  # real robot only (robot_state seen)
                print("ABORT: robot state frozen for 0.3 s: the robot stopped (reflex? see launch.log)", flush=True)
                state = PHASE_ABORT
                break
            if mode["m"] == 2:
                mode["active"] = True
            elif mode["active"] and mode["m"] is not None:  # was moving, now reflex / stopped
                print(f"ABORT: robot left Move mode (mode {mode['m']}; reflex, see launch.log)", flush=True)
                state = PHASE_ABORT
                break
        if time.time() - last_report > 30 and rows:
            r = rows[-1]
            print(f"t {r[0]:7.1f} s  joint {int(r[1])}  phase {int(r[2])}  rows {len(rows)}", flush=True)
            last_report = time.time()
except BaseException:
    save()  # keep what was recorded if the run is cut off (sim exit, Ctrl-C, shutdown)
    raise

run(["ros2", "control", "set_controller_state", args.controller, "inactive"])
if args.param_file:
    run(["ros2", "control", "unload_controller", args.controller])
save()
status = {PHASE_DONE: "DONE", PHASE_ABORT: "ABORTED"}.get(state, "TIMEOUT")
print(f"RESULT {status}", flush=True)
node.destroy_node()
rclpy.shutdown()
finish(0 if state == PHASE_DONE else 1)
