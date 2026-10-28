#!/usr/bin/env python3
# Records the real FR3 during a joint-space move (Franka's move_to_start) and checks that it arrived.
# Started in the background before the move; stopped (SIGINT) after it, then prints the final joint error and saves
# q, measured torque tau_J, commanded torque tau_J_d (robot_state) and the robot mode at ~100 Hz to --out (.npz).
# Exit code 3 if any joint is more than --tol from the target.
# --switch-to NAME: when the move is done (move_to_start then commands exactly zero torque) switch from
# move_to_start_example_controller to NAME (loaded + configured, inactive) in one controller_manager call, then exit.
# Why: with zero commanded torque only the robot's own gravity compensation holds the arm, and joint 2 sinks
# (~0.1 rad/s, 0.38 rad in the 4 s the session took to notice "finished"; 2026-10-28).
import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from franka_msgs.msg import FrankaRobotState
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node

p = argparse.ArgumentParser()
p.add_argument("--target", required=True, help="7 joint values, comma separated")
p.add_argument("--tol", type=float, default=0.02, help="[rad]")
p.add_argument("--out", required=True)
p.add_argument("--switch-to", default="", help="controller to activate as soon as the move is done")
p.add_argument("--timeout", type=float, default=60.0, help="[s] with --switch-to: give up (exit 4)")
args = p.parse_args()
target = np.array([float(v) for v in args.target.split(",")])

rclpy.init()
node = Node("move_monitor")
rows = []


def on_state(m):
    q = list(m.measured_joint_state.position)
    tau = list(m.measured_joint_state.effort)
    tau_d = list(m.desired_joint_state.effort)
    if len(q) == 7 and len(tau) == 7 and len(tau_d) == 7:
        rows.append([time.monotonic(), m.robot_mode, *q, *tau, *tau_d])


node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", on_state, 10)
stop = {"now": False}
signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))
signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("now", True))
cli = node.create_client(SwitchController, "/controller_manager/switch_controller")
t_end = time.monotonic() + args.timeout
moving, zeros, switched = False, 0, False
while not stop["now"]:
    rclpy.spin_once(node, timeout_sec=0.005)
    if not args.switch_to or switched or not rows:
        continue
    r = rows[-1]
    if r[1] == 2 and any(abs(v) > 0 for v in r[16:23]):
        moving, zeros = True, 0
    elif moving and r[1] == 2 and all(v == 0 for v in r[16:23]):
        zeros += 1
    if zeros >= 3:  # move_to_start finished: hand over now
        req = SwitchController.Request()
        req.activate_controllers = [args.switch_to]
        req.deactivate_controllers = ["move_to_start_example_controller"]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        ok = fut.result() is not None and fut.result().ok
        print(f"move: done, switched to {args.switch_to}: {'ok' if ok else 'FAILED'}", flush=True)
        switched = True
        if not ok:
            break
        t_stop = time.monotonic() + 1.0  # record 1 s of the hold
        while time.monotonic() < t_stop:
            rclpy.spin_once(node, timeout_sec=0.005)
        break
    if time.monotonic() > t_end:
        print(f"move: not done after {args.timeout:.0f} s", flush=True)
        break
a = np.array(rows)
np.savez_compressed(args.out, log=a, target=target, cols="t mode q1..q7 tau1..tau7 tau_d1..tau_d7")
if len(a) == 0:
    print("move monitor: no robot_state received")
    sys.exit(2)
err = a[-1, 2:9] - target
k = int(np.argmax(np.abs(err)))
print(f"move: {len(a)} samples over {a[-1, 0] - a[0, 0]:.1f} s; final joint error [rad] {np.round(err, 3).tolist()}; "
      f"peak |tau_d| {np.round(np.abs(a[:, 16:23]).max(0), 1).tolist()} Nm; modes {sorted(set(a[:, 1].astype(int)))}")
if args.switch_to and not switched:
    sys.exit(4)
if args.switch_to and not ok:
    sys.exit(5)
if abs(err[k]) > args.tol:
    print(f"move: joint {k + 1} is {err[k]:+.3f} rad from the target (tol {args.tol}); log {args.out}")
    sys.exit(3)
sys.exit(0)
