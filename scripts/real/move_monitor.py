#!/usr/bin/env python3
# Records the real FR3 during a joint-space move (Franka's move_to_start) and checks that it arrived.
# Started in the background before the move; stopped (SIGINT) after it, then prints the final joint error and saves
# q, measured torque tau_J, commanded torque tau_J_d (robot_state) and the robot mode at ~100 Hz to --out (.npz).
# Exit code 3 if any joint is more than --tol from the target (2026-10-28: joint 2 stopped 0.35 rad short of home
# twice while the other joints arrived).
import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from franka_msgs.msg import FrankaRobotState
from rclpy.node import Node

p = argparse.ArgumentParser()
p.add_argument("--target", required=True, help="7 joint values, comma separated")
p.add_argument("--tol", type=float, default=0.02, help="[rad]")
p.add_argument("--out", required=True)
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
while not stop["now"]:
    rclpy.spin_once(node, timeout_sec=0.05)
a = np.array(rows)
np.savez_compressed(args.out, log=a, target=target, cols="t mode q1..q7 tau1..tau7 tau_d1..tau_d7")
if len(a) == 0:
    print("move monitor: no robot_state received")
    sys.exit(2)
err = a[-1, 2:9] - target
k = int(np.argmax(np.abs(err)))
print(f"move: {len(a)} samples over {a[-1, 0] - a[0, 0]:.1f} s; final joint error [rad] {np.round(err, 3).tolist()}; "
      f"peak |tau_d| {np.round(np.abs(a[:, 16:23]).max(0), 1).tolist()} Nm; modes {sorted(set(a[:, 1].astype(int)))}")
if abs(err[k]) > args.tol:
    print(f"move: joint {k + 1} is {err[k]:+.3f} rad from the target (tol {args.tol}); log {args.out}")
    sys.exit(3)
sys.exit(0)
