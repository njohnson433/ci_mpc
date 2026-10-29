#!/usr/bin/env python3
# Background recorder for real-FR3 sessions: franka_robot_state_broadcaster/robot_state at --rate (default 200 Hz of the
# ~1 kHz stream), saved as float32 to --out (.npz) on SIGINT / SIGTERM. Started by the session scripts after the launch.
# Columns (key `cols`): t (robot time), mode, success_rate, q1..7, dq1..7, tau_J1..7 (measured link torque),
# tau_J_d1..7 (commanded, without gravity), tau_ext1..7 (tau_ext_hat_filtered), theta1..7 (motor position),
# O_F_ext_hat_K fx fy fz tx ty tz.
# Why: per-joint fit of how Franka's external-force estimate reads held / kinetic joint friction (src-35, 2026-10-29).
import argparse
import signal
import sys

import numpy as np
import rclpy
from franka_msgs.msg import FrankaRobotState
from rclpy.node import Node

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--rate", type=float, default=200.0, help="[Hz] kept samples (robot_state comes at ~1 kHz)")
args = p.parse_args()

J7 = [f"{n}{i}" for n in ("q", "dq", "tau_J", "tau_J_d", "tau_ext", "theta") for i in range(1, 8)]
COLS = ["t", "mode", "success_rate"] + J7 + ["fx", "fy", "fz", "tx", "ty", "tz"]
rows = []
last = {"t": -1.0}


def on_state(m):
    t = m.time
    if t - last["t"] < 1.0 / args.rate - 1e-4:
        return
    js, jd, je, jm = m.measured_joint_state, m.desired_joint_state, m.tau_ext_hat_filtered, m.measured_joint_motor_state
    if len(js.position) != 7:
        return
    w = m.o_f_ext_hat_k.wrench
    pad = lambda v: list(v) if len(v) == 7 else [np.nan] * 7  # noqa: E731
    rows.append([t, m.robot_mode, m.control_command_success_rate] + list(js.position) + list(js.velocity)
                + pad(js.effort) + pad(jd.effort) + pad(je.effort) + pad(jm.position)
                + [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
    last["t"] = t


rclpy.init()
node = Node("state_recorder")
node.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", on_state, 100)
stop = {"now": False}
signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))
signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("now", True))
while not stop["now"]:
    rclpy.spin_once(node, timeout_sec=0.05)
a = np.array(rows, dtype=np.float32) if rows else np.zeros((0, len(COLS)), np.float32)
np.savez_compressed(args.out, data=a, cols=" ".join(COLS))
print(f"state recorder: {len(a)} samples -> {args.out}", flush=True)
node.destroy_node()
rclpy.try_shutdown()
sys.exit(0)
