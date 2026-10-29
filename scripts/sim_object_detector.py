#!/usr/bin/env python3
# Sim stand-in for the vision pipeline: publishes the pick-place objects (one detection each, id = box name) as vision_msgs/Detection3DArray on
# /perception/detections (--rate, default 10 Hz, frame "base" = robot base) from the sim ground truth (<sync>/object.json),
# with a fixed estimation error per run: position offset ~ N(0, pos_noise) in x/y/z, size ~ N(0, size_noise)
# per extent, yaw ~ N(0, yaw_noise_deg). The real vision pipeline publishes the same message.
#   sim_object_detector.py --sync-dir DIR [--pos-noise 0.003] [--size-noise 0.002] [--yaw-noise-deg 3] [--seed 1]
import argparse
import json
import os

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

p = argparse.ArgumentParser()
p.add_argument("--sync-dir", required=True)
p.add_argument("--pos-noise", type=float, default=0.0, help="std of the position error [m]")
p.add_argument("--pos-noise-z", type=float, default=-1.0, help="std of the z position error [m] (default: --pos-noise; a depth plane fit of a top face is better than edge localization in xy)")
p.add_argument("--size-noise", type=float, default=0.0, help="std of the size error [m]")
p.add_argument("--yaw-noise-deg", type=float, default=0.0, help="std of the yaw error [deg]")
p.add_argument("--seed", type=int, default=1)
p.add_argument("--rate", type=float, default=10.0, help="publish rate [Hz] (wall time)")
args, _ = p.parse_known_args()

rng = np.random.default_rng(args.seed)
errors = []  # fixed estimation error per object: (position, size, yaw)


def error(i):
    while len(errors) <= i:
        sz = args.pos_noise if args.pos_noise_z < 0 else args.pos_noise_z
        errors.append((rng.normal(0, [args.pos_noise, args.pos_noise, sz]), rng.normal(0, args.size_noise, 3),
                       np.radians(rng.normal(0, args.yaw_noise_deg))))
    return errors[i]


class Detector(Node):
    def __init__(self):
        super().__init__("sim_object_detector")
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self.create_timer(1.0 / args.rate, self.tick)
        d_pos, d_size, d_yaw = error(0)
        self.get_logger().info(f"estimation error (first object): position {np.round(d_pos * 1000, 1)} mm, "
                               f"size {np.round(d_size * 1000, 1)} mm, yaw {np.degrees(d_yaw):.1f} deg")

    def tick(self):
        try:
            data = json.load(open(os.path.join(args.sync_dir, "object.json")))
        except (OSError, ValueError):
            return
        msg = Detection3DArray()
        msg.header.frame_id = "base"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, o in enumerate(data.get("objects", [{**data, "name": "box"}])):
            msg.detections.append(self.detection(o, *error(i), msg.header))
        self.pub.publish(msg)

    def detection(self, o, d_pos, d_size, d_yaw, header):
        w, x, y, z = o["quat_wxyz"]
        R = pin.Quaternion(w, x, y, z).normalized().toRotationMatrix()
        R = pin.exp3(np.array([0, 0, d_yaw])) @ R
        q = pin.Quaternion(R)
        det = Detection3D()
        det.header = header
        det.id = o["name"]
        c = np.array(o["cube"]) + d_pos
        det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = map(float, c)
        det.bbox.center.orientation.x, det.bbox.center.orientation.y = q.x, q.y
        det.bbox.center.orientation.z, det.bbox.center.orientation.w = q.z, q.w
        size = np.maximum(np.array(o["size"]) + d_size, 0.005)
        det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = map(float, size)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "box"
        hyp.hypothesis.score = 1.0
        det.results.append(hyp)
        return det


rclpy.init()
node = Detector()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
