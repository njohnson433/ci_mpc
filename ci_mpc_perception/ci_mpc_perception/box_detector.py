"""Box detector on the wrist camera (Gemini 305 topics): publishes vision_msgs/Detection3DArray on
/perception/detections (frame "base", one detection per box, id = box name) from color + aligned depth.

Pipeline per frame (color and depth paired by stamp, camera pose from /joint_states at that stamp via Pinocchio FK of
the hand and the mount in the camera config; on the real robot: the hand-eye calibration):
  1. 2D: colored-object masks (HSV thresholds; the stand-in for a learned detector such as YOLO + its class ->
     box name), largest connected component per color, at least `min_pixels`.
  2. 3D: the mask's valid depth pixels back-projected into the base frame.
  3. Box fit: top face = points within `top_band` of the highest; center xy, yaw (mod 90 deg) and xy extent from
     the minimum-area rectangle of the top face (cv2.minAreaRect); height = top - lowest visible point of the box
     (its sides; 2nd percentile); if too few side points: top - table height (`table_z`). Center z = top - h / 2.
Assumes boxes lie flat (yaw only). Parameters: config, colors (name:color pairs), min_pixels, top_band, joint_stamp.
"""
import bisect
import collections

import cv2
import numpy as np
import pinocchio as pin
import rclpy
import yaml
from rclpy.node import Node
from scipy import ndimage
from sensor_msgs.msg import CameraInfo, Image, JointState
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from ci_mpc_perception.camera_model import optical_in_hand

DEFAULT_CONFIG = "/home/njohnson/franka_ros2_ws/src/ci_mpc/ci_mpc_perception/config/gemini305_sim.yaml"
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
# hue ranges in OpenCV units (0-180)
HUES = {"red": [(0, 8), (172, 180)], "green": [(45, 85)], "blue": [(100, 130)], "yellow": [(20, 35)],
        "purple": [(135, 160)]}


class BoxDetector(Node):
    def __init__(self):
        super().__init__("box_detector")
        dp = self.declare_parameter
        dp("config", DEFAULT_CONFIG)
        dp("urdf_path", "/home/njohnson/ci_mpc_assets/fr3.urdf")
        dp("colors", ["box0:red", "box1:green", "box2:blue"])
        dp("min_pixels", 150)
        dp("top_band", 0.006)
        dp("table_z", 0.2)
        dp("min_saturation", 90)
        # joint states timed by their header stamp ("header", real robot) or by arrival on this node's clock
        # ("arrival", sim: Isaac's in-process controller manager stamps /joint_states ~0.24 s behind /clock)
        dp("joint_stamp", "arrival")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        cfg = yaml.safe_load(open(gp("config")))["camera"]
        self.T_hand_cam = optical_in_hand(cfg)
        self.colors = dict(kv.split(":") for kv in gp("colors"))
        self.min_pixels, self.top_band, self.table_z = gp("min_pixels"), gp("top_band"), gp("table_z")
        self.min_sat = gp("min_saturation")
        self.joint_stamp = gp("joint_stamp")
        full = pin.buildModelFromUrdf(gp("urdf_path"))
        lock = [j for j in range(1, full.njoints) if full.names[j] not in ARM]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.data = self.model.createData()
        self.hand = self.model.getFrameId(cfg["parent_frame"])
        self.js = collections.deque(maxlen=2000)  # (stamp, q)
        self.colors_buf = collections.OrderedDict()  # stamp -> rgb image
        self.info = None
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 100)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, 10)
        self.frames = 0
        self.create_timer(5.0, self.report)
        self.seen = collections.Counter()
        self.drops = collections.Counter()
        self.last_dt = None
        self.get_logger().info(f"boxes by color {self.colors}")

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def on_js(self, msg):
        if all(n in msg.name for n in ARM):
            t = self.stamp(msg) if self.joint_stamp == "header" else self.get_clock().now().nanoseconds * 1e-9
            self.js.append((t, np.array([msg.position[msg.name.index(n)] for n in ARM])))

    def on_info(self, msg):
        self.info = msg

    def on_color(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[..., :3]
        self.colors_buf[round(self.stamp(msg), 4)] = img
        while len(self.colors_buf) > 30:
            self.colors_buf.popitem(last=False)

    def camera_pose(self, t):
        if not self.js:
            return None
        ts = [s for s, _ in self.js]
        i = min(max(bisect.bisect_left(ts, t), 0), len(ts) - 1)
        if i > 0 and abs(ts[i - 1] - t) < abs(ts[i] - t):
            i -= 1
        if abs(ts[i] - t) > 0.05:
            return None
        pin.framesForwardKinematics(self.model, self.data, self.js[i][1])
        T = np.eye(4)
        T[:3, :3] = self.data.oMf[self.hand].rotation
        T[:3, 3] = self.data.oMf[self.hand].translation
        return T @ self.T_hand_cam

    def on_depth(self, msg):
        t = self.stamp(msg)
        rgb = self.colors_buf.get(round(t, 4))
        if rgb is None or self.info is None:
            self.drops["no color/info"] += 1
            return
        T = self.camera_pose(t)
        if T is None:
            self.drops["no joint state at stamp"] += 1
            if self.js:
                self.last_dt = t - self.js[-1][0]
            return
        z = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float64) / 1000.0
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        sat_ok = (hsv[..., 1] > self.min_sat) & (hsv[..., 2] > 40)
        out = Detection3DArray()
        out.header.frame_id = "base"
        out.header.stamp = msg.header.stamp
        for name, color in self.colors.items():
            mask = np.zeros(z.shape, bool)
            for lo, hi in HUES[color]:
                mask |= (hsv[..., 0] >= lo) & (hsv[..., 0] < hi)
            mask &= sat_ok
            mask = ndimage.binary_opening(mask, iterations=1)
            lab, n = ndimage.label(mask)
            if n == 0:
                continue
            sizes = ndimage.sum(mask, lab, range(1, n + 1))
            k = int(np.argmax(sizes)) + 1
            if sizes[k - 1] < self.min_pixels:
                continue
            v, u = np.nonzero((lab == k) & (z > 0))
            if len(u) < self.min_pixels // 2:
                continue
            d = z[v, u]
            pc = np.column_stack([(u - cx) * d / fx, (v - cy) * d / fy, d])
            pb = pc @ T[:3, :3].T + T[:3, 3]
            det = self.fit_box(pb)
            if det is None:
                continue
            center, yaw, size = det
            out.detections.append(self.detection(name, center, yaw, size, out.header))
            self.seen[name] += 1
        self.frames += 1
        self.pub.publish(out)

    def fit_box(self, pb):
        top = np.percentile(pb[:, 2], 98)
        face = pb[pb[:, 2] > top - self.top_band]
        if len(face) < 20:
            return None
        (cxm, cym), (w, h), ang = cv2.minAreaRect((face[:, :2] * 1000.0).astype(np.float32))
        yaw = (np.radians(ang) + np.pi / 4) % (np.pi / 2) - np.pi / 4  # box yaw mod 90 deg, in [-45, 45)
        k = int(round((np.radians(ang) - yaw) / (np.pi / 2)))
        sx, sy = (w, h) if k % 2 == 0 else (h, w)  # w lies along the rectangle angle
        low = np.percentile(pb[:, 2], 2)
        height = top - low
        if height < 0.5 * min(w, h) / 1000.0:  # sides barely visible: assume it stands on the table
            height = top - self.table_z
        center = np.array([cxm / 1000.0, cym / 1000.0, top - height / 2])
        return center, yaw, np.array([sx / 1000.0, sy / 1000.0, height])

    @staticmethod
    def detection(name, center, yaw, size, header):
        det = Detection3D()
        det.header = header
        det.id = name
        det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = map(float, center)
        det.bbox.center.orientation.z, det.bbox.center.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = map(float, size)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "box"
        hyp.hypothesis.score = 1.0
        det.results.append(hyp)
        return det

    def report(self):
        self.get_logger().info(f"frames {self.frames}, detections {dict(self.seen)}, dropped {dict(self.drops)}"
                               + (f", image - latest joint state stamp {self.last_dt:+.3f} s" if self.last_dt else ""))


def main():
    rclpy.init()
    node = BoxDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
