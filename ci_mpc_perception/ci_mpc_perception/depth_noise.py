"""Stereo depth noise for the simulated Gemini 305: turns Isaac's noise-free depth (/sim/camera/depth, 32FC1 [m])
into what the Orbbec driver publishes (/camera/depth/image_raw, 16UC1 [mm], depth aligned to color) plus
/camera/depth/camera_info (copied from the color camera, same pose and intrinsics).

Model (config depth_noise in gemini305_sim.yaml):
  - stereo depth error sigma_z = z^2 sigma_d / (f b), sigma_d = disparity_std_px scaled from 1280 px width to the
    image width (Gemini 305: b = 18 mm: 0.5 mm at 30 cm at full resolution), correlated over small patches
    (stereo matching works on windows), quantized to 1 mm;
  - invalid (0) pixels: outside [min_range, max_range], within edge_holes_px of a depth jump > edge_jump
    (occlusion / matching failure at object edges), and a random fraction `dropout`;
  - latency: each image is published `latency` seconds (sim time) after its stamp, keeping the stamp.
"""
import collections

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from scipy.ndimage import binary_dilation, uniform_filter, zoom
from sensor_msgs.msg import CameraInfo, Image

DEFAULT_CONFIG = "/home/njohnson/franka_ros2_ws/src/ci_mpc/ci_mpc_perception/config/gemini305_sim.yaml"


class DepthNoise(Node):
    def __init__(self):
        super().__init__("depth_noise")
        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("seed", 0)
        cfg = yaml.safe_load(open(self.get_parameter("config").value))
        self.n = cfg["depth_noise"]
        self.rng = np.random.default_rng(self.get_parameter("seed").value)
        self.info = None
        self.queue = collections.deque()
        self.pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self.create_subscription(Image, "/sim/camera/depth", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, 10)
        self.create_timer(0.005, self.flush)
        self.count = 0
        self.get_logger().info(f"depth noise: {self.n}")

    def on_info(self, msg):
        self.info = msg

    def stamp_s(self, msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def on_depth(self, msg):
        if self.info is None:
            return
        z = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).astype(np.float64)
        n = self.n
        valid = np.isfinite(z) & (z > n["min_range"]) & (z < n["max_range"])
        z = np.where(valid, z, 0.0)
        f = self.info.k[0]
        sigma_d = n["disparity_std_px"] * msg.width / 1280.0
        # correlated noise: white noise smoothed over 3x3 (std restored), coarse 1/8-resolution component
        w = uniform_filter(self.rng.standard_normal(z.shape), 3) * 3.0
        h, wd = z.shape
        coarse = zoom(self.rng.standard_normal((h // 8 + 1, wd // 8 + 1)), 8, order=1)[:h, :wd]
        noise = (0.7 * w + 0.7 * coarse) * (z ** 2) * sigma_d / (f * n["baseline"])
        zn = z + noise
        # holes at depth discontinuities and random dropouts
        gy, gx = np.gradient(z)
        edge = (np.abs(gx) > n["edge_jump"]) | (np.abs(gy) > n["edge_jump"])
        if n["edge_holes_px"] > 0:
            edge = binary_dilation(edge, iterations=int(n["edge_holes_px"]))
        drop = self.rng.random(z.shape) < n["dropout"]
        zn[~valid | edge | drop] = 0.0
        mm = np.clip(np.round(zn * 1000.0), 0, 65535).astype(np.uint16)
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = mm.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = self.info.height, self.info.width
        info.distortion_model, info.d = self.info.distortion_model, self.info.d
        info.k, info.r, info.p = self.info.k, self.info.r, self.info.p
        self.queue.append((self.stamp_s(msg) + n["latency"], out, info))

    def flush(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.queue and self.queue[0][0] <= now:
            _, img, info = self.queue.popleft()
            self.pub.publish(img)
            self.pub_info.publish(info)
            self.count += 1


def main():
    rclpy.init()
    node = DepthNoise()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
