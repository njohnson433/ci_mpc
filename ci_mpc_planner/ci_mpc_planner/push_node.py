"""Stage-2 pushing node: pushes a box along a planned slider path with a sticking point contact
(pusher_slider.PushingMpc) and streams the pusher motion to the Cartesian impedance controller as MpcCommand.

Phases (sim time):
  approach  min-jerk TCP moves (free space, integral on): above the pre-contact point -> down to the push height,
            `pre_contact` behind the face center (contact at p_y = 0, the face whose normal is closest to the
            push direction).
  contact   creep along the face normal at `contact_speed` until the rod is `preload` into the face
            (measured from the detection) or the slider moved `contact_detect` (integral off from here on).
  push      MPC at solve_rate on the plan grid dt; the command at cmd_rate follows the plan: TCP xy = planned
            slider pose + R(theta) [-a/2 - rod_radius + preload, p_y], twist = R(theta) u, feedforward =
            quasi-static contact force (scaled by `feedforward`). p_y and the slider pose are measured each solve.
  retreat   back `pre_contact` along the last push direction, then up.
Reference (task): line (start = the slider at contact, direction `direction` (world xy), `length`, `speed`) or
arc (`radius`, `angle` [rad, + = left], then straight to `length` total).
Slider pose: vision_msgs/Detection3DArray on /perception/detections (first detection). The square box's yaw is
taken modulo 90 deg, closest to the push direction.
The log (log_path, npz) has t, measured slider state, reference, p_y, contact gap, planned input, phase.
"""
import time

import numpy as np
import pinocchio as pin
import rclpy
from ci_mpc_msgs.msg import MpcCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection3DArray

from ci_mpc_planner.ee_model import ARM_JOINTS, EeModel
from ci_mpc_planner.pusher_slider import (PusherSlider, PushingMpc, PushingMpcConfig, SliderParams,
                                          arc_reference, line_reference, rot, wrap)


def min_jerk(p0, p1, T, t):
    s = np.clip(t / T, 0.0, 1.0)
    h = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    dh = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T if t < T else 0.0
    return p0 + (p1 - p0) * h, (p1 - p0) * dh


class PushNode(Node):
    def __init__(self):
        super().__init__("ci_mpc_push")
        dp = self.declare_parameter
        dp("urdf_path", "/home/njohnson/ci_mpc_assets/fr3.urdf")
        dp("ee_frame", "fr3_hand_tcp")
        dp("command_topic", "/cartesian_impedance_controller/mpc_command")
        dp("task", "line")
        dp("direction", [0.0, 1.0])
        dp("speed", 0.03)
        dp("length", 0.2)
        dp("radius", 0.15)
        dp("angle", 1.5708)
        dp("push_z", 0.2946)        # TCP z while pushing (rod tip 8 mm above a table at z 0.2)
        dp("safe_dz", 0.07)
        dp("rod_radius", 0.006)
        dp("pre_contact", 0.03)
        # contact position on the face [m] (slider frame y), or "auto": the one that centers the nominal input in
        # the motion cone for the path curvature (line: 0, arc: PusherSlider.contact_for_curvature)
        dp("contact_py", "auto")
        dp("contact_speed", 0.01)
        dp("contact_detect", 0.001)
        dp("preload", 0.0005)
        dp("approach_speed", 0.06)
        dp("feedforward", 1.0)
        dp("solve_rate", 20.0)
        dp("cmd_rate", 200.0)
        dp("stiffness", [0.0])
        dp("lost_contact_gap", 0.005)
        # detection latency [s]: the measured slider pose is predicted forward by this much with the current plan
        # (sim: ~30 ms: object.json poll + detector rate + DDS; 0: slider 1 mm behind, 1.8 mm overshoot at the stop)
        dp("latency", 0.03)
        dp("log_path", "")
        sp_ = SliderParams()
        for name in ("a", "c", "mu_p", "f_max"):
            dp("slider_" + name, float(getattr(sp_, name)))
        cfg = PushingMpcConfig()
        for name in ("dt", "q_pos", "q_theta", "terminal", "r_u", "r_du", "v_n_min", "v_n_max", "v_t_max",
                     "cone_margin"):
            dp(name, float(getattr(cfg, name)))
        dp("horizon", cfg.horizon)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gp = gp
        for name in ("dt", "q_pos", "q_theta", "terminal", "r_u", "r_du", "v_n_min", "v_n_max", "v_t_max",
                     "cone_margin", "horizon"):
            setattr(cfg, name, gp(name))
        self.cfg = cfg
        self.sp = SliderParams(**{n: gp("slider_" + n) for n in ("a", "c", "mu_p", "f_max")})
        self.slider = PusherSlider(self.sp)
        self.mpc = PushingMpc(self.slider, cfg)
        self.model = EeModel(gp("urdf_path"), gp("ee_frame"))
        self.stiffness = [float(k) for k in gp("stiffness")] if len(gp("stiffness")) == 6 else None
        self.q = self.det = None
        self.phase, self.t_phase = "wait", None
        self.segments = []
        self.R0 = None
        self.cmd = None  # (p, v, F, integral)
        self.plan = None  # (t0, S, U, py)
        self.log = {k: [] for k in ("t", "s", "s_ref", "py", "gap", "u", "phase", "tcp", "solve_ms")}
        self.lost = 0
        self.theta_last = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Detection3DArray, "/perception/detections", self.on_det, 10)
        self.pub = self.create_publisher(MpcCommand, gp("command_topic"), 10)
        self.create_timer(1.0 / gp("solve_rate"), self.on_solve)
        self.create_timer(1.0 / gp("cmd_rate"), self.on_command)
        self.get_logger().info(f"task {gp('task')}, speed {gp('speed')}, length {gp('length')}, slider {self.sp}, "
                               f"mpc dt {cfg.dt} N {cfg.horizon}")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_js(self, msg):
        if all(n in msg.name for n in ARM_JOINTS):
            idx = [msg.name.index(n) for n in ARM_JOINTS]
            self.q = np.array([msg.position[i] for i in idx])
            self.dq = np.array([msg.velocity[i] for i in idx]) if msg.velocity else np.zeros(7)

    def on_det(self, msg):
        if msg.detections:
            self.det = msg.detections[0]

    def direction(self):
        d = np.array(self.gp("direction"), dtype=float)
        return d / np.linalg.norm(d)

    def slider_state(self, heading):
        """Measured [x, y, theta]: box yaw modulo 90 deg, the multiple closest to `heading`."""
        c = self.det.bbox.center
        w, x, y, z = c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        k = np.round((heading - yaw) / (np.pi / 2))
        return np.array([c.position.x, c.position.y, wrap(yaw + k * np.pi / 2)])

    def contact(self, s, tcp_xy):
        """(p_y, gap): rod axis in the slider frame -> contact position on the face and gap to the face."""
        r = rot(s[2]).T @ (tcp_xy - s[:2])
        return r[1], -r[0] - self.sp.a / 2 - self.gp("rod_radius")

    def tcp(self):
        p, R, twist, _ = self.model.update(self.q, self.dq)
        return p, R

    def start_segments(self, points, now):
        self.segments, t0 = [], now
        for a, b in zip(points[:-1], points[1:]):
            T = max(0.5, 1.875 * np.linalg.norm(b - a) / self.gp("approach_speed"))  # peak speed = approach_speed
            self.segments.append((t0, T, a, b))
            t0 += T

    def segment_ref(self, now):
        for t0, T, a, b in self.segments:
            if now < t0 + T:
                return (*min_jerk(a, b, T, now - t0), False)
        t0, T, a, b = self.segments[-1]
        return b, np.zeros(3), True

    def reference(self, t):
        gp, cfg = self.gp, self.cfg
        if gp("task") == "arc":
            s_ref, ds = arc_reference(self.s_start, self.s_start[2], gp("radius"), gp("speed"), gp("angle"), t,
                                      cfg.horizon, cfg.dt)
            L = gp("length")
            stop = gp("speed") * (t + cfg.dt * np.arange(cfg.horizon + 1)) >= L
            if stop.any():  # hold the end of the path
                s_end, _ = arc_reference(self.s_start, self.s_start[2], gp("radius"), gp("speed"), gp("angle"),
                                         L / gp("speed"), 0, cfg.dt)
                s_ref[stop], ds[stop] = s_end[0], 0.0
            return s_ref, ds
        return line_reference(self.s_start, self.direction(), gp("speed"), gp("length"), t, cfg.horizon, cfg.dt)

    def on_solve(self):
        if self.q is None or self.det is None:
            return
        now = self.now_s()
        p, R = self.tcp()
        gp = self.gp
        heading = np.arctan2(*self.direction()[::-1])
        if self.phase == "wait":
            self.R0 = R.copy()
            s = self.slider_state(heading)
            n = np.array([np.cos(s[2]), np.sin(s[2])])
            tan = np.array([-n[1], n[0]])
            if gp("contact_py") == "auto":
                kappa = np.sign(gp("angle")) / gp("radius") if gp("task") == "arc" else 0.0
                py_star = self.slider.contact_for_curvature(kappa)
            else:
                py_star = float(gp("contact_py"))
            pre = s[:2] - n * (self.sp.a / 2 + gp("rod_radius") + gp("pre_contact")) + tan * py_star
            self.get_logger().info(f"contact planned at p_y {py_star * 1000:.1f} mm")
            z = gp("push_z")
            self.start_segments([p, np.array([*pre, z + gp("safe_dz")]), np.array([*pre, z])], now)
            self.phase, self.t_phase = "approach", now
            self.get_logger().info(f"slider {np.round(s, 4)}; approach to {np.round(pre, 4)} at z {z}")
            return
        # yaw continuity: closest to the last measured heading once pushing (the box turns on arcs)
        s = self.slider_state(self.theta_last if self.phase in ("push", "retreat") else heading)
        self.theta_last = s[2]
        py, gap = self.contact(s, p[:2])
        if self.phase == "approach":
            pos, vel, done = self.segment_ref(now)
            self.cmd = (pos, vel, np.zeros(3), True)
            if done and now - self.segments[-1][0] - self.segments[-1][1] > 0.5:
                self.phase, self.t_phase = "contact", now
                self.s_contact0 = s.copy()
                self.p_contact0 = pos.copy()
                self.get_logger().info(f"contact phase: gap {gap * 1000:.1f} mm, p_y {py * 1000:.1f} mm")
        elif self.phase == "contact":
            n = np.array([np.cos(s[2]), np.sin(s[2])])
            tc = now - self.t_phase
            pos = self.p_contact0 + np.concatenate([n * gp("contact_speed") * tc, [0.0]])
            self.cmd = (pos, np.concatenate([n * gp("contact_speed"), [0.0]]), np.zeros(3), False)
            moved = np.linalg.norm(s[:2] - self.s_contact0[:2])
            if gap < -gp("preload") or moved > gp("contact_detect") or tc > 10.0:
                self.s_start = s.copy()
                self.phase, self.t_phase = "push", now
                self.get_logger().info(f"in contact after {tc:.2f} s: gap {gap * 1000:.2f} mm, slider moved "
                                       f"{moved * 1000:.2f} mm, p_y {py * 1000:.1f} mm; start {np.round(s, 4)}")
        elif self.phase == "push":
            t = now - self.t_phase
            s_ref, ds_ref = self.reference(t)
            py_m = float(np.clip(py, -self.sp.a / 2 + 0.01, self.sp.a / 2 - 0.01))
            if self.plan is not None and gp("latency") > 0:
                s = s + gp("latency") * self.slider.dynamics(s, self.plan_input(now), py_m)
            u_ref = np.array([self.slider.nominal_input(s_ref[k], ds_ref[k], py_m) for k in range(self.cfg.horizon)])
            t0 = time.perf_counter()
            U, S, status = self.mpc.solve(s, py_m, s_ref, u_ref)
            ms = (time.perf_counter() - t0) * 1000
            if U is None:
                self.get_logger().warn(f"MPC {status}: keeping the previous plan")
            else:
                self.plan = (now, S, U, py_m)
            if gap > gp("lost_contact_gap"):
                self.lost += 1
            self.record(now, s, s_ref[0], py, gap, U[0] if U is not None else np.full(2, np.nan), p, ms)
            path_done = gp("speed") * t >= gp("length") + 0.5 * gp("speed")
            if path_done:
                n = np.array([np.cos(s[2]), np.sin(s[2])])
                back = p.copy()
                back[:2] -= n * gp("pre_contact")
                self.start_segments([p, back, back + np.array([0, 0, gp("safe_dz")])], now)
                self.phase, self.t_phase = "retreat", now
                e = s - s_ref[0]
                self.get_logger().info(f"PUSH DONE: slider {np.round(s, 4)}, target {np.round(s_ref[0], 4)}, "
                                       f"error {np.linalg.norm(e[:2]) * 1000:.2f} mm, {np.degrees(wrap(e[2])):.2f} deg; "
                                       f"lost-contact solves {self.lost}")
                self.save()
        elif self.phase == "retreat":
            pos, vel, done = self.segment_ref(now)
            self.cmd = (pos, vel, np.zeros(3), True)
            if done and self.phase != "finished":
                self.phase = "finished"
                self.get_logger().info("FINISHED")
        if self.phase in ("approach", "contact", "retreat"):
            self.record(now, s, np.full(3, np.nan), py, gap, np.full(2, np.nan), p, 0.0)

    def record(self, now, s, s_ref, py, gap, u, tcp, ms):
        for k, v in zip(self.log, (now, s, s_ref, py, gap, u, self.phase, tcp, ms)):
            self.log[k].append(v)

    def save(self):
        path = self.gp("log_path")
        if path:
            np.savez(path, **{k: np.array(v) for k, v in self.log.items()})

    def plan_input(self, now):
        t0, _, U, _ = self.plan
        k = min(int(max(0.0, now - t0) / self.cfg.dt), self.cfg.horizon - 1)
        return U[k]

    def push_command(self, now):
        t0, S, U, py = self.plan
        dt, N = self.cfg.dt, self.cfg.horizon
        tau = min(max(0.0, now - t0), dt * N - 1e-9)
        k = int(tau / dt)
        a = (tau - k * dt) / dt
        s = S[k] + a * (S[k + 1] - S[k])
        s[2] = S[k, 2] + a * wrap(S[k + 1, 2] - S[k, 2])
        u = U[k]
        Rs = rot(s[2])
        r = np.array([-self.sp.a / 2 - self.gp("rod_radius") + self.gp("preload"), py])
        xy = s[:2] + Rs @ r
        V = self.slider.M(py) @ u
        f = self.gp("feedforward") * (Rs @ self.slider.force(V, py))
        return (np.array([*xy, self.gp("push_z")]), np.concatenate([Rs @ u, [0.0]]), np.array([*f, 0.0]), False)

    def on_command(self):
        now = self.now_s()
        if self.phase == "push" and self.plan is not None:
            self.cmd = self.push_command(now)
        elif self.phase in ("approach", "retreat", "finished") and self.segments:
            pos, vel, _ = self.segment_ref(now)
            self.cmd = (pos, vel, np.zeros(3), True)
        if self.cmd is None or self.R0 is None:
            return
        pos, vel, F, integral = self.cmd
        m = MpcCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        q = pin.Quaternion(self.R0)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q.x, q.y, q.z, q.w
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, vel)
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, F)
        m.integral_enabled = bool(integral)
        if self.stiffness is not None:
            m.stiffness = self.stiffness
        self.pub.publish(m)


def main():
    rclpy.init()
    node = PushNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()
