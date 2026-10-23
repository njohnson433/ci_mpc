"""Stage-1 MPC node: plans the end-effector translation with TranslationalMpc and streams the plan to the
Cartesian impedance controller as ci_mpc_msgs/MpcCommand.

Each solve (solve_rate) starts from the previous plan's state at that time (replan_from=plan), so the
impedance controller's spring and damping keep pulling the arm onto the plan; only if the arm is more than
reset_threshold away from the plan (or replan_from=measured) it starts from the measured EE state (FK and
J*dq from /joint_states). Starting every solve from the measured state puts the setpoint where the arm
already is and switches the impedance feedback off (first sim test: 10-15 mm short, 44 mm off the line).
Between solves the plan is interpolated and sent at cmd_rate: pose and twist from the planned state,
wrench = Lambda(q) [a; alpha] (feedforward for the planned accelerations; `armature` makes Lambda include
the rotor inertia).

Orientation: a second, independent QP of the same form plans the rotation vector phi (world frame,
R = exp(phi) R_start) with angular limits. The commanded angular velocity is J_l(phi) * dphi/dt (SO(3) left
Jacobian), the commanded orientation exp(phi) R_start. Valid for rotations within `rot_box` (default
1.2 rad) of the start orientation.

Tasks (parameter `task`): hold (stay at the start pose), reach (go to `goal` and `goal_rotvec` (rotation
vector relative to the start orientation, world frame); also settable on ~/goal (position) and ~/goal_pose),
circle (radius / period in the y-z plane, starting at the current position, `circle_laps` laps, then hold),
figure8 (same parameters: y = r sin(wt), z = r/2 sin(2wt) around the start position),
path (~/path, geometry_msgs/PoseArray: from the current plan pose through the poses, corners rounded with
`path_blend`, stopping at the last pose; timed with path_v_max / path_a_max, see path.BlendedPath).
~/active (std_msgs/Bool): false stops planning and publishing (another node commands the controller, e.g. a
contact phase); true resumes from the measured pose (holding it until a new goal arrives).
Run with use_sim_time:=true in the simulator.
"""
import time

import numpy as np
import pinocchio as pin
import rclpy
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ci_mpc_planner.ee_model import ARM_JOINTS, EeModel
from ci_mpc_planner.path import BlendedPath
from ci_mpc_planner.translational_mpc import MpcConfig, TranslationalMpc


class MpcNode(Node):
    def __init__(self):
        super().__init__("ci_mpc_planner")
        dp = self.declare_parameter
        dp("urdf_path", "/home/njohnson/ci_mpc_assets/fr3.urdf")
        dp("ee_frame", "fr3_hand_tcp")
        dp("command_topic", "/cartesian_impedance_controller/mpc_command")
        dp("task", "hold")
        dp("goal", [0.4, 0.1, 0.4])
        dp("goal_rotvec", [0.0, 0.0, 0.0])
        dp("circle_radius", 0.05)
        dp("circle_period", 5.0)
        dp("circle_laps", 2.0)
        dp("solve_rate", 50.0)
        dp("cmd_rate", 200.0)
        dp("integral_enabled", True)
        dp("replan_from", "plan")
        dp("reset_threshold", 0.02)
        dp("armature", [0.0])
        cfg = MpcConfig()
        for name in ("dt", "q_pos", "q_vel", "r_acc", "r_dacc", "terminal", "v_max", "a_max"):
            dp(name, float(getattr(cfg, name)))
        dp("horizon", cfg.horizon)
        dp("p_min", cfg.p_min.tolist())
        dp("p_max", cfg.p_max.tolist())
        # rotation QP (rotation vector relative to the start orientation)
        dp("w_max", 0.6)
        dp("alpha_max", 4.0)
        dp("rot_box", 1.2)
        dp("rot_reset_threshold", 0.05)
        # stop extending the motion when the arm gets close to a singularity (smallest singular value of J)
        dp("sigma_min_stop", 0.08)
        # acceleration limits from the wrench the controller can apply: a_lim = F_lim / max_i sum_j |Lambda_ij|
        # (the controller clamps at 18 N / 9 Nm / 2 Nm about z; the rest is headroom for its feedback).
        # 2 m/s^2 needs 47-80 N on the FR3 (Lambda_x 20-30 kg with armature): the arm overshot a wall by 22 mm.
        dp("force_limit", 12.0)
        dp("stiffness", [0.0])
        dp("path_v_max", 0.25)   # [m/s] along the path
        dp("path_a_max", 0.35)   # [m/s^2] tangential and centripetal; below the force-aware MPC limit (~0.5)
        dp("path_blend", 0.03)   # [m] corner radius at via points  # 6 values: commanded diagonal K (else the controller keeps its own)
        dp("torque_limit", [6.0, 6.0, 1.5])
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        for name in ("dt", "q_pos", "q_vel", "r_acc", "r_dacc", "terminal", "v_max", "a_max", "horizon"):
            setattr(cfg, name, gp(name))
        cfg.p_min, cfg.p_max = np.array(gp("p_min")), np.array(gp("p_max"))
        self.cfg = cfg
        self.mpc = TranslationalMpc(cfg)
        rb = gp("rot_box")
        self.cfg_rot = MpcConfig(dt=cfg.dt, horizon=cfg.horizon, q_pos=cfg.q_pos, q_vel=cfg.q_vel, r_acc=cfg.r_acc,
                                 r_dacc=cfg.r_dacc, terminal=cfg.terminal, v_max=gp("w_max"),
                                 a_max=gp("alpha_max"), p_min=np.full(3, -rb), p_max=np.full(3, rb))
        self.mpc_rot = TranslationalMpc(self.cfg_rot)
        self.rot_reset_threshold = gp("rot_reset_threshold")
        self.sigma_min_stop = gp("sigma_min_stop")
        self.force_limit = gp("force_limit")
        self.stiffness = [float(k) for k in gp("stiffness")] if len(gp("stiffness")) == 6 else None
        self.torque_limit = np.array(gp("torque_limit"), dtype=float)
        self.a_lims = []
        self.singular_stop = False
        self.goal_rotvec = np.array(gp("goal_rotvec"), dtype=float)
        armature = [a for a in gp("armature")] if len(gp("armature")) == 7 else None
        self.model = EeModel(gp("urdf_path"), gp("ee_frame"), armature)
        self.replan_from = gp("replan_from")
        self.reset_threshold = gp("reset_threshold")
        self.resets = 0
        self.task = gp("task")
        self.goal = np.array(gp("goal"), dtype=float)
        self.integral = gp("integral_enabled")

        self.q = None
        self.start = None  # (p, R) at the first state
        self.t_task = None
        self.plan = None  # (t0, X, U, Lambda 6x6, X_rot, U_rot)
        self.solve_ms, self.status = [], {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Point, "~/goal", self.on_goal, 10)
        self.create_subscription(Pose, "~/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(PoseArray, "~/path", self.on_path, 10)
        self.create_subscription(Bool, "~/active", self.on_active, 10)
        self.active = True
        self.path = None
        self.t_path = 0.0
        self.pub = self.create_publisher(MpcCommand, gp("command_topic"), 10)
        self.create_timer(1.0 / gp("solve_rate"), self.on_solve)
        self.create_timer(1.0 / gp("cmd_rate"), self.on_command)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"task {self.task}, dt {cfg.dt}, horizon {cfg.horizon} "
                               f"({cfg.dt * cfg.horizon:.2f} s), v_max {cfg.v_max}, a_max {cfg.a_max}, "
                               f"replan from {self.replan_from}, armature {armature}")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joint_states(self, msg):
        if not all(n in msg.name for n in ARM_JOINTS):
            return
        idx = [msg.name.index(n) for n in ARM_JOINTS]
        self.q = np.array([msg.position[i] for i in idx])
        self.dq = np.array([msg.velocity[i] for i in idx]) if msg.velocity else np.zeros(7)

    def on_active(self, msg):
        if msg.data == self.active:
            return
        self.active = msg.data
        if self.active and self.q is not None and self.start is not None:
            p, R, _, _ = self.model.update(self.q, self.dq)
            self.plan = None  # the next solve starts from the measured state
            self.task, self.goal, self.goal_rotvec = "reach", p.copy(), pin.log3(R @ self.start[1].T)
            self.singular_stop = False
        self.get_logger().info(f"{'active: holding ' + str(np.round(self.goal, 4)) if self.active else 'paused'}")

    def on_goal(self, msg):
        self.singular_stop = False
        self.goal = np.array([msg.x, msg.y, msg.z])
        self.task = "reach"
        self.get_logger().info(f"new goal {np.round(self.goal, 4)}")

    def on_goal_pose(self, msg):
        if self.start is None:
            self.get_logger().warn("goal_pose ignored: no robot state yet")
            return
        self.singular_stop = False
        self.goal = np.array([msg.position.x, msg.position.y, msg.position.z])
        o = msg.orientation
        R_goal = pin.Quaternion(o.w, o.x, o.y, o.z).normalized().toRotationMatrix()
        self.goal_rotvec = pin.log3(R_goal @ self.start[1].T)
        self.task = "reach"
        self.get_logger().info(f"new goal pose {np.round(self.goal, 4)}, rotation {np.round(self.goal_rotvec, 3)} rad")

    def on_path(self, msg):
        if self.start is None or not msg.poses:
            self.get_logger().warn("path ignored: no robot state yet or empty")
            return
        now = self.now_s()
        if self.plan is not None:
            p0, rv0 = self.plan_state(now)[0], self.plan_state(now, rot=True)[0]
        else:
            p0, rv0 = self.start[0], np.zeros(3)
        pts, rvs = [p0], [rv0]
        for pose in msg.poses:
            pts.append([pose.position.x, pose.position.y, pose.position.z])
            o = pose.orientation
            R = pin.Quaternion(o.w, o.x, o.y, o.z).normalized().toRotationMatrix()
            rvs.append(pin.log3(R @ self.start[1].T))
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.path = BlendedPath(pts, rvs, gp("path_v_max"), gp("path_a_max"), gp("path_blend"))
        self.t_path = now
        self.goal, self.goal_rotvec = np.array(pts[-1]), np.array(rvs[-1])  # held after the path ends
        self.singular_stop = False
        self.task = "path"
        self.get_logger().info(f"path with {len(msg.poses)} poses, {self.path.s[-1]:.3f} m, {self.path.duration:.2f} s")

    def path_references(self, now):
        N, dt = self.cfg.horizon, self.cfg.dt
        ts = (now - self.t_path) + dt * np.arange(N + 1)
        p, v, rv = self.path.sample(ts)
        drv = np.gradient(rv, dt, axis=0) * (ts < self.path.duration)[:, None]
        return p, v, rv, drv

    def rot_references(self):
        N = self.cfg.horizon
        if self.task == "path" and self.path is not None:
            _, _, rv, drv = self.path_references(self.now_s())
            return rv, drv
        target = self.goal_rotvec if self.task == "reach" else np.zeros(3)
        return np.tile(target, (N + 1, 1)), np.zeros((N + 1, 3))

    def references(self, t_rel):
        """Position and velocity references for x_0..x_N, the first at t_rel after the task start."""
        N, dt = self.cfg.horizon, self.cfg.dt
        ts = t_rel + dt * np.arange(N + 1)
        p0 = self.start[0]
        if self.task == "reach":
            return np.tile(self.goal, (N + 1, 1)), np.zeros((N + 1, 3))
        if self.task == "path" and self.path is not None:
            p, v, _, _ = self.path_references(self.now_s())
            return p, v
        if self.task == "figure8":
            r = self.get_parameter("circle_radius").value
            T = self.get_parameter("circle_period").value
            laps = self.get_parameter("circle_laps").value
            w = 2 * np.pi / T
            ts = np.minimum(ts, laps * T)
            moving = (ts < laps * T)[:, None]
            zero = np.zeros_like(ts)
            p = p0 + np.stack([zero, r * np.sin(w * ts), 0.5 * r * np.sin(2 * w * ts)], axis=1)
            v = np.stack([zero, r * w * np.cos(w * ts), r * w * np.cos(2 * w * ts)], axis=1) * moving
            return p, v
        if self.task == "circle":
            r = self.get_parameter("circle_radius").value
            T = self.get_parameter("circle_period").value
            laps = self.get_parameter("circle_laps").value
            w = 2 * np.pi / T
            ts = np.minimum(ts, laps * T)  # hold at the end of the last lap
            moving = (ts < laps * T)[:, None]
            center = p0 - np.array([0.0, r, 0.0])
            p = center + r * np.stack([np.zeros_like(ts), np.cos(w * ts), np.sin(w * ts)], axis=1)
            v = r * w * np.stack([np.zeros_like(ts), -np.sin(w * ts), np.cos(w * ts)], axis=1) * moving
            return p, v
        return np.tile(p0, (N + 1, 1)), np.zeros((N + 1, 3))

    @staticmethod
    def jl(phi):
        """SO(3) left Jacobian: world angular velocity w = J_l(phi) dphi/dt for R = exp(phi) R0."""
        return pin.Jexp3(phi).T

    def plan_state(self, t, rot=False):
        """Interpolated (p, v, a) of the current plan at time t (rotation vector plan if rot)."""
        t0, X, U = self.plan[0], self.plan[1], self.plan[2]
        if rot:
            X, U = self.plan[4], self.plan[5]
        dt, N = self.cfg.dt, self.cfg.horizon
        s = max(0.0, t - t0)
        k = min(int(s / dt), N - 1)
        tau = min(s - k * dt, dt)
        a = U[k]
        return X[k, :3] + X[k, 3:] * tau + 0.5 * a * tau ** 2, X[k, 3:] + a * tau, a

    def on_solve(self):
        if self.q is None or not self.active:
            return
        p, R, twist, lam = self.model.update(self.q, self.dq)
        v, w = twist[:3], twist[3:]
        sigma = self.model.sigma_min
        if sigma < self.sigma_min_stop and not self.singular_stop:
            # hold where the plan is now: the impedance controller loses authority near a singularity
            self.singular_stop = True
            self.task = "reach"
            self.goal = self.plan_state(self.now_s())[0] if self.plan is not None else p.copy()
            self.goal_rotvec = self.plan_state(self.now_s(), rot=True)[0] if self.plan is not None else np.zeros(3)
            self.get_logger().error(f"close to a singularity (sigma_min {sigma:.3f} < {self.sigma_min_stop}): "
                                    f"holding at {np.round(self.goal, 4)}; send a new goal away from it")
        now = self.now_s()
        if self.start is None:
            self.start = (p.copy(), R.copy())
            self.t_task = now
            self.get_logger().info(f"start pose {np.round(p, 4)}")
        p_ref, v_ref = self.references(now - self.t_task)
        lt, lr = np.abs(lam[:3, :3]).sum(1), np.abs(lam[3:, 3:]).sum(1)
        a_lim = self.force_limit / lt.max()
        alpha_lim = (self.torque_limit / lr).min()
        self.a_lims.append((a_lim, alpha_lim))
        x0 = self.mpc.feasible_start(np.concatenate([p, v]), a_lim)
        if self.replan_from == "plan" and self.plan is not None:
            p_plan, v_plan, _ = self.plan_state(now)
            if np.linalg.norm(p_plan - p) < self.reset_threshold:
                x0 = np.concatenate([p_plan, v_plan])
            else:
                self.resets += 1
        t0 = time.perf_counter()
        X, U, status = self.mpc.solve(x0, p_ref, v_ref, a_max=a_lim)
        # rotation: phi = log(R R_start^T); dphi/dt = J_l(phi)^-1 w
        phi = pin.log3(R @ self.start[1].T)
        r0 = self.mpc_rot.feasible_start(np.concatenate([phi, np.linalg.solve(self.jl(phi), w)]), alpha_lim)
        if self.replan_from == "plan" and self.plan is not None:
            phi_plan, dphi_plan, _ = self.plan_state(now, rot=True)
            if np.linalg.norm(phi_plan - phi) < self.rot_reset_threshold:
                r0 = np.concatenate([phi_plan, dphi_plan])
            else:
                self.resets += 1
        Xr, Ur, status_r = self.mpc_rot.solve(r0, *self.rot_references(), a_max=alpha_lim)
        self.solve_ms.append((time.perf_counter() - t0) * 1000)
        for st in (status, status_r):
            self.status[st] = self.status.get(st, 0) + 1
        if X is not None and Xr is not None:
            self.plan = (now, X, U, lam, Xr, Ur)

    def on_command(self):
        if self.plan is None or not self.active:
            return
        p, v, a = self.plan_state(self.now_s())
        phi, dphi, ddphi = self.plan_state(self.now_s(), rot=True)
        Jl = self.jl(phi)
        w, alpha = Jl @ dphi, Jl @ ddphi  # (the dJ_l/dt term is neglected in the feedforward)
        F = self.plan[3] @ np.concatenate([a, alpha])
        m = MpcCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        quat = pin.Quaternion(pin.exp3(phi) @ self.start[1])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            quat.x, quat.y, quat.z, quat.w)
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v)
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = map(float, w)
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, F[:3])
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = map(float, F[3:])
        m.integral_enabled = bool(self.integral)
        if self.stiffness is not None:
            m.stiffness = self.stiffness
        self.pub.publish(m)

    def report(self):
        if self.solve_ms:
            ms = np.array(self.solve_ms)
            self.get_logger().info(f"solves {len(ms)}: {ms.mean():.2f} ms mean, {ms.max():.2f} ms max; "
                                   f"status {self.status}; plan resets to the measured state {self.resets}; "
                                   f"a_lim {np.min([a for a, _ in self.a_lims]):.2f}-{np.max([a for a, _ in self.a_lims]):.2f} m/s^2, "
                                   f"alpha_lim {np.min([b for _, b in self.a_lims]):.2f}-{np.max([b for _, b in self.a_lims]):.2f} rad/s^2")
            self.solve_ms, self.status, self.a_lims = [], {}, []


def main():
    rclpy.init()
    node = MpcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise  # only swallow errors from a message take interrupted by the shutdown signal
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
