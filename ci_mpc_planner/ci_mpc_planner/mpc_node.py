"""Stage-1 MPC node: plans the end-effector translation with TranslationalMpc and streams the plan to the
Cartesian impedance controller as ci_mpc_msgs/MpcCommand.

Each solve (solve_rate) starts from the previous plan's state at that time (replan_from=plan), so the
impedance controller's spring and damping keep pulling the arm onto the plan; only if the arm is more than
reset_threshold away from the plan (or replan_from=measured) it starts from the measured EE state (FK and
J*dq from /joint_states). Starting every solve from the measured state puts the setpoint where the arm
already is and switches the impedance feedback off (first sim test: 10-15 mm short, 44 mm off the line).
Between solves the plan is interpolated and sent at cmd_rate: pose and twist from the planned state,
wrench = Lambda_t(q) * a (feedforward for the planned acceleration; `armature` makes Lambda include the
rotor inertia). The orientation is held at the start orientation.

Tasks (parameter `task`): hold (stay at the start pose), reach (go to `goal`, also settable on ~/goal),
circle (radius / period in the y-z plane, starting at the current position, `circle_laps` laps, then hold).
Run with use_sim_time:=true in the simulator.
"""
import time

import numpy as np
import pinocchio as pin
import rclpy
from ci_mpc_msgs.msg import MpcCommand
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ci_mpc_planner.ee_model import ARM_JOINTS, EeModel
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
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        for name in ("dt", "q_pos", "q_vel", "r_acc", "r_dacc", "terminal", "v_max", "a_max", "horizon"):
            setattr(cfg, name, gp(name))
        cfg.p_min, cfg.p_max = np.array(gp("p_min")), np.array(gp("p_max"))
        self.cfg = cfg
        self.mpc = TranslationalMpc(cfg)
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
        self.plan = None  # (t0, X, U, lam_t)
        self.solve_ms, self.status = [], {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Point, "~/goal", self.on_goal, 10)
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

    def on_goal(self, msg):
        self.goal = np.array([msg.x, msg.y, msg.z])
        self.task = "reach"
        self.get_logger().info(f"new goal {np.round(self.goal, 4)}")

    def references(self, t_rel):
        """Position and velocity references for x_0..x_N, the first at t_rel after the task start."""
        N, dt = self.cfg.horizon, self.cfg.dt
        ts = t_rel + dt * np.arange(N + 1)
        p0 = self.start[0]
        if self.task == "reach":
            return np.tile(self.goal, (N + 1, 1)), np.zeros((N + 1, 3))
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

    def plan_state(self, t):
        """Interpolated (p, v, a) of the current plan at time t."""
        t0, X, U, _ = self.plan
        dt, N = self.cfg.dt, self.cfg.horizon
        s = max(0.0, t - t0)
        k = min(int(s / dt), N - 1)
        tau = min(s - k * dt, dt)
        a = U[k]
        return X[k, :3] + X[k, 3:] * tau + 0.5 * a * tau ** 2, X[k, 3:] + a * tau, a

    def on_solve(self):
        if self.q is None:
            return
        p, R, v, lam_t = self.model.update(self.q, self.dq)
        now = self.now_s()
        if self.start is None:
            self.start = (p.copy(), R.copy())
            self.t_task = now
            self.get_logger().info(f"start pose {np.round(p, 4)}")
        p_ref, v_ref = self.references(now - self.t_task)
        x0 = np.concatenate([p, v])
        if self.replan_from == "plan" and self.plan is not None:
            p_plan, v_plan, _ = self.plan_state(now)
            if np.linalg.norm(p_plan - p) < self.reset_threshold:
                x0 = np.concatenate([p_plan, v_plan])
            else:
                self.resets += 1
        t0 = time.perf_counter()
        X, U, status = self.mpc.solve(x0, p_ref, v_ref)
        self.solve_ms.append((time.perf_counter() - t0) * 1000)
        self.status[status] = self.status.get(status, 0) + 1
        if X is not None:
            self.plan = (now, X, U, lam_t)

    def on_command(self):
        if self.plan is None:
            return
        p, v, a = self.plan_state(self.now_s())
        F = self.plan[3] @ a
        m = MpcCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        quat = pin.Quaternion(self.start[1])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            quat.x, quat.y, quat.z, quat.w)
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v)
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, F)
        m.integral_enabled = bool(self.integral)
        self.pub.publish(m)

    def report(self):
        if self.solve_ms:
            ms = np.array(self.solve_ms)
            self.get_logger().info(f"solves {len(ms)}: {ms.mean():.2f} ms mean, {ms.max():.2f} ms max; "
                                   f"status {self.status}; plan resets to the measured state {self.resets}")
            self.solve_ms, self.status = [], {}


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
