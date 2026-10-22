"""Offline checks of the translational MPC on its own model (no robot, no ROS)."""
import numpy as np

from ci_mpc_planner.translational_mpc import MpcConfig, TranslationalMpc


def closed_loop(mpc, x0, p_goal, steps):
    """Apply the first planned acceleration each step on the exact double integrator."""
    dt = mpc.cfg.dt
    x = np.array(x0, dtype=float)
    traj = [x.copy()]
    for _ in range(steps):
        X, U, status = mpc.solve(x, np.tile(p_goal, (mpc.N + 1, 1)))
        assert X is not None, status
        a = U[0]
        x = np.concatenate([x[:3] + dt * x[3:] + 0.5 * dt ** 2 * a, x[3:] + dt * a])
        traj.append(x.copy())
    return np.array(traj)


def test_reach_converges_within_limits():
    cfg = MpcConfig()
    mpc = TranslationalMpc(cfg)
    p0 = np.array([0.307, 0.0, 0.487])
    goal = p0 + np.array([0.1, 0.1, -0.05])
    traj = closed_loop(mpc, np.concatenate([p0, np.zeros(3)]), goal, steps=150)  # 3 s
    assert np.linalg.norm(traj[-1, :3] - goal) < 1e-4
    assert np.abs(traj[:, 3:]).max() <= cfg.v_max + 1e-6
    # a straight-ish line: never further from the goal than at the start
    assert np.linalg.norm(traj[:, :3] - goal, axis=1).max() <= np.linalg.norm(p0 - goal) + 1e-6


def test_workspace_box_respected():
    cfg = MpcConfig(p_max=np.array([0.7, 0.4, 0.5]))
    mpc = TranslationalMpc(cfg)
    p0 = np.array([0.3, 0.0, 0.45])
    traj = closed_loop(mpc, np.concatenate([p0, np.zeros(3)]), np.array([0.3, 0.0, 0.7]), steps=100)
    assert traj[:, 2].max() <= 0.5 + 1e-4
    assert abs(traj[-1, 2] - 0.5) < 1e-3


def test_solve_time():
    import time
    mpc = TranslationalMpc(MpcConfig())
    x0 = np.array([0.3, 0, 0.45, 0, 0, 0])
    ts = []
    for k in range(50):
        t = time.perf_counter()
        mpc.solve(x0, np.tile([0.4, 0.1, 0.4], (mpc.N + 1, 1)))
        ts.append(time.perf_counter() - t)
    assert np.median(ts) < 0.01  # 10 ms at 50 Hz leaves headroom


def test_left_jacobian_matches_finite_difference():
    """mpc_node commands w = J_l(phi) dphi/dt for R = exp(phi) R0 (world-frame angular velocity)."""
    import pinocchio as pin
    from ci_mpc_planner.mpc_node import MpcNode
    rng = np.random.default_rng(0)
    R0 = pin.exp3(rng.normal(size=3))
    for _ in range(5):
        phi, dphi, h = rng.normal(size=3) * 0.6, rng.normal(size=3), 1e-6
        R1, R2 = pin.exp3(phi) @ R0, pin.exp3(phi + h * dphi) @ R0
        w_fd = pin.log3(R2 @ R1.T) / h  # world-frame angular velocity
        assert np.allclose(MpcNode.jl(phi) @ dphi, w_fd, atol=1e-5)


def test_feasible_start_recovers_from_outside_the_box():
    """A measured state outside the box (or too fast towards a wall) must still give a feasible problem."""
    cfg = MpcConfig()
    mpc = TranslationalMpc(cfg)
    x_bad = np.array([cfg.p_max[0] + 0.02, 0.0, 0.45, 0.3, 0.0, 0.0])  # 2 cm past the wall, moving out
    X, U, status = mpc.solve(x_bad, np.tile([0.9, 0.0, 0.45], (mpc.N + 1, 1)))
    assert X is None  # infeasible as given
    X, U, status = mpc.solve(mpc.feasible_start(x_bad), np.tile([0.9, 0.0, 0.45], (mpc.N + 1, 1)))
    assert X is not None, status
    assert X[:, 0].max() <= cfg.p_max[0] + 1e-4


def test_blended_path_corner_deviation_and_limits():
    from ci_mpc_planner.path import BlendedPath
    # home -> above pick -> grasp (90 deg corner at "above pick")
    pts = [[0.307, 0.0, 0.487], [0.45, -0.12, 0.33], [0.45, -0.12, 0.225]]
    path = BlendedPath(pts, np.zeros((3, 3)), v_max=0.25, a_max=0.4, blend=0.03)
    ts = np.linspace(0, path.duration, 2000)
    p, v, _ = path.sample(ts)
    assert np.linalg.norm(p[0] - pts[0]) < 1e-9 and np.linalg.norm(p[-1] - pts[-1]) < 1e-9
    assert np.linalg.norm(v, axis=1).max() <= 0.25 + 1e-6
    assert np.linalg.norm(v[0]) < 1e-9 and np.linalg.norm(v[-1]) < 1e-9
    # the corner is cut by at most blend * (1/cos(theta/2) - 1)-ish; at 90+ deg well below 3 cm
    corner_dist = np.linalg.norm(p - pts[1], axis=1).min()
    assert corner_dist < 0.03
    # the last part of the descent is vertical: below z 0.30 the path stays at x, y of the grasp
    low = p[:, 2] < 0.30
    assert np.abs(p[low, :2] - np.array(pts[2][:2])).max() < 1e-6
    # never longer than stopping at the corner
    stop = BlendedPath(pts[:2], np.zeros((2, 3)), 0.25, 0.4).duration + BlendedPath(pts[1:], np.zeros((2, 3)), 0.25, 0.4).duration
    assert path.duration < stop
