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
