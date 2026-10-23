import numpy as np

from ci_mpc_planner.pusher_slider import (PusherSlider, PushingMpc, PushingMpcConfig, SliderParams,
                                          arc_reference, line_reference, wrap)


def closed_loop(s0, py0, task="line", truth=SliderParams(), T=10.0, sim_dt=0.005, period=0.05):
    model = PusherSlider(SliderParams())
    true = PusherSlider(truth)
    cfg = PushingMpcConfig()
    mpc = PushingMpc(model, cfg)
    s, py, t, t_solve, modes = np.array(s0, float), py0, 0.0, -1.0, {}
    while t < T:
        if t - t_solve >= period - 1e-9:
            if task == "line":
                s_ref, ds = line_reference(np.zeros(3), [1, 0], 0.03, 0.25, t, cfg.horizon, cfg.dt)
            else:
                s_ref, ds = arc_reference(np.zeros(3), 0.0, 0.25, 0.03, np.pi / 2, t, cfg.horizon, cfg.dt)
            u_ref = np.array([model.nominal_input(s_ref[k], ds[k], py) for k in range(cfg.horizon)])
            U, _, _ = mpc.solve(s, py, s_ref, u_ref)
            assert U is not None
            t_solve, err = t, s - s_ref[0]
        k = min(int((t - t_solve) / cfg.dt + 1e-9), cfg.horizon - 1)
        s, py, mode = true.step(s, py, U[k], sim_dt)
        modes[mode] = modes.get(mode, 0) + 1
        t += sim_dt
    return np.linalg.norm(err[:2]), abs(wrap(err[2])), modes


def test_motion_cone_symmetric_at_center():
    lo, hi = PusherSlider(SliderParams()).motion_cone(0.0)
    assert np.isclose(lo, -hi) and hi > 0


def test_sticking_inside_cone_moves_with_pusher():
    m = PusherSlider(SliderParams())
    s, py, mode = m.step(np.zeros(3), 0.0, np.array([0.03, 0.0]), 0.1)
    assert mode == "stick" and np.allclose(s, [0.003, 0, 0]) and py == 0.0


def test_line_converges_from_offset():
    e, eth, modes = closed_loop([0.0, 0.01, np.radians(10)], 0.0, T=12.0)
    assert e < 3e-3 and eth < np.radians(2)


def test_arc_tracks():
    py = PusherSlider(SliderParams()).contact_for_curvature(1 / 0.25)  # centered at p_y 0 the cone saturates
    assert -0.015 < py < -0.004
    e, _, modes = closed_loop([0.0, 0.0, 0.0], py, task="arc", T=12.0)
    assert e < 2e-3 and set(modes) == {"stick"}


def test_line_robust_to_model_error():
    e, _, _ = closed_loop([0.0, 0.01, np.radians(10)], 0.0, truth=SliderParams(c=0.06, mu_p=0.2), T=12.0)
    assert e < 2e-3
