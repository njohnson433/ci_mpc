#!/usr/bin/env python3
# Offline closed loop of the stage-2 pushing MPC on the quasi-static pusher-slider (no Isaac): the "true"
# slider can have other c / mu_p than the MPC model, sticks or slides along the face (PusherSlider.step).
#   push_offline_sim.py [--task line|arc] [--c-true 0.04] [--mu-true 0.2] [--y0 0.01] [--th0-deg 10]
import argparse
import time

import numpy as np

from ci_mpc_planner.pusher_slider import (PusherSlider, PushingMpc, PushingMpcConfig, SliderParams,
                                          arc_reference, line_reference, wrap)

p = argparse.ArgumentParser()
p.add_argument("--task", default="line", choices=["line", "arc"])
p.add_argument("--c-true", type=float, default=0.03)
p.add_argument("--mu-true", type=float, default=0.3)
p.add_argument("--y0", type=float, default=0.0, help="initial lateral slider offset [m]")
p.add_argument("--th0-deg", type=float, default=0.0, help="initial slider yaw error [deg]")
p.add_argument("--py0", default="0", help="initial contact position on the face [m] or auto (arc: centered nominal)")
p.add_argument("--speed", type=float, default=0.03)
p.add_argument("--T", type=float, default=10.0)
p.add_argument("--sim-dt", type=float, default=0.005)
p.add_argument("--solve-period", type=float, default=0.05, help="replanning period [s] (the plan grid is cfg.dt)")
p.add_argument("--latency", type=float, default=0.0, help="measurement delay [s]")
p.add_argument("--noise", type=float, default=0.0, help="measurement noise std [m] (yaw: noise / 0.04 rad)")
p.add_argument("--seed", type=int, default=0)
p.add_argument("--set", nargs="*", default=[], help="MPC config overrides name=value")
args = p.parse_args()

model = PusherSlider(SliderParams())
truth = PusherSlider(SliderParams(c=args.c_true, mu_p=args.mu_true))
cfg = PushingMpcConfig()
for kv in args.set:
    k, v = kv.split("=")
    setattr(cfg, k, type(getattr(cfg, k))(float(v)))
mpc = PushingMpc(model, cfg)
start = np.array([0.0, 0.0, 0.0])
s = np.array([0.0, args.y0, np.radians(args.th0_deg)])
py = model.contact_for_curvature(1 / 0.25) if args.py0 == "auto" and args.task == "arc" else float(
    0 if args.py0 == "auto" else args.py0)
t, t_solve, U = 0.0, -1.0, None
modes, errs, ms = {}, [], []
rng = np.random.default_rng(args.seed)
hist = []  # (t, s) for the delayed measurement
while t < args.T:
    if t - t_solve >= args.solve_period - 1e-9:
        if args.task == "line":
            s_ref, ds_ref = line_reference(start, [1, 0], args.speed, 0.25, t, cfg.horizon, cfg.dt)
        else:
            s_ref, ds_ref = arc_reference(start, 0.0, 0.25, args.speed, np.pi / 2, t, cfg.horizon, cfg.dt)
        u_ref = np.array([model.nominal_input(s_ref[k], ds_ref[k], py) for k in range(cfg.horizon)])
        t0 = time.perf_counter()
        s_meas = next((h for th, h in reversed(hist) if th <= t - args.latency), s)
        s_meas = s_meas + rng.normal(0, args.noise, 3) * [1, 1, 1 / 0.04]
        if args.latency > 0 and U is not None:  # same forward prediction as push_node
            s_meas = s_meas + args.latency * model.dynamics(s_meas, U[0], py)
        U, S, status = mpc.solve(s_meas, py, s_ref, u_ref)
        ms.append((time.perf_counter() - t0) * 1000)
        if U is None:
            print(f"t {t:.2f}: solver {status}")
            U = np.zeros((cfg.horizon, 2))
        t_solve = t
        e = s - s_ref[0]
        errs.append([np.hypot(*e[:2]), abs(wrap(e[2]))])
    hist.append((t, s.copy()))
    s, py, mode = truth.step(s, py, U[min(int((t - t_solve) / cfg.dt + 1e-9), cfg.horizon - 1)], args.sim_dt)
    modes[mode] = modes.get(mode, 0) + 1
    t += args.sim_dt
errs = np.array(errs)
n = sum(modes.values())
print(f"task {args.task} c_true {args.c_true} mu_true {args.mu_true} y0 {args.y0} th0 {args.th0_deg} deg: "
      f"final pos err {errs[-1, 0] * 1000:.2f} mm, yaw {np.degrees(errs[-1, 1]):.2f} deg; "
      f"max after 3 s {errs[int(3 / args.solve_period):, 0].max() * 1000:.2f} mm; p_y {py * 1000:.1f} mm; "
      f"modes {', '.join(f'{k} {v / n:.0%}' for k, v in modes.items())}; solve {np.mean(ms):.1f} ms mean "
      f"{np.max(ms):.1f} max")
