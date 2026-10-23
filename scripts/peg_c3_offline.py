#!/usr/bin/env python3
# Offline closed loop of the planar peg-in-hole CI-MPC (C3+) on the quasi-static contact plant (no Isaac).
# The plant's hole is at --hole-x (true); the MPC's model at --est-x (perception estimate).
#   peg_c3_offline.py [--hole-x 0.002] [--est-x 0.0] [--policy c3|down] [--steps 300] [--kx 1500 --kz 500 --kphi 130]
import argparse
import time

import numpy as np

from ci_mpc_planner.c3 import C3Config, PegHoleC3
from ci_mpc_planner.hole_estimator import HoleEstimator
from ci_mpc_planner.peg_hole import PegHole

p = argparse.ArgumentParser()
p.add_argument("--hole-x", type=float, default=0.0)
p.add_argument("--est-x", type=float, default=0.0)
p.add_argument("--start-x", type=float, default=None, help="TCP start x (default: over the estimate)")
p.add_argument("--policy", default="c3", choices=["c3", "down"])
p.add_argument("--steps", type=int, default=300)
p.add_argument("--kx", type=float, default=1500.0)
p.add_argument("--kz", type=float, default=500.0)
p.add_argument("--kphi", type=float, default=130.0)
p.add_argument("--set", nargs="*", default=[], help="C3Config overrides name=value")
p.add_argument("--verbose", action="store_true")
p.add_argument("--estimate", action="store_true", help="localize the hole from contact (particle filter)")
p.add_argument("--wrench-noise", type=float, default=0.3, help="std of the measured force [N] (torque: /60 N m)")
args = p.parse_args()

compliance = np.array([1 / args.kx, 1 / args.kz, 1 / args.kphi])
plant = PegHole(hole_x=args.hole_x)
model = PegHole(hole_x=args.est_x)
cfg = C3Config()
for kv in args.set:
    k, v = kv.split("=")
    cur = getattr(cfg, k)
    setattr(cfg, k, np.array([float(t) for t in v.split(",")]) if isinstance(cur, np.ndarray) else type(cur)(float(v)))
mpc = PegHoleC3(model, compliance, cfg)
tip_below = plant.g.tip_below_tcp
x = np.array([args.est_x if args.start_x is None else args.start_x, tip_below + 0.002, 0.0])  # tip 2 mm above the opening, over the estimate
u = x.copy()
goal = np.array([args.est_x, tip_below - plant.g.depth, 0.0])
ms, viol = [], []
rng = np.random.default_rng(1)
est = HoleEstimator(plant.g, compliance, args.est_x) if args.estimate else None
for k in range(args.steps):
    if args.policy == "c3":
        t0 = time.perf_counter()
        u, info = mpc.solve(x, goal, u)
        ms.append((time.perf_counter() - t0) * 1000)
        viol.append(info.get("viol", 0))
    else:  # straight down at the (estimated) hole x, pressing at most 6 mm into the spring
        u = u.copy()
        u[0] += np.clip(goal[0] - u[0], -0.0005, 0.0005)
        u[1] = max(u[1] - 0.0005, goal[1] - 0.006)
    x_prev = x
    x, lam, nc = plant.step(x, u, compliance)
    tip = plant.tip(x)
    if est is not None:
        w_meas = (plant.contact_wrench(x_prev, lam, compliance) if nc else np.zeros(3)) + rng.normal(
            0, args.wrench_noise, 3) * np.array([1, 1, 1 / 60])
        est.update(x_prev, u, x + rng.normal(0, 5e-5, 3) * np.array([1, 1, 10]), w_meas)
        h, h_std = est.estimate()
        model.hole_x = h
        goal[0] = h
        if args.verbose and k % 10 == 0:
            print(f"      hole estimate {h * 1000:+.2f} mm (std {h_std * 1000:.2f}), true {args.hole_x * 1000:+.2f}")
    if args.verbose and k % 10 == 0:
        w = plant.contact_wrench(x, lam, compliance) if nc else np.zeros(3)
        print(f"k {k:3d} tip x {(tip[0]) * 1000:+6.2f} z {tip[1] * 1000:+7.2f} mm phi {np.degrees(x[2]):+5.1f} deg "
              f"u-x {np.round((u - x) * 1000, 2)} contact force {np.round(w, 2)} contacts {nc}")
    if tip[1] < -plant.g.depth + 0.001 and abs(tip[0]) < 0.001:
        break
tip = plant.tip(x)
ok = tip[1] < -plant.g.depth + 0.001
print(f"policy {args.policy} hole {args.hole_x * 1000:+.1f} est {args.est_x * 1000:+.1f} mm: {'OK' if ok else 'FAIL'} "
      f"after {k + 1} steps, tip x {tip[0] * 1000:+.2f} z {tip[1] * 1000:+.2f} mm, phi {np.degrees(x[2]):+.1f} deg"
      + (f"; solve {np.mean(ms):.0f} ms mean {np.max(ms):.0f} max, complementarity violation {np.mean(viol):.3f}" if ms else ""))
