#!/usr/bin/env python3
# Reliability of the sampled C3 teacher vs the force-following heuristic on random engaged starts (offline).
#   sampled_teacher_eval.py --episodes 5 --seed 0 --policy teacher|heuristic
import argparse
import time

import numpy as np

from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.sampled_teacher import SampledTeacher, heuristic_command

p = argparse.ArgumentParser()
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--steps", type=int, default=60)
p.add_argument("--policy", default="teacher", choices=["teacher", "heuristic"])
args = p.parse_args()
rng = np.random.default_rng(args.seed)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
for ep in range(args.episodes):
    r, a = rng.uniform(0.3, 1.8), rng.uniform(0, 2 * np.pi)
    off = np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0
    plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
    tb = plant.g.tip_below_tcp
    x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
    for _ in range(10):
        x, lam, nc = plant.step(x, np.array([0, 0, x[2] - 0.004, 0, 0]) + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    teacher = SampledTeacher(PegHole3D(hole_xy=-off, n_ring=8, n_rim=12), comp, fa)
    goal = np.array([*(-off), tb - plant.g.depth, 0.0, 0.0])
    u, peak, srcs, t0 = x.copy(), 0.0, {}, time.perf_counter()
    for k in range(args.steps):
        if args.policy == "teacher":
            u, ff, info = teacher.act(plant, x, goal, u, w)
            srcs[info["source"]] = srcs.get(info["source"], 0) + 1
        else:
            u, ff = heuristic_command(x, w)
        x, lam, nc = plant.step(x, u + comp * ff, comp, fa)
        w = plant.contact_wrench(lam) if nc else np.zeros(5)
        peak = max(peak, np.linalg.norm(w[:3]))
        if plant.tip(x)[2] < -0.005:
            break
    tip = plant.tip(x)
    print(f"{args.policy} ep {ep}: offset {np.round(off * 1000, 2)} mm: {'ENTERED' if tip[2] < -0.005 else 'stuck'} after "
          f"{k + 1} steps, tip z {tip[2] * 1000:+.2f} mm, peak force {peak:.2f} N, {(time.perf_counter() - t0) / (k + 1):.1f} s/step, "
          f"sources {srcs}", flush=True)
