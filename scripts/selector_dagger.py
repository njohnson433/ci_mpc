#!/usr/bin/env python3
# DAgger data for the option selector (amortized sampled C3, step 3): episodes from random engaged starts; at every
# visited state the sampled teacher scores all options (heuristic + 9 C3 warm starts, privileged true hole); the
# executed option is the teacher's best with probability --beta, else the selector's choice (--selector .npz).
# Records the student features (TCP vs the *estimated* hole, measured wrench) and all option scores.
#   selector_dagger.py --out d.npz --episodes 5 --seed 0 [--selector sel.npz --beta 0.5]
import argparse

import numpy as np

from ci_mpc_planner.c3_student import C3Selector
from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.sampled_teacher import SampledTeacher

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--steps", type=int, default=60)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--selector", default="")
p.add_argument("--beta", type=float, default=1.0, help="probability of executing the teacher's choice")
args = p.parse_args()
rng = np.random.default_rng(args.seed)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
sel = C3Selector(args.selector) if args.selector else None
F, S, M = [], [], []
entered = 0
for ep in range(args.episodes):
    r, a = rng.uniform(0.3, 1.8), rng.uniform(0, 2 * np.pi)
    off = np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0
    err = rng.normal(0, 0.2, 2) / 1000.0
    plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
    tb = plant.g.tip_below_tcp
    x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
    for _ in range(10):
        x, lam, nc = plant.step(x, np.array([0, 0, x[2] - 0.004, 0, 0]) + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    teacher = SampledTeacher(PegHole3D(hole_xy=-off, n_ring=8, n_rim=12), comp, fa)
    goal = np.array([*(-off), tb - plant.g.depth, 0.0, 0.0])
    u = x.copy()
    for k in range(args.steps):
        w_meas = w + rng.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])
        f = C3Selector.features(x, -off + err, w_meas, tb)
        u_t, ff_t, info = teacher.act(plant, x, goal, u, w)
        F.append(f), S.append(info["scores"]), M.append((ep, k))
        choice = info["best"]
        if sel is not None and rng.random() >= args.beta:
            choice = sel.choose(f)
            o = teacher.option(choice, x, goal, u, w)
            u_t, ff_t = (o[0], o[1]) if o is not None else (u_t, ff_t)
        u = u_t
        x, lam, nc = plant.step(x, u + comp * ff_t, comp, fa)
        w = plant.contact_wrench(lam) if nc else np.zeros(5)
        if plant.tip(x)[2] < -0.005:
            entered += 1
            break
    print(f"episode {ep}: offset {np.round(off * 1000, 2)} mm, {k + 1} steps, tip z {plant.tip(x)[2] * 1000:+.2f} mm",
          flush=True)
np.savez(args.out, F=np.array(F), S=np.array(S), meta=np.array(M))
print(f"saved {len(F)} samples, entered {entered}/{args.episodes}")
