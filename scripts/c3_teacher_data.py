#!/usr/bin/env python3
# Teacher data for an amortized 3D CI-MPC (step 3): the slow, reliable C3+ (rollout warm start on every solve,
# ~1.5 s) runs on the 3D convex plant from random engaged starts (TCP 0.3-1.8 mm off the hole axis, pressed onto the
# chamfer; hole estimate error ~0.2 mm). Each step records
#   features: TCP relative to the *estimated* hole (x, y [mm], tip depth below the opening [mm], rx, ry [mrad]) and
#             the measured contact wrench (Fx, Fy, Fz [N], Mx, My [N m]) with noise;
#   target:   the teacher's command step u - x (mm, mrad).
# With probability --perturb the executed command is perturbed (DART-style), so the data covers off-trajectory states.
# Episodes stop once the tip is 5 mm into the bore (the rest is a plain push).
#   c3_teacher_data.py --out data/teacher_0.npz --episodes 6 --seed 0
import argparse

import numpy as np

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
from ci_mpc_planner.peg_hole_3d import PegHole3D

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--episodes", type=int, default=6)
p.add_argument("--steps", type=int, default=60)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--perturb", type=float, default=0.3)
args = p.parse_args()

rng = np.random.default_rng(args.seed)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
SC = np.array([1000, 1000, 1000, 1000, 1000.0])
feats, targets, meta = [], [], []
for ep in range(args.episodes):
    r, a = rng.uniform(0.3, 1.8), rng.uniform(0, 2 * np.pi)
    off = np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0
    err = rng.normal(0, 0.2, 2) / 1000.0
    plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
    model = PegHole3D(hole_xy=-off + err, n_ring=8, n_rim=12)
    tb = plant.g.tip_below_tcp
    x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
    for _ in range(10):
        x, lam, nc = plant.step(x, np.array([0, 0, x[2] - 0.004, 0, 0]) + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    cfg = C33Config()
    cfg.rollout_every = 1
    mpc = PegHoleC3_3D(model, comp, cfg)
    goal = np.array([*(-off + err), tb - plant.g.depth, 0.0, 0.0])
    u = x.copy()
    for k in range(args.steps):
        est = -off + err
        f = np.concatenate([(x[:2] - est) * 1000, [(x[2] - tb) * 1000], x[3:] * 1000,
                            w + rng.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])])
        u_t, info = mpc.solve(x, goal, u)
        feats.append(f)
        targets.append((u_t - x) * SC)
        meta.append((ep, k))
        u = u_t.copy()
        if rng.random() < args.perturb:
            u[:3] += rng.normal(0, 0.2e-3, 3)
        x, lam, nc = plant.step(x, u, comp, fa)
        w = plant.contact_wrench(lam) if nc else np.zeros(5)
        if plant.tip(x)[2] < -0.005:
            break
    print(f"episode {ep}: offset {np.round(off * 1000, 2)} mm, {k + 1} steps, tip {np.round(plant.tip(x) * 1000, 2)} mm",
          flush=True)
np.savez(args.out, X=np.array(feats), Y=np.array(targets), meta=np.array(meta))
print(f"saved {len(feats)} samples to {args.out}")
