#!/usr/bin/env python3
# Offline comparison of the engaged phase (after probing localized the hole): 3D C3+ vs the force-following
# heuristic, on the 3D convex plant with arm stiction. The peg starts pressed onto the chamfer at --offset (mm,
# TCP relative to the true hole axis); the model knows the hole up to --est-error (mm, estimate error).
#   peg_c3_3d_offline.py --offset 1.5,0 [--policy c3|follow] [--est-error 0.3,0] [--steps 80]
import argparse
import time

import numpy as np

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
from ci_mpc_planner.c3_student import C3Student
from ci_mpc_planner.peg_hole_3d import PegHole3D

p = argparse.ArgumentParser()
p.add_argument("--offset", default="1.5,0")
p.add_argument("--est-error", default="0,0")
p.add_argument("--policy", default="c3", choices=["c3", "follow", "student"])
p.add_argument("--student", default="", help="student weights (.npz) for --policy student")
p.add_argument("--steps", type=int, default=80)
p.add_argument("--warm-start", default="rollout")
p.add_argument("--success-depth", type=float, default=0.024, help="[m] tip depth counted as success (0.005: entered)")
p.add_argument("--verbose", action="store_true")
args = p.parse_args()

off = np.array([float(v) for v in args.offset.split(",")]) / 1000.0
err = np.array([float(v) for v in args.est_error.split(",")]) / 1000.0
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)  # model frame origin = TCP start xy; the hole at -off
model = PegHole3D(hole_xy=-off + err, n_ring=8, n_rim=12)
tb = plant.g.tip_below_tcp
x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
u = x.copy()
# settle onto the chamfer with a light press
for _ in range(10):
    u = np.array([0.0, 0.0, x[2] - 0.004, 0.0, 0.0])
    x, lam, nc = plant.step(x, u + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa)
goal = np.array([*(-off + err), tb - plant.g.depth, 0.0, 0.0])
cfg = C33Config()
cfg.warm_start = args.warm_start
mpc = PegHoleC3_3D(model, comp, cfg)
peak_f, peak_tau, ms, viol = 0.0, 0.0, [], []
F = np.zeros(3)
w = plant.contact_wrench(lam) if nc else np.zeros(5)
student = C3Student(args.student) if args.policy == "student" else None
rng = np.random.default_rng(1)
for k in range(args.steps):
    if args.policy == "c3":
        t0 = time.perf_counter()
        u, info = mpc.solve(x, goal, u)
        ms.append((time.perf_counter() - t0) * 1000)
        viol.append(info.get("viol", np.nan))
        ff = np.zeros(5)
    elif args.policy == "student":
        t0 = time.perf_counter()
        u = student.command(x, -off + err, w + rng.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01]), tb)
        ms.append((time.perf_counter() - t0) * 1000)
        ff = np.zeros(5)
    else:  # follow the chamfer's lateral push with a light normal force, else press (as in Isaac)
        lat = F[:2]
        if np.linalg.norm(lat) > 0.3:
            d = lat / np.linalg.norm(lat)
            u = np.array([x[0] + 4e-4 * d[0], x[1] + 4e-4 * d[1], x[2] - 0.002, 0, 0])
            ff = np.array([2.5 * d[0], 2.5 * d[1], -2.0, 0, 0])
        else:
            u = np.array([x[0], x[1], x[2] - 0.004, 0, 0])
            ff = np.array([0, 0, -3.0, 0, 0])
    x, lam, nc = plant.step(x, u + comp * ff, comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    F = w[:3]
    peak_f, peak_tau = max(peak_f, np.linalg.norm(w[:3])), max(peak_tau, np.linalg.norm(w[3:]))
    tip = plant.tip(x)
    if args.verbose and k % 5 == 0:
        print(f"k {k:3d} tip {np.round(tip * 1000, 2)} mm, contact force {np.round(F, 2)} N, torque {np.round(w[3:], 3)}"
              + (f", {ms[-1]:.0f} ms, viol {viol[-1]:.2f}, m {info['m']}" if viol else ""))
    if tip[2] < -args.success_depth:
        break
tip = plant.tip(x)
ok = tip[2] < -args.success_depth
print(f"{args.policy} offset {args.offset} est-error {args.est_error}: {'OK' if ok else 'FAIL'} after {k + 1} steps, "
      f"tip {np.round(tip * 1000, 2)} mm; peak contact force {peak_f:.2f} N, torque {peak_tau:.3f} N m"
      + (f"; solve {np.mean(ms):.2f} ms mean {np.max(ms):.2f} max" if ms else "")
      + (f", violation {np.nanmean(viol):.3f}" if viol else ""))
