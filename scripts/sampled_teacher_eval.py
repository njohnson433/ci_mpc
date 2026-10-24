#!/usr/bin/env python3
# Reliability of the sampled C3 teacher vs the force-following heuristic on random engaged starts (offline).
#   sampled_teacher_eval.py --episodes 5 --seed 0 --policy teacher|heuristic
import argparse
import time

import numpy as np

from ci_mpc_planner.peg_hole import PegHoleGeometry
from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.c3_student import C3Selector, C3Student
from ci_mpc_planner.sampled_teacher import SampledTeacher, heuristic_command

p = argparse.ArgumentParser()
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--steps", type=int, default=60)
p.add_argument("--policy", default="teacher", choices=["teacher", "heuristic", "student", "selector", "gpu", "refine", "icnn"])
p.add_argument("--student", default="")
p.add_argument("--plant-mu", type=float, default=None, help="held-out H4: true friction (model keeps 0.75)")
p.add_argument("--plant-R", type=float, default=None, help="held-out H4: true hole radius [mm] (model 4.5)")
p.add_argument("--plant-fa-scale", type=float, default=1.0, help="held-out H4: true arm stiction scale")
p.add_argument("--gpu-wforce", type=float, default=0.5, help="gpu policy: weight of the force penalty above f_soft")
p.add_argument("--gpu-fsoft", type=float, default=3.0, help="gpu policy: force [N] above which it is penalized")
p.add_argument("--refine-margin", type=float, default=0.0, help="refine policy: C3 replaces the sample if its score "
               "is at least this much higher")
p.add_argument("--icnn", default="", help="icnn policy: checkpoint from train_icnn.py")
p.add_argument("--est-error", type=float, default=0.2, help="std of the hole estimate error [mm] (student features)")
args = p.parse_args()
rng = np.random.default_rng(args.seed)          # start states only: identical for every policy
rng_noise = np.random.default_rng(args.seed + 10000)  # measurement noise (the selector / student draw from it)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
for ep in range(args.episodes):
    r, a = rng.uniform(0.3, 1.8), rng.uniform(0, 2 * np.pi)
    off = np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0
    geo_true = PegHoleGeometry()
    if args.plant_mu is not None:
        geo_true.mu = args.plant_mu
    if args.plant_R is not None:
        geo_true.R = args.plant_R / 1000.0
    plant = PegHole3D(geo_true, hole_xy=-off, n_ring=8, n_rim=12)
    fa_true = fa * args.plant_fa_scale
    tb = plant.g.tip_below_tcp
    x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
    for _ in range(10):
        x, lam, nc = plant.step(x, np.array([0, 0, x[2] - 0.004, 0, 0]) + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa_true)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    err = rng.normal(0, args.est_error, 2) / 1000.0
    student = C3Student(args.student) if args.policy == "student" else None
    selector = C3Selector(args.student) if args.policy == "selector" else None
    # runtime options use the *estimated* hole (no privileged information)
    runtime = SampledTeacher(PegHole3D(hole_xy=-off + err, n_ring=8, n_rim=12), comp, fa) if selector else None
    goal_est = np.array([*(-off + err), plant.g.tip_below_tcp - plant.g.depth, 0.0, 0.0])
    teacher = SampledTeacher(PegHole3D(hole_xy=-off, n_ring=8, n_rim=12), comp, fa)
    goal = np.array([*(-off), tb - plant.g.depth, 0.0, 0.0])
    u, peak, srcs, t0 = x.copy(), 0.0, {}, time.perf_counter()
    for k in range(args.steps):
        if args.policy == "teacher":
            u, ff, info = teacher.act(plant, x, goal, u, w)
            srcs[info["source"]] = srcs.get(info["source"], 0) + 1
        elif args.policy in ("gpu", "refine"):
            if k == 0 and args.policy == "refine":  # sample, then QP-refine (ci_mpc_planner/refine_mpc.py)
                from ci_mpc_planner.refine_mpc import SampleRefineMPC
                gpu = SampleRefineMPC(comp, fa, margin=args.refine_margin, seed=args.seed, w_force=args.gpu_wforce,
                                      f_soft=args.gpu_fsoft)
            elif k == 0:
                from ci_mpc_planner.sampling_mpc import SamplingMPC
                gpu = SamplingMPC(comp, fa, seed=args.seed, w_force=args.gpu_wforce, f_soft=args.gpu_fsoft)
            wm = w + rng_noise.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])
            u, info = gpu.act(x, -off + err, wm)
            ff = np.zeros(5)
            srcs[info["source"]] = srcs.get(info["source"], 0) + 1
        elif args.policy == "icnn":  # one convex minimization of the learned cost-to-go per step
            if k == 0:
                import torch
                from ci_mpc_planner.icnn import ICNNPlanner, sequence
                planner = ICNNPlanner(args.icnn)
            wm = w + rng_noise.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])
            a_best, _ = planner.plan(x, -off + err, tb)
            u = sequence(torch.tensor(x[None], dtype=torch.float32), torch.tensor(a_best[None], dtype=torch.float32))[0, 0]
            u, ff = u.double().numpy(), np.zeros(5)
            srcs["icnn"] = srcs.get("icnn", 0) + 1
        elif args.policy == "selector":
            wm = w + rng_noise.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])
            choice = selector.choose(C3Selector.features(x, -off + err, wm, tb))
            o = runtime.option(choice, x, goal_est, u, wm)
            if o is None:
                o = runtime.option(0, x, goal_est, u, wm)
            u, ff = o[0], o[1]
            srcs[choice] = srcs.get(choice, 0) + 1
        elif args.policy == "student":
            u = student.command(x, -off + err, w + rng_noise.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01]), tb)
            ff = np.zeros(5)
        else:
            u, ff = heuristic_command(x, w)
        x, lam, nc = plant.step(x, u + comp * ff, comp, fa_true)
        w = plant.contact_wrench(lam) if nc else np.zeros(5)
        peak = max(peak, np.linalg.norm(w[:3]))
        if plant.tip(x)[2] < -0.005:
            break
    tip = plant.tip(x)
    print(f"{args.policy} ep {ep}: offset {np.round(off * 1000, 2)} mm: {'ENTERED' if tip[2] < -0.005 else 'stuck'} after "
          f"{k + 1} steps, tip z {tip[2] * 1000:+.2f} mm, peak force {peak:.2f} N, {(time.perf_counter() - t0) / (k + 1):.1f} s/step, "
          f"sources {srcs}", flush=True)
