#!/usr/bin/env python3
# DAgger data for the option selector (amortized sampled C3, step 3): episodes from random engaged starts; at every
# visited state the sampled teacher scores all options (heuristic + 9 C3 warm starts, privileged true hole); the
# executed option is the teacher's best with probability --beta, else the selector's choice (--selector .npz).
# Records the student features (TCP vs the *estimated* hole, measured wrench) and all option scores.
# --cache start_cache.json: starts drawn success-guided (ci_mpc_planner/start_cache.py) instead of uniform; every run
# also writes <out>_eps.json (per episode: start kind / id / offset / entered, and snapshots of visited states every
# 10 steps) for scripts/start_cache_update.py. --no-label: rollouts of the selector only (no teacher: fast; options
# on the *estimated* hole, as in the fair eval), to estimate success per start before a DAgger round.
#   selector_dagger.py --out d.npz --episodes 5 --seed 0 [--selector sel.npz --beta 0.5] [--cache c.json] [--no-label]
import argparse

import numpy as np

from ci_mpc_planner.c3_student import C3Selector
from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.sampled_teacher import SampledTeacher
from ci_mpc_planner.start_cache import StartCache
import json

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--steps", type=int, default=60)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--selector", default="")
p.add_argument("--beta", type=float, default=1.0, help="probability of executing the teacher's choice")
p.add_argument("--cache", default="", help="success-guided start cache (json); default: uniform starts as before")
p.add_argument("--no-label", action="store_true", help="selector rollouts only, no teacher labels")
args = p.parse_args()
rng = np.random.default_rng(args.seed)
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
sel = C3Selector(args.selector) if args.selector else None
cache = StartCache.load(args.cache) if args.cache else None
F, S, M, EPS, ST = [], [], [], [], []  # ST: plant state per sample (off, x, u, w) for relabeling
entered = 0
for ep in range(args.episodes):
    if cache is None:
        r, a = rng.uniform(0.3, 1.8), rng.uniform(0, 2 * np.pi)
        off = np.array([r * np.cos(a), r * np.sin(a)]) / 1000.0
        start = {"kind": "uniform", "id": -1, "x": None}
    else:
        start = cache.sample(rng)
        off = start["off"]
    err = rng.normal(0, 0.2, 2) / 1000.0
    plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
    tb = plant.g.tip_below_tcp
    if start["x"] is None:
        x = np.array([0.0, 0.0, tb + 0.0005, 0.0, 0.0])
        for _ in range(10):
            x, lam, nc = plant.step(x, np.array([0, 0, x[2] - 0.004, 0, 0]) + comp * np.array([0, 0, -3.0, 0, 0]), comp, fa)
        u = x.copy()
    else:  # snapshot: hold its last command once for a consistent contact state
        x, u = start["x"], start["u"]
        x, lam, nc = plant.step(x, u, comp, fa)
    w = plant.contact_wrench(lam) if nc else np.zeros(5)
    teacher = SampledTeacher(PegHole3D(hole_xy=-off, n_ring=8, n_rim=12), comp, fa)
    runtime = SampledTeacher(PegHole3D(hole_xy=-off + err, n_ring=8, n_rim=12), comp, fa) if args.no_label else None
    goal = np.array([*(-off), tb - plant.g.depth, 0.0, 0.0])
    goal_est = np.array([*(-off + err), tb - plant.g.depth, 0.0, 0.0])
    snaps = []
    for k in range(args.steps):
        if k and k % 10 == 0:
            snaps.append((k, x.copy(), u.copy()))
        w_meas = w + rng.normal(0, 0.2, 5) * np.array([1, 1, 1, 0.01, 0.01])
        f = C3Selector.features(x, -off + err, w_meas, tb)
        if args.no_label:  # as the fair eval: the selector's option on the estimated hole, heuristic if it fails
            o = runtime.option(sel.choose(f), x, goal_est, u, w_meas) or runtime.option(0, x, goal_est, u, w_meas)
            u_t, ff_t = o[0], o[1]
        else:
            u_t, ff_t, info = teacher.act(plant, x, goal, u, w)
            F.append(f), S.append(info["scores"]), M.append((ep, k))
            ST.append(np.concatenate([off, x, u, w]))
            if sel is not None and rng.random() >= args.beta:
                o = teacher.option(sel.choose(f), x, goal, u, w)
                u_t, ff_t = (o[0], o[1]) if o is not None else (u_t, ff_t)
        u = u_t
        x, lam, nc = plant.step(x, u + comp * ff_t, comp, fa)
        w = plant.contact_wrench(lam) if nc else np.zeros(5)
        if plant.tip(x)[2] < -0.005:
            entered += 1
            break
    ok = bool(plant.tip(x)[2] < -0.005)
    EPS.append({"kind": start["kind"], "id": int(start["id"]), "off": off.tolist(), "entered": ok, "steps": k + 1,
                "snaps": [{"step": st, "x": xs.tolist(), "u": us.tolist()} for st, xs, us in snaps]})
    print(f"episode {ep}: {start['kind']} start, offset {np.round(off * 1000, 2)} mm, {k + 1} steps, tip z "
          f"{plant.tip(x)[2] * 1000:+.2f} mm", flush=True)
np.savez(args.out, F=np.array(F), S=np.array(S), meta=np.array(M), state=np.array(ST))
json.dump(EPS, open(args.out.replace(".npz", "_eps.json"), "w"))
print(f"saved {len(F)} samples, entered {entered}/{args.episodes}")
