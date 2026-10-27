#!/usr/bin/env python3
# Why does the 3D C3 handover that inserts 16/16 offline (c3_stall_diag.py) do worse in Isaac? Replays the C3 phases
# logged by peg_insert_demo.py --entry c3 (c3_trace.json: model-frame state x, command u, tared force per step, the
# belief estimate and the true hole) against the offline model:
#   1. one-step mismatch: model prediction x+ = PegHole3D(true hole).step(x_k, u_k) vs the measured x_{k+1};
#   2. closed loop from the Isaac start state: C3 on the model plant with the model hole at the estimate (as in Isaac)
#      and at the true hole. Fails with the estimate but inserts with the truth: the estimate error is the cause;
#      inserts offline either way while Isaac stalled: model mismatch.
#   c3_isaac_replay.py RUN_DIR [RUN_DIR ...] [--steps 150]
import argparse
import glob
import json
import os

import numpy as np

from ci_mpc_planner.c3_3d import C33Config, PegHoleC3_3D
from ci_mpc_planner.peg_hole import PegHoleGeometry
from ci_mpc_planner.peg_hole_3d import PegHole3D

p = argparse.ArgumentParser()
p.add_argument("dirs", nargs="+", help="run dirs containing c3_trace.json (globs are expanded)")
p.add_argument("--steps", type=int, default=150)
p.add_argument("--no-closed-loop", action="store_true")
p.add_argument("--grasp-offset", action="store_true", help="plant: the true peg tip offset from the TCP (run.out)")
p.add_argument("--set", action="append", default=[], help="C33Config override name=value for the replayed C3")
args = p.parse_args()

fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
files = sorted({f for d in args.dirs for f in glob.glob(os.path.join(os.path.expanduser(d), "**", "c3_trace.json"),
                                                         recursive=True)})


def cfg_from_args():
    cfg = C33Config()
    for s_ in args.set:
        k, v = s_.split("=")
        cur = getattr(cfg, k)
        setattr(cfg, k, np.array([float(t) for t in v.split(",")]) if isinstance(cur, np.ndarray) else type(cur)(float(v)))
    return cfg


def make_plant(geo, hole, go):
    pl = PegHole3D(geo, hole_xy=hole, n_ring=8, n_rim=12)
    pl.tcp_offset = np.array([-go[0], -go[1], geo.tip_below_tcp])
    return pl


def closed_loop(x0, hole_model, hole_true, goal, comp, geo, go):
    plant = make_plant(geo, hole_true, go)
    mpc = PegHoleC3_3D(PegHole3D(geo, hole_xy=hole_model, n_ring=8, n_rim=12), comp, cfg_from_args())
    x, u, peak = np.array(x0, float), np.array(x0, float), 0.0
    for k in range(args.steps):
        u, _ = mpc.solve(x, goal, u)
        x, lam, nc = plant.step(x, u, comp, fa)
        if nc:
            peak = max(peak, float(np.linalg.norm(plant.contact_wrench(lam)[:3])))
        if -plant.tip(x)[2] > geo.depth - 0.001:
            break
    return -plant.tip(x)[2] * 1000, k + 1, peak


print(f"{len(files)} trace files")
for f in files:
    for i, tr in enumerate(json.load(open(f))):
        S = np.array(tr["steps"])
        if len(S) < 2:
            continue
        geo = PegHoleGeometry(tip_below_tcp=tr["tip_below_tcp"])
        go = np.zeros(2)
        if args.grasp_offset:
            import re
            m = re.search(r"peg bottom \[\s*([-\d.]+)\s+([-\d.]+)\] mm from the TCP", open(os.path.join(os.path.dirname(f), "run.out")).read())
            go = np.array([float(m.group(1)), float(m.group(2))]) / 1000.0 if m else go
        comp, est, true, goal = (np.array(tr[k]) for k in ("comp", "est", "true", "goal"))
        X, U, F = S[:, 1:6], S[:, 6:11], S[:, 11:14]
        plant = make_plant(geo, true, go)
        err = []
        for k in range(len(S) - 1):
            xp, _, _ = plant.step(X[k], U[k], comp, fa)
            err.append((xp - X[k + 1])[:3] * 1000)
        err = np.array(err)
        dz_meas = (X[1:, 2] - X[:-1, 2]) * 1000
        name = os.path.relpath(f, os.path.expanduser("~/ci_mpc_runs"))
        line = (f"{name} #{i} (grasp offset {np.round(go * 1000, 2)} mm): est err {np.linalg.norm(est - true) * 1000:.2f} mm (std {tr['std'] * 1000:.2f}), "
                f"Isaac depth {tr.get('depth_end', float('nan')) * 1000:.1f} mm in {len(S)} steps, "
                f"|F| med {np.median(np.linalg.norm(F, axis=1)):.2f} N; one-step error xy {np.median(np.linalg.norm(err[:, :2], axis=1)):.3f} "
                f"z {np.median(np.abs(err[:, 2])):.3f} mm (median; measured |dz| {np.median(np.abs(dz_meas)):.3f})")
        if not args.no_closed_loop:
            de, ke, pe = closed_loop(X[0], est, true, goal, comp, geo, go)
            goal_t = np.array([true[0], true[1], goal[2], 0.0, 0.0])
            dt_, kt, pt = closed_loop(X[0], true, true, goal_t, comp, geo, go)
            line += (f" | offline from this start: model at estimate {de:.1f} mm/{ke} steps/{pe:.2f} N, "
                     f"at truth {dt_:.1f} mm/{kt} steps/{pt:.2f} N")
        print(line, flush=True)
