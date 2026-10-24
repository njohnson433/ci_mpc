#!/usr/bin/env python3
# Binary option labels for the option selector (lever 2 of the AlphaNIST ideas, docs/PROGRESS.md 2026-10-24): for
# every recorded state (selector_dagger.py files with "state"), each option runs in a plant copy (its planned
# commands, then the force-following heuristic up to --n steps) and is labeled by whether the peg *enters* (tip
# below -depth) without exceeding --f-max (hard force gate). Written as scores S in the trainer's format:
# 10 * success - w_tie * peak force (tie-break among successes only; failed solves -inf). States where no option
# succeeds carry no binary information and are dropped (counted).
#   relabel_binary.py IN.npz OUT.npz [--n 20 --depth 0.002 --f-max 6 --w-tie 0.05]
import argparse
import copy

import numpy as np

from ci_mpc_planner.c3_student import C3Selector
from ci_mpc_planner.peg_hole_3d import PegHole3D
from ci_mpc_planner.sampled_teacher import SampledTeacher, heuristic_command

p = argparse.ArgumentParser()
p.add_argument("inp")
p.add_argument("out")
p.add_argument("--n", type=int, default=20, help="rollout steps per option")
p.add_argument("--depth", type=float, default=0.002, help="entered: tip this far below the opening [m]")
p.add_argument("--f-max", type=float, default=6.0, help="hard force gate [N]")
p.add_argument("--w-tie", type=float, default=0.05)
p.add_argument("--cont", default="heuristic", choices=["heuristic", "selector"],
               help="policy after the option's planned commands (selector: the current policy, re-solved each step; "
                    "labels then read 'option now, then the current policy, enters')")
p.add_argument("--selector", default="")
p.add_argument("--first", type=int, default=0, help="relabel only this many states (0: all; informativeness check)")
args = p.parse_args()
comp = np.array([1 / 1500, 1 / 1500, 1 / 500, 1 / 130, 1 / 130])
fa = np.array([2.0, 2.0, 3.5, 0.05, 0.05])
d = np.load(args.inp)
lim = args.first or len(d["F"])
F, M, ST = d["F"][:lim], d["meta"][:lim], d["state"][:lim]
sel = C3Selector(args.selector) if args.cont == "selector" else None
S_new, keep = np.full((len(F), SampledTeacher.N_OPTIONS), -np.inf), np.zeros(len(F), bool)
for k, st in enumerate(ST):
    off, x0, u0, w0 = st[:2], st[2:7], st[7:12], st[12:17]
    plant = PegHole3D(hole_xy=-off, n_ring=8, n_rim=12)
    teacher = SampledTeacher(PegHole3D(hole_xy=-off, n_ring=8, n_rim=12), comp, fa)
    goal = np.array([*(-off), plant.g.tip_below_tcp - plant.g.depth, 0.0, 0.0])
    for i in range(SampledTeacher.N_OPTIONS):
        o = teacher.option(i, x0, goal, u0, w0)
        if o is None:
            continue
        _, _, seq, ffs = o
        pc, x, w, peak, ok = copy.copy(plant), x0.copy(), w0.copy(), 0.0, False
        for t in range(args.n):
            if t < len(seq):
                u, ff = seq[t], (np.zeros(5) if ffs is None else ffs[t])
            elif sel is not None:
                oc = teacher.option(sel.choose(C3Selector.features(x, -off, w, plant.g.tip_below_tcp)), x, goal, u, w)
                u, ff = (oc[0], oc[1]) if oc is not None else heuristic_command(x, w)
            else:
                u, ff = heuristic_command(x, w)
            x, lam, nc = pc.step(x, u + comp * ff, comp, fa, eps=1e-6, max_iter=4000)
            w = pc.contact_wrench(lam) if nc else np.zeros(5)
            peak = max(peak, np.linalg.norm(w[:3]))
            if peak > args.f_max:
                break
            if pc.tip(x)[2] < -args.depth:
                ok = True
                break
        S_new[k, i] = 10.0 * ok - args.w_tie * peak
    keep[k] = (S_new[k] > 5.0).any()
np.savez(args.out, F=F[keep], S=S_new[keep], meta=M[keep], state=ST[keep], S_dense=d["S"][:lim][keep])
n_succ = (S_new > 5.0).sum(1)
informative = ((S_new > 5.0).any(1) & ((S_new <= 5.0) & np.isfinite(S_new)).any(1)).mean()
print(f"informative (some options enter, some do not): {informative:.2f}")
print(f"{args.inp}: {keep.sum()}/{len(F)} states kept (no option enters: {(~keep).sum()}); options entering per kept "
      f"state: mean {n_succ[keep].mean():.1f}; heuristic enters in {(S_new[keep, 0] > 5).mean():.2f}, dense-score best "
      f"enters in {(S_new[keep][np.arange(keep.sum()), d['S'][:lim][keep].argmax(1)] > 5).mean():.2f}")
