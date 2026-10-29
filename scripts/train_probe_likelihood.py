#!/usr/bin/env python3
# Train the learned probe likelihood (ci_mpc_planner/probe_likelihood.py) on probe_dataset.py data: a classifier of
# (measurement, hole hypothesis) pairs, the true hole vs K hypotheses drawn uniformly from the belief grid; its logit is
# the log-likelihood ratio up to a constant, which is what the grid belief needs. Measurements get fresh randomized
# sensor errors every epoch (the FR3 estimates the EE force from link-side joint torque sensors via the model and
# Jacobian; the values below are assumptions until measured on the robot):
#   force: gain / cross-axis error (model, Jacobian), bias drift after the per-probe tare, noise
#   tip height: calibration bias (and creep), noise
# Runs are split train / validation; validation replays every held-out run's logged probes through the grid belief
# (learned vs the hand model as deployed: f_lateral off, touch 0.15 mm) with injected sensor errors at 0, 1, 2x.
# Needs PyTorch: ~/IsaacLab/.venv/bin/python scripts/train_probe_likelihood.py DATA.npz OUT.npz [--no-rand]
import argparse
import os
import random
import sys

import numpy as np
import torch
from torch import nn

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ci_mpc_planner"))
from ci_mpc_planner.hole_belief import HoleBelief  # noqa: E402
from ci_mpc_planner.probe_likelihood import ProbeLikelihood, features  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("data")
p.add_argument("out")
p.add_argument("--no-rand", action="store_true", help="no sensor-error randomization (ablation)")
p.add_argument("--rand-scale", type=float, default=1.0, help="scale of the training randomization")
p.add_argument("--neg", type=int, default=16, help="grid hypotheses per measurement")
p.add_argument("--hidden", type=int, default=64)
p.add_argument("--epochs", type=int, default=80)
p.add_argument("--lr", type=float, default=2e-3)
p.add_argument("--val", type=float, default=0.2)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--all-runs", action="store_true", help="also uncalibrated runs (tip biased by the grasp offset)")
p.add_argument("--train-filter", default="", help="train only on runs whose log path contains one of these (comma list)")
p.add_argument("--val-filter", default="", help="draw validation runs only from runs whose path contains one of these")
p.add_argument("--no-rcorr", action="store_true", help="do not add the distance-prior correction (rprior_correction.py)")
p.add_argument("--rcorr-clamp", action="store_true", help="hold the distance-prior table constant beyond the 99th percentile of\n               the training probe-hole distances (no data there: v5 batched, PROGRESS 2026-10-29)")
p.add_argument("--eval", default="", help="no training: replay-validate this saved model (OUT unused) on the same held-out runs\n               (e.g. v3 vs v5 on --val-filter runs)")
args = p.parse_args()

SPREAD = 6.0  # mm, the belief grid half width (HoleBelief default)
# sensor-error magnitudes at scale 1 (sigma): force gain diag / cross-axis, bias drift [N], noise [N]; tip bias, noise [mm]
ERR = dict(gain=0.10, cross=0.05, bias=0.3, noise=0.2, tip_bias=0.15, tip_noise=0.03)


def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def perturb(F, tip, rng, scale):
    """Randomized sensor errors on N measurements (independent per measurement)."""
    if scale <= 0:
        return F, tip
    n = len(F)
    E = rng.normal(0, ERR["cross"] * scale, (n, 3, 3))
    E[:, [0, 1, 2], [0, 1, 2]] = rng.normal(0, ERR["gain"] * scale, (n, 3))
    F = F + np.einsum("nij,nj->ni", E, F) + rng.normal(0, ERR["bias"] * scale, (n, 3)) \
        + rng.normal(0, ERR["noise"] * scale, (n, 3))
    tip = tip + rng.normal(0, ERR["tip_bias"] * scale, n) + rng.normal(0, ERR["tip_noise"] * scale, n)
    return F, tip


set_seed(args.seed)
rng = np.random.default_rng(args.seed)
d = np.load(args.data)
keep = np.ones(len(d["run"]), bool) if args.all_runs else d["calibrated"]
run, tcp, hole, tip, force = d["run"][keep], d["tcp"][keep], d["hole"][keep], d["tip"][keep], d["force"][keep]
runs = np.unique(run)
names = d["runs"]


def match(r, filt):
    return not filt or any(f in str(names[r]) for f in filt.split(","))


val_pool = np.array([r for r in runs if match(r, args.val_filter)])
val_runs = set(rng.choice(val_pool, int(len(val_pool) * args.val), replace=False).tolist())
is_val = np.array([r in val_runs for r in run])
tr = ~is_val & np.array([match(r, args.train_filter) for r in run])
print(f"{tr.sum()} train / {is_val.sum()} val probes ({len(runs) - len(val_runs)} / {len(val_runs)} runs)")


def pairs(idx, scale):
    """Features and labels: per measurement the true hole and K grid hypotheses; sensor errors on the measurement."""
    F, t = perturb(force[idx], tip[idx], rng, scale)
    K = args.neg
    h_neg = rng.uniform(-SPREAD, SPREAD, (len(idx), K, 2))
    rel = np.concatenate([(hole[idx] - tcp[idx])[:, None], h_neg - tcp[idx][:, None]], axis=1)  # N x (1+K) x 2
    X = np.concatenate([features(rel[i], t[i], F[i]) for i in range(len(idx))])
    y = np.tile(np.r_[1.0, np.zeros(K)], len(idx))
    return X.astype(np.float32), y.astype(np.float32)


if not args.eval:
    tr_idx, va_idx = np.flatnonzero(tr), np.flatnonzero(is_val)
    X0, _ = pairs(tr_idx, 0.0)
    mu, sd = X0.mean(0), X0.std(0) + 1e-6
    net = nn.Sequential(nn.Linear(5, args.hidden), nn.SiLU(), nn.Linear(args.hidden, args.hidden), nn.SiLU(),
                        nn.Linear(args.hidden, 1))
    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=torch.tensor(float(args.neg)))  # balanced classes: logit = log ratio + c
    mu_t, sd_t = torch.tensor(mu), torch.tensor(sd)
    rand_scale = 0.0 if args.no_rand else args.rand_scale
    for ep in range(args.epochs):
        net.train()
        X, y = pairs(tr_idx, rand_scale)
        X, y = (torch.tensor(X) - mu_t) / sd_t, torch.tensor(y)
        perm = torch.randperm(len(X))
        tot = 0.0
        for b in range(0, len(X), 4096):
            i = perm[b:b + 4096]
            opt.zero_grad(set_to_none=True)
            loss = loss_fn(net(X[i])[:, 0], y[i])
            loss.backward()
            opt.step()
            tot += loss.item() * len(i)
        sched.step()
        if ep % 20 == 19 or ep == args.epochs - 1:
            net.eval()
            with torch.no_grad():
                Xv, yv = pairs(va_idx, rand_scale)
                lv = loss_fn(net((torch.tensor(Xv) - mu_t) / sd_t)[:, 0], torch.tensor(yv)).item()
            print(f"epoch {ep + 1}: train {tot / len(X):.4f}  val {lv:.4f}")

    lin = [m for m in net if isinstance(m, nn.Linear)]
    np.savez(args.out, mu=mu, sd=sd, **{f"W{i}": m.weight.detach().numpy().astype(np.float64) for i, m in enumerate(lin)},
             **{f"b{i}": m.bias.detach().numpy().astype(np.float64) for i, m in enumerate(lin)},
             err=np.array([ERR[k] for k in ERR]), rand_scale=rand_scale)
    if not args.no_rcorr:  # distance-prior correction from the training positives vs the grid negatives (as rprior_correction.py)
        r_pos = np.linalg.norm(hole[tr_idx] - tcp[tr_idx], axis=1)
        r_neg = np.linalg.norm(rng.uniform(-SPREAD, SPREAD, (len(tr_idx), 16, 2)) - tcp[tr_idx][:, None], axis=2).ravel()
        edges = np.arange(0.0, 15.01, 0.5)
        p_pos, p_neg = np.histogram(r_pos, edges)[0] + 1.0, np.histogram(r_neg, edges)[0] + 1.0
        rc = np.convolve(np.log((p_pos / p_pos.sum()) / (p_neg / p_neg.sum())), np.ones(3) / 3, mode="same")
        m = dict(np.load(args.out))
        rc = rc - rc[np.searchsorted(edges, 6.0) - 1]
        if args.rcorr_clamp:
            i = max(np.searchsorted(edges, np.percentile(r_pos, 99)) - 1, 0)
            rc[i + 1:] = rc[i]
            print(f"distance-prior table held at {rc[i]:+.2f} beyond {edges[i]:.1f} mm (p99 of the training distances)")
        m["rcorr_edges"], m["rcorr"] = edges, rc
        np.savez(args.out, **m)
    print(f"saved {args.out}" + ("" if args.no_rcorr else " (with distance-prior correction)"))

# replay: every held-out run's logged probes through the grid belief, learned vs hand model, injected sensor errors.
# "search": only probes before the tip sank > 1 mm (a probe in the hole localizes it trivially).
lik = ProbeLikelihood(args.eval or args.out)
TIP_BELOW = 0.040
k_all = np.asarray(d["k"])[keep]
for scale in (0.0, 1.0, 2.0):
    for mode in ("all", "search"):
        res = {"hand": [], "learned": []}
        ev = np.random.default_rng(100)
        for r in sorted(val_runs):
            i = np.flatnonzero(run == r)
            i = i[np.argsort(k_all[i])]
            if mode == "search":
                sank = np.flatnonzero(tip[i] < -1.0)
                i = i[:sank[0]] if len(sank) else i
            if len(i) == 0:
                continue
            F, t = perturb(force[i], tip[i], ev, scale)
            true = hole[i[0]] / 1000.0
            for name in res:
                b = HoleBelief(f_lateral=1e9, touch=1.5e-4, likelihood=lik if name == "learned" else None)
                for j in range(len(i)):
                    b.update(np.array([*(tcp[i[j]] / 1000.0), t[j] / 1000.0 + TIP_BELOW, 0.0, 0.0]), F[j])
                w = b.weights()
                est, _ = b.estimate()
                near = np.linalg.norm(b.h - true, axis=1)
                res[name].append((np.linalg.norm(est - true) * 1000, w[near < 0.002].sum(),
                                  np.log(w[np.argmin(near)] + 1e-12)))
        print(f"replay {mode}, sensor errors x{scale:.0f} ({len(res['hand'])} held-out runs): "
              f"mean error / P(within 2 mm) / log p(true cell)")
        for name, v in res.items():
            v = np.array(v)
            print(f"  {name:8s} {np.median(v[:, 0]):5.2f} mm median  {np.mean(v[:, 1]):.2f}  {np.mean(v[:, 2]):7.2f}")
