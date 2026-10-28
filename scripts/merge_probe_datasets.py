#!/usr/bin/env python3
"""Merge probe_dataset.py-format npz files (run indices re-based, run names concatenated), optionally dropping runs
whose name contains one of --exclude. Used for v5: batched collection (train) + held-out pipeline sweeps (val via
train_probe_likelihood.py --train-filter batched --val-filter sweeps2 --val 1.0).
    python3 scripts/merge_probe_datasets.py OUT.npz A.npz B.npz ... [--exclude sweeps2/w0_e000,sweeps2/w0_e001]
"""
import argparse

import numpy as np

KEYS = ("run", "k", "tcp", "hole", "tip", "force", "calibrated")
p = argparse.ArgumentParser()
p.add_argument("out")
p.add_argument("inputs", nargs="+")
p.add_argument("--exclude", default="", help="comma list of run-name substrings to drop")
args = p.parse_args()
excl = [e for e in args.exclude.split(",") if e]
parts, names = {k: [] for k in KEYS}, []
for f in args.inputs:
    d = np.load(f)
    runs = [str(r) for r in d["runs"]]
    keep_run = np.array([not any(e in r for e in excl) for r in runs])
    new_idx = np.cumsum(keep_run) - 1 + len(names)
    m = keep_run[d["run"]]
    for k in KEYS:
        v = d[k][m]
        parts[k].append(new_idx[v] if k == "run" else v)
    names += [r for r, kr in zip(runs, keep_run) if kr]
    print(f"{f}: {int(keep_run.sum())} / {len(runs)} runs, {int(m.sum())} probes")
np.savez(args.out, runs=np.array(names), **{k: np.concatenate(v) for k, v in parts.items()})
print(f"-> {args.out}: {len(names)} runs, {sum(len(v) for v in parts['run'])} probes")
