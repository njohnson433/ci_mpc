"""In-hand slip / press force summary of fidelity_probe_replay.py outputs (vectorized step 4): tip height of probes
> 6.5 mm from the true hole (surely on the flat face; not class-filtered, a slipped tip reads "engaged") per
probe-index block (pipeline vs batched), the same right after a probe the pipeline logged in the hole, flat Fz and
class agreement.
    python scripts/isaac/fidelity_slip.py ~/ci_mpc_runs/vec/step4/f1.npz [more.npz ...] [--episodes 8]
"""
import argparse
import json

import numpy as np

p = argparse.ArgumentParser()
p.add_argument("npz", nargs="+")
p.add_argument("--episodes", type=int, default=0, help="only the first N episodes (to compare with --max-episodes N)")
args = p.parse_args()
BLOCKS = ((0, 10), (10, 20), (20, 30), (30, 40))


def cls(tip, F):  # as fidelity_probe_replay.py
    lat = np.linalg.norm(F[..., :2], axis=-1)
    return np.where(tip < -0.003, 2, np.where((tip < -3e-4) | (lat > 1.2), 1, 0))


def summary(tip, F, far, other_cls):
    c = cls(tip, F)
    out = {}
    k = np.broadcast_to(np.arange(tip.shape[1]), tip.shape)
    flat_far = far & (tip > -0.003)  # NaN compares False
    out["flat_far_tip_mm_by_k"] = [round(float(np.median(tip[flat_far & (k >= a) & (k < b)])) * 1000, 2)
                                   if (flat_far & (k >= a) & (k < b)).any() else None for a, b in BLOCKS]
    prev_in = np.zeros_like(flat_far)
    prev_in[:, 1:] = other_cls[:, :-1] == 2  # the pipeline's previous probe was in the hole
    m = flat_far & prev_in
    out["flat_after_in_hole_mm"] = [round(float(np.median(tip[m])) * 1000, 2) if m.any() else None, int(m.sum())]
    out["flat_Fz"] = round(float(np.nanmean(F[..., 2][(c == 0) & ~np.isnan(tip)])), 2)
    return out, c


for path in args.npz:
    d = np.load(path)
    n = args.episodes or d["log_tip"].shape[0]
    log_tip, log_F, log_xy, hole = d["log_tip"][:n], d["log_F"][:n], d["log_xy"][:n], d["hole"][:n]
    sim_x, sim_F = d["sim_x"][:n], d["sim_F"][:n]
    # TCP - tip offset of the replay; files before 2026-10-27 (afternoon) lack it: median TCP - logged tip instead
    tip_off = float(d["tip_off"]) if "tip_off" in d else float(np.nanmedian(sim_x[..., 2] - log_tip))
    sim_tip = sim_x[..., 2] - tip_off
    r = np.linalg.norm(log_xy - hole[:, None], axis=-1)
    far = r > 0.0065
    lc = cls(log_tip, log_F)
    log_s, _ = summary(log_tip, log_F, far, lc)
    sim_s, sc = summary(sim_tip, sim_F, far, lc)
    valid = ~np.isnan(log_tip) & ~np.isnan(sim_tip)
    print(f"{path} (episodes {n}): class agree {(lc == sc)[valid].mean():.3f}")
    print(f"  pipeline {json.dumps(log_s)}")
    print(f"  batched  {json.dumps(sim_s)}")
