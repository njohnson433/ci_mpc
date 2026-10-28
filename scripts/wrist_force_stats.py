"""Robust true-contact-force statistics of a saved run (wrist F/T, lag-corrected via wrist_csv), next to the plain max.
The lag-corrected max is often one or two 16.7 ms samples when the peg seats on the bottom (D1 plain (3, 0):
Fz -24.4 N for one sample, then ~8 N; 2026-10-28): a real impact, but not the search / insertion contact force.
    stats(run_dir, res) -> {"max": engaged-phase max |F - tare| [N] (= peak_force_wrist_engaged_N, lag-corrected),
                            "mean50": max of the 50 ms moving mean over the engaged phases,
                            "preseat": max over the engaged phases without the last 0.15 s of the final phase}
The tare time is res["wrist_t_hover0"] (runs from 2026-10-28 evening), res["wrist_fixed"]["t_hover0"]
(recompute_wrist_peaks.py), or recovered from the stored lag-corrected phase peaks (runs in between).
    python3 scripts/wrist_force_stats.py RUN_DIR   # prints the three
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wrist_csv  # noqa: E402

SEAT_S, MEAN_S = 0.15, 0.05


def _t_hover0(path, res, t):
    if res.get("wrist_t_hover0"):
        return res["wrist_t_hover0"]
    if res.get("wrist_fixed"):
        return res["wrist_fixed"]["t_hover0"]
    if res.get("wrist_lag_s") and res.get("phases_wrist_N"):  # lag-corrected peaks stored, tare time not: recover it
        phases = [ph[:3] for ph in res["phases"]]
        ts = t[t < phases[0][1]]
        for h in np.unique(np.r_[ts, ts + 0.25 - 1e-9]):
            pk = wrist_csv.phase_peaks(path, h, phases)
            if pk is not None and np.allclose(pk, res["phases_wrist_N"], rtol=0, atol=1e-6):
                return h
    return None


def stats(run_dir, res):
    path = os.path.join(run_dir, "wrist_ft.csv")
    if not res.get("phases") or not os.path.exists(path):
        return None
    t, f, _ = wrist_csv.load(path)
    h = _t_hover0(path, res, t)
    if h is None:
        return None
    tw = (t >= h - 0.25) & (t <= h)
    fm = np.linalg.norm(f - np.median(f[tw], axis=0), axis=1)
    k = max(1, int(round(MEAN_S / np.median(np.diff(t)))))
    fmean = np.convolve(fm, np.ones(k) / k, mode="same")
    eng = np.zeros(len(t), bool)
    pre = np.zeros(len(t), bool)
    for i, ph in enumerate(res["phases"]):
        a, b = ph[1], ph[2]
        eng |= (t >= a) & (t <= b)
        pre |= (t >= a) & (t <= (b - SEAT_S if i == len(res["phases"]) - 1 else b))
    if not eng.any():
        return None
    return {"max": float(fm[eng].max()), "mean50": float(fmean[eng].max()),
            "preseat": float(fm[pre].max()) if pre.any() else float("nan")}


if __name__ == "__main__":
    d = sys.argv[1]
    print(stats(d, json.load(open(os.path.join(d, "res.json")))))
