#!/usr/bin/env python3
"""Redo the true contact force of saved runs (res.json phases_wrist_N / peak_force_wrist_engaged_N) with the wrist
F/T clock lag (wrist_csv.WRIST_LAG, PROGRESS 2026-10-28 "RETRACTION"): before the fix the phase windows (joint-state
time) cut the CSV 0.235 s early. The tare time t_hover0 is not saved, so it is recovered exactly: the CSV sample whose
old tare window [t - 0.25, t] reproduces the stored phases_wrist_N. Writes res.json["wrist_fixed"] (backs nothing up:
the old values stay in place) and prints old -> new per run.
    python3 scripts/recompute_wrist_peaks.py RUN_ROOT [RUN_ROOT ...] [--write]
"""
import argparse
import glob
import json
import os

import numpy as np

import wrist_csv

p = argparse.ArgumentParser()
p.add_argument("roots", nargs="+")
p.add_argument("--write", action="store_true", help="add res.json['wrist_fixed'] = {phases_wrist_N, peak, ...}")
args = p.parse_args()


def old_peaks(ft, h, phases):
    tw = (ft[:, 0] >= h - 0.25) & (ft[:, 0] <= h)
    fm = np.linalg.norm(ft[:, 1:4] - np.median(ft[tw, 1:4], axis=0), axis=1)
    return [float(fm[(ft[:, 0] >= a) & (ft[:, 0] <= b)].max()) if ((ft[:, 0] >= a) & (ft[:, 0] <= b)).any() else 0.0
            for _, a, b in phases]


n_ok = n_fail = 0
for root in args.roots:
    for res_path in sorted(glob.glob(os.path.join(os.path.expanduser(root), "**", "res.json"), recursive=True)):
        d = os.path.dirname(res_path)
        res = json.load(open(res_path))
        if "phases_wrist_N" not in res or not os.path.exists(os.path.join(d, "wrist_ft.csv")):
            continue
        if res.get("wrist_lag_s"):  # written after the fix
            continue
        ft = np.loadtxt(os.path.join(d, "wrist_ft.csv"), delimiter=",", skiprows=1, ndmin=2)
        phases, old = [ph[:3] for ph in res["phases"]], np.array(res["phases_wrist_N"])  # (name, a, b, observer peak)
        h0 = None
        ts = ft[ft[:, 0] < phases[0][1], 0]  # the hover is before the first engaged phase
        # every distinct tare window: its end on a sample, or its start just past one
        for h in np.unique(np.r_[ts, ts + 0.25 - 1e-9]):
            if np.allclose(old_peaks(ft, h, phases), old, rtol=0, atol=1e-6):
                h0 = h
                break
        if h0 is None:
            print(f"{d}: tare time not recovered, skipped")
            n_fail += 1
            continue
        new = wrist_csv.phase_peaks(os.path.join(d, "wrist_ft.csv"), h0, phases)
        print(f"{d}: peak {max(old):6.2f} -> {max(new):6.2f} N  (phases {np.round(old, 2).tolist()} -> "
              f"{np.round(new, 2).tolist()})")
        n_ok += 1
        if args.write:
            res["wrist_fixed"] = {"phases_wrist_N": new, "peak_force_wrist_engaged_N": max(new), "t_hover0": float(h0),
                                  "wrist_lag_s": wrist_csv.WRIST_LAG}
            json.dump(res, open(res_path, "w"), indent=1)
print(f"recomputed {n_ok}, not recovered {n_fail}")
