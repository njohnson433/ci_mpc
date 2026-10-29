"""CI-MPC stage B1 v3 (pre-registered in PROGRESS 2026-10-29 15:35): the primitive SDF contact model on transitional
contacts. Samples every 0.1 s in the res.json `center` / `push` phases (without the last 0.15 s of the final push), peg
pose from peg_truth.npz, wrench on the peg at the tip from the wrist F/T (lag-corrected, mean +-0.05 s, tared before
wrist_t_hover0). Same model and metrics as b1v2_contact_model.py; M4 only on samples where the wrong model's contact set
differs (contact flag differs or Jaccard < 0.5), wrong models: hole +0.5 mm (primary) / +2 mm in x; n >= 30 required.
    python3 scripts/b1v3_contact_model.py
"""
import glob
import json
import os
import re
import sys

import numpy as np
from scipy.optimize import nnls

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ci_mpc_planner"))
import wrist_csv  # noqa: E402
from b1v2_contact_model import GEO, HAND_TO_TCP, L, N_FRIC, GAP, peg_points, quat_wxyz  # noqa: E402
from ci_mpc_planner.hole_belief import profile_sdf  # noqa: E402

SETS = {"D1": sorted(glob.glob(os.path.expanduser("~/ci_mpc_runs/pushstop/d1_0.000[37]/plain/probe_*"))),
        "dev": sorted(glob.glob(os.path.expanduser("~/ci_mpc_runs/b1v3/dev_round_8mm_*")))}


def samples(ep):
    r = json.load(open(os.path.join(ep, "res.json")))
    h0 = r.get("wrist_t_hover0")
    if not r.get("phases") or h0 is None:
        return []
    tw, fw, tqw = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))
    tare = (tw >= h0 - 0.25) & (tw <= h0)
    f0, t0 = np.median(fw[tare], 0), np.median(tqw[tare], 0)
    pt = np.load(os.path.join(ep, "peg_truth.npz"))["log"]
    out = []
    ph = [p for p in r["phases"] if p[0] in ("center", "push")]
    for i, (name, a, b, *_) in enumerate(ph):
        end = b - 0.15 if i == len(ph) - 1 else b
        for t in np.arange(a, end, 0.1):
            m = (tw >= t - 0.05) & (tw <= t + 0.05)
            if m.sum() < 2:
                continue
            row = pt[np.argmin(abs(pt[:, 0] - t))]
            if abs(row[0] - t) > 0.06:
                continue
            F, tq = fw[m].mean(0) - f0, tqw[m].mean(0) - t0
            tcp, qt = row[1:4], row[4:8]
            Rh = quat_wxyz([qt[3], qt[0], qt[1], qt[2]])
            hand = tcp - Rh[:, 2] * HAND_TO_TCP
            part, Rp = row[8:11], quat_wxyz(row[11:15])
            out.append({"part": part, "Rp": Rp, "F": -F, "tau": -(tq + np.cross(hand - part, F))})
    return out


def model(h, g, hole, dx=0.0):
    pp = peg_points(g)
    W = h["part"] + pp @ h["Rp"].T
    rel = W - (hole + [dx, 0.0, 0.0])
    r = np.hypot(rel[:, 0], rel[:, 1])
    dist, nr, nz = profile_sdf(r, rel[:, 2], g)
    rr = np.maximum(r, 1e-12)
    return W, np.stack([nr * rel[:, 0] / rr, nr * rel[:, 1] / rr, nz], 1), dist < GAP


def residual(h, g, W, n, c):
    cols = []
    for pi, ni in zip(W[c] - h["part"], n[c]):
        t1 = np.cross(ni, [0, 0, 1.0]) if abs(ni[2]) < 0.9 else np.cross(ni, [1.0, 0, 0])
        t1 /= np.linalg.norm(t1); t2 = np.cross(ni, t1)
        for a_ in np.linspace(0, 2 * np.pi, N_FRIC, endpoint=False):
            gg = ni + g.mu * (np.cos(a_) * t1 + np.sin(a_) * t2)
            cols.append(np.r_[gg, np.cross(pi, gg) / L])
    y = np.r_[h["F"], h["tau"] / L]
    if not cols:
        return 1.0  # no predicted contact explains nothing
    A = np.array(cols).T
    lam, _ = nnls(A, y)
    return float(np.linalg.norm(y - A @ lam) / np.linalg.norm(y))


for name, eps in SETS.items():
    hs = []
    for ep in eps:
        if not os.path.exists(os.path.join(ep, "peg_truth.npz")):
            continue
        hole = np.array([float(v) for v in re.search(r"HOLE opening at \[([^\]]*)\]",
                                                     open(os.path.join(ep, "run.out")).read()).group(1).split()])
        hs += [(s_, hole) for s_ in samples(ep)]
    if not hs:
        print(f"{name}: no samples")
        continue
    g = GEO["dev" if name == "dev" else "factory"]
    m1, res = [], []
    diff = {0.0005: ([], []), 0.002: ([], [])}
    for h, hole in hs:
        W, n, c = model(h, g, hole)
        meas = np.linalg.norm(h["F"]) > 1.0
        m1.append(bool(c.any()) == meas)
        if not meas:
            continue
        rt = residual(h, g, W, n, c) if c.any() else 1.0
        if c.any():
            res.append(rt)
        for dx, (a_, b_) in diff.items():
            Ww, nw, cw = model(h, g, hole, dx)
            inter, uni = (c & cw).sum(), (c | cw).sum()
            differs = (c.any() != cw.any()) or (uni > 0 and inter / uni < 0.5)
            if differs:
                a_.append(rt); b_.append(residual(h, g, Ww, nw, cw) if cw.any() else 1.0)
    med, p90 = np.median(res), np.percentile(res, 90)
    a5, b5 = diff[0.0005]
    a2, b2 = diff[0.002]
    r5 = np.median(a5) / np.median(b5) if len(b5) and np.median(b5) > 0 else np.nan
    r2 = np.median(a2) / np.median(b2) if len(b2) and np.median(b2) > 0 else np.nan
    testable = len(a5) >= 30
    ok = np.mean(m1) >= 0.90 and med <= 0.20 and p90 <= 0.45 and testable and r5 <= 0.5
    print(f"{name}: samples {len(hs)} (runs {len(eps)}) | M1 {np.mean(m1) * 100:.1f} % | M2v2 median {med:.3f} p90 {p90:.3f} "
          f"(n {len(res)}) | M4 0.5 mm: n differing {len(a5)}, true {np.median(a5) if a5 else np.nan:.3f} vs wrong "
          f"{np.median(b5) if b5 else np.nan:.3f}, ratio {r5:.2f} | M4 2 mm: n {len(a2)}, ratio {r2:.2f} -> "
          f"{'PASS' if ok else ('UNTESTABLE' if not testable else 'FAIL')}")
