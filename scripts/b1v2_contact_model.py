"""CI-MPC stage B1 v2 (pre-registered in PROGRESS 2026-10-29 14:10): the primitive SDF contact model vs the wrist F/T
on new collect-mode episodes with the sim-truth peg pose (peg_truth.npz, src-35's recorder).
Per probe hold (trace.npz: ff z -3 N at K_z 500, mean over the last 0.3 s, tared over the 0.25 s before):
  peg pose = truth part origin (bottom centre) + body z up the peg; hole = asset-spec dimensions (analytic SDF);
  wrench on the peg at the tip centre = -(wrist force, wrist torque moved from the hand origin to the tip).
  M1    contact detection: (min gap < 0.2 mm) vs (|F| > 1 N)                               target >= 90 %
  M2v2  NNLS of [F; tau / L] on (g, (p_i - tip) x g / L), L = 4 mm, relative residual   median <= 0.20, p90 <= 0.45
  M4    discrimination: M2v2 median <= 0.5 x the same with the hole 2 mm off in x
    python3 scripts/b1v2_contact_model.py [EP_DIR ...]   (default ~/ci_mpc_runs/b1v2/*)
    --perturb: B1b sensitivity (dimensions, hole xy) on the same holds
"""
import argparse
import dataclasses
import glob
import os
import re
import sys

import numpy as np
from scipy.optimize import nnls

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ci_mpc_planner"))
import wrist_csv  # noqa: E402
from ci_mpc_planner.hole_belief import profile_sdf  # noqa: E402
from ci_mpc_planner.peg_hole import PegHoleGeometry  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("episodes", nargs="*", default=sorted(glob.glob(os.path.expanduser("~/ci_mpc_runs/b1v2/*"))))
p.add_argument("--perturb", action="store_true")
args = p.parse_args()
GAP, L, N_RING, N_FRIC, HAND_TO_TCP = 2e-4, 0.004, 24, 8, 0.1034
GEO = {"factory": PegHoleGeometry(),  # Factory 8 mm peg / hole (asset spec)
       "dev": PegHoleGeometry(R=0.00405, c=1e-5, depth=0.015, a=0.004, pc=0.0005, top_half_width=0.015, mu=0.75)}


def quat_wxyz(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def peg_points(g):
    ang = np.linspace(0, 2 * np.pi, N_RING, endpoint=False)
    ring = lambda r, z: np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(N_RING, z)])  # noqa: E731
    pts = [ring(g.a - g.pc, 0.0), ring(g.a, g.pc), ring(0.5 * (g.a - g.pc), 0.0)]
    pts += [ring(g.a, z) for z in (0.001, 0.002, 0.004, 0.007, 0.010)]
    return np.vstack(pts + [[[0.0, 0.0, 0.0]]])


def holds(ep):
    tr = np.load(os.path.join(ep, "trace.npz"))
    t = tr["t"] + tr["t0"]
    ff, K = tr["ff"][:, 2], tr["K"][:, 2]
    tw, fw, tqw = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))  # world frame, robot on environment
    pt = np.load(os.path.join(ep, "peg_truth.npz"))["log"]
    m = (np.abs(ff + 3.0) < 1e-6) & (K == 500.0)
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    out = []
    for a, b in zip(e[::2], e[1::2]):
        if not 0.15 < t[b - 1] - t[a] < 1.5:
            continue
        tare = (tw >= t[a] - 0.25) & (tw <= t[a])
        hold = (tw >= t[b - 1] - 0.3) & (tw <= t[b - 1])
        ph = (pt[:, 0] >= t[b - 1] - 0.3) & (pt[:, 0] <= t[b - 1])
        if tare.sum() < 3 or hold.sum() < 3 or ph.sum() < 2:
            continue
        F = fw[hold].mean(0) - np.median(fw[tare], 0)
        tq = tqw[hold].mean(0) - np.median(tqw[tare], 0)
        row = pt[np.flatnonzero(ph)[ph.sum() // 2]]
        tcp, qt = row[1:4], row[4:8]  # quat xyzw
        Rh = quat_wxyz([qt[3], qt[0], qt[1], qt[2]])
        hand = tcp - Rh[:, 2] * HAND_TO_TCP  # fr3_hand origin (the wrist F/T reference point)
        part, Rp = row[8:11], quat_wxyz(row[11:15])
        tau_tip = tq + np.cross(hand - part, F)  # robot-on-env torque moved to the peg tip centre
        out.append({"part": part, "Rp": Rp, "F": -F, "tau": -tau_tip})
    return out


def evaluate(hs, g, hole, dxy=(0.0, 0.0)):
    pp = peg_points(g)
    rows = []
    for h in hs:
        W = h["part"] + pp @ h["Rp"].T
        rel = W - (hole + [dxy[0], dxy[1], 0.0])
        r = np.hypot(rel[:, 0], rel[:, 1])
        dist, nr, nz = profile_sdf(r, rel[:, 2], g)
        rr = np.maximum(r, 1e-12)
        n = np.stack([nr * rel[:, 0] / rr, nr * rel[:, 1] / rr, nz], 1)
        F, tau = h["F"], h["tau"]
        c = dist < GAP
        meas = np.linalg.norm(F) > 1.0
        row = {"pred": bool(c.any()), "meas": bool(meas)}
        if meas and c.any():
            cols = []
            for pi, ni in zip(W[c] - h["part"], n[c]):
                t1 = np.cross(ni, [0, 0, 1.0]) if abs(ni[2]) < 0.9 else np.cross(ni, [1.0, 0, 0])
                t1 /= np.linalg.norm(t1); t2 = np.cross(ni, t1)
                for a_ in np.linspace(0, 2 * np.pi, N_FRIC, endpoint=False):
                    gg = ni + g.mu * (np.cos(a_) * t1 + np.sin(a_) * t2)
                    cols.append(np.r_[gg, np.cross(pi, gg) / L])
            A = np.array(cols).T
            y = np.r_[F, tau / L]
            lam, _ = nnls(A, y)
            row["res"] = float(np.linalg.norm(y - A @ lam) / np.linalg.norm(y))
        rows.append(row)
    return rows


def metrics(rows):
    m1 = np.mean([r["pred"] == r["meas"] for r in rows])
    rs = np.array([r["res"] for r in rows if "res" in r])
    return m1, (np.median(rs) if len(rs) else np.nan), (np.percentile(rs, 90) if len(rs) else np.nan), len(rs)


sets = {}
for ep in args.episodes:
    if not (os.path.exists(os.path.join(ep, "trace.npz")) and os.path.exists(os.path.join(ep, "peg_truth.npz"))):
        print(f"skip {ep}: trace.npz / peg_truth.npz missing")
        continue
    kind = "dev" if os.path.basename(ep).startswith("dev") else "factory"
    hole = np.array([float(v) for v in re.search(r"HOLE opening at \[([^\]]*)\]",
                                                 open(os.path.join(ep, "run.out")).read()).group(1).split()])
    sets.setdefault(kind, []).append((holds(ep), hole))
for kind, eps in sets.items():
    g = GEO[kind]
    rows = sum((evaluate(hs, g, hole) for hs, hole in eps), [])
    wrong = sum((evaluate(hs, g, hole, (0.002, 0.0)) for hs, hole in eps), [])
    m1, med, p90, n = metrics(rows)
    _, wmed, _, wn = metrics(wrong)
    ok = m1 >= 0.90 and med <= 0.20 and p90 <= 0.45 and med <= 0.5 * wmed
    print(f"{kind}: holds {len(rows)} | M1 {m1 * 100:.1f} % | M2v2 median {med:.3f} p90 {p90:.3f} (n {n}) | "
          f"M4 wrong-model median {wmed:.3f} (n {wn}), ratio {med / wmed if wmed else np.nan:.2f} -> B1 v2 {'PASS' if ok else 'FAIL'}")
    if args.perturb:
        for name in ("R", "a", "c"):
            for dv in (-1e-3, -3e-4, -1e-4, 1e-4, 3e-4, 1e-3):
                gp = dataclasses.replace(g, **{name: getattr(g, name) + dv})
                if gp.c <= 0 or gp.a <= gp.pc or gp.a >= gp.R + 1e-3:
                    continue
                m1p, medp, p90p, np_ = metrics(sum((evaluate(hs, gp, hole) for hs, hole in eps), []))
                print(f"   {name} {dv * 1000:+.1f} mm: M1 {m1p * 100:5.1f} %, M2v2 median {medp:.3f} p90 {p90p:.3f} (n {np_})")
        for d in (1e-4, 3e-4, 1e-3):
            m1p, medp, p90p, np_ = metrics(sum((evaluate(hs, g, hole, (d, 0.0)) for hs, hole in eps), []))
            print(f"   hole x +{d * 1000:.1f} mm: M1 {m1p * 100:5.1f} %, M2v2 median {medp:.3f} p90 {p90p:.3f} (n {np_})")
