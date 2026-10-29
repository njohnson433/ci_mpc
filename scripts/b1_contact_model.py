"""CI-MPC stage B1 (pre-registered in PROGRESS 2026-10-29 14:40): the primitive SDF contact model alone vs Isaac
wrist logs. Per probe hold of src-35's pipeline episodes (vec/step4d/w0_e*): TCP pose (bag trace) + hand orientation
(wrist_ft.csv) + the grasp-time in-hand peg offset -> peg surface points; analytic axisymmetric hole SDF from asset
dimensions (PegHoleGeometry, profile_sdf) -> gaps and normals; measured contact force = wrist F/T (lag-corrected,
tared before the press, mean over the hold's last 0.3 s).
  M1 contact detection: (min gap < 0.2 mm) vs (|F_true| > 1 N)
  M2 relative NNLS residual of F_true on the predicted contacts' friction-cone generators (contact holds)
  M3 lateral angle between F_true,lat and the fit (|F_lat| > 0.5 N)
B1b (--perturb): one dimension / the hole xy shifted at a time, M1-M3 per perturbation.
    python3 scripts/b1_contact_model.py [--episodes ~/ci_mpc_runs/vec/step4d/w0_e00?] [--perturb]
"""
import argparse
import dataclasses
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
from ci_mpc_planner.hole_belief import profile_sdf  # noqa: E402
from ci_mpc_planner.peg_hole import PegHoleGeometry  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--episodes", nargs="*", default=sorted(glob.glob(os.path.expanduser("~/ci_mpc_runs/vec/step4d/w0_e*"))))
p.add_argument("--contact-gap", type=float, default=2e-4)
p.add_argument("--perturb", action="store_true")
p.add_argument("--diag-zcal", action="store_true", help="diagnostic only: shift the peg by the demo's per-episode grasp "
               "height calibration (run.out 'grasp height offset'); not part of the pre-registered B1")
p.add_argument("--diag-support", type=float, default=0.0, help="diagnostic only: M2 / M3 on the support set (contacts "
               "within this distance of the minimum gap) instead of gap < contact-gap")
args = p.parse_args()
G0 = PegHoleGeometry()
N_RING, N_FRIC = 24, 8


def peg_points(g):
    """Peg surface points in the peg frame (tip centre origin, axis +z up the peg)."""
    ang = np.linspace(0, 2 * np.pi, N_RING, endpoint=False)
    ring = lambda r, z: np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(N_RING, z)])  # noqa: E731
    pts = [ring(g.a - g.pc, 0.0), ring(g.a, g.pc), ring(0.5 * (g.a - g.pc), 0.0)]
    pts += [ring(g.a, z) for z in (0.001, 0.002, 0.004, 0.007, 0.010)]
    return np.vstack(pts + [[[0.0, 0.0, 0.0]]])


def holds(ep):
    """-> list of dicts per hold: tcp (3), R_hand (3x3), F_on_peg (3), and episode constants."""
    tr = np.load(os.path.join(ep, "trace.npz"))
    t = tr["t"] + tr["t0"]
    ff, K, x = tr["ff"][:, 2], tr["K"][:, 2], tr["x"]
    tw, fw, _ = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))
    q = np.loadtxt(os.path.join(ep, "wrist_ft.csv"), delimiter=",", skiprows=1, ndmin=2)[:, 7:11]
    Rq = wrist_csv.hand_rotation(q)
    m = (np.abs(ff + 3.0) < 1e-6) & (K == 500.0)
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    out = []
    for a, b in zip(e[::2], e[1::2]):
        if not 0.15 < t[b - 1] - t[a] < 1.5:
            continue
        tare_m = (tw >= t[a] - 0.25) & (tw <= t[a])
        hold_m = (tw >= t[b - 1] - 0.3) & (tw <= t[b - 1])
        xs = (t >= t[b - 1] - 0.3) & (t <= t[b - 1])
        if tare_m.sum() < 3 or hold_m.sum() < 3 or xs.sum() < 3:
            continue
        F_franka = fw[hold_m].mean(0) - np.median(fw[tare_m], 0)  # force of the robot on the environment
        j = np.flatnonzero(hold_m)[len(np.flatnonzero(hold_m)) // 2]
        out.append({"tcp": x[xs].mean(0), "R": Rq[j], "F": -F_franka})  # force on the peg
    return out


def episode(ep):
    run = open(os.path.join(ep, "run.out")).read()
    hole = np.array([float(v) for v in re.search(r"HOLE opening at \[([^\]]*)\]", run).group(1).split()])
    off = np.array([float(v) for v in re.search(r"peg bottom \[([^\]]*)\] mm from the TCP", run).group(1).split()]) / 1000
    tip = float(re.search(r"tip ([\d.]+) mm below the TCP", run).group(1)) / 1000
    P = json.load(open(os.path.join(ep, "params.json")))
    mz = re.search(r"grasp height offset ([+-][\d.]+) mm", run)
    P["zcal"] = float(mz.group(1)) / 1000 if mz else 0.0
    return hole, off, tip, P


def evaluate(data, g, dxy=(0.0, 0.0)):
    pp = peg_points(g)
    rows = []
    for d in data:
        R0, R = d["R0"], d["R"]
        # grasp-time in-hand offset: world vector at the grasp (xy offset, tip below) carried in the hand frame
        p_hand = R0.T @ np.array([d["off"][0], d["off"][1], -d["tip"]])
        tipc = d["tcp"] + R @ p_hand - np.array([0.0, 0.0, d["zcal"] if args.diag_zcal else 0.0])
        up = -R[:, 2] if R[2, 2] < 0 else R[:, 2]  # the hand z axis points down the peg
        e1 = np.cross(up, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(up, e1)
        W = tipc + pp @ np.stack([e1, e2, up])
        rel = W - (d["hole"] + [dxy[0], dxy[1], 0.0])
        if d["tilt"]:  # hole block tilted about x through the opening (pivot assumed)
            c_, s_ = np.cos(-np.radians(d["tilt"])), np.sin(-np.radians(d["tilt"]))
            rel = rel @ np.array([[1, 0, 0], [0, c_, -s_], [0, s_, c_]]).T
        r = np.hypot(rel[:, 0], rel[:, 1])
        dist, nr, nz = profile_sdf(r, rel[:, 2], g)
        rr = np.maximum(r, 1e-12)
        n = np.stack([nr * rel[:, 0] / rr, nr * rel[:, 1] / rr, nz], 1)
        F = d["F"]
        c = dist < args.contact_gap
        meas = np.linalg.norm(F) > 1.0
        row = {"ep": d["ep"], "k": d["k"], "gap_min": float(dist.min()), "pred": bool(c.any()), "meas": bool(meas),
               "F": F}
        if args.diag_support > 0:
            c = dist < dist.min() + args.diag_support
        if meas and c.any():
            gens = []
            for ni in n[c]:
                t1 = np.cross(ni, [0, 0, 1.0]) if abs(ni[2]) < 0.9 else np.cross(ni, [1.0, 0, 0])
                t1 /= np.linalg.norm(t1); t2 = np.cross(ni, t1)
                for a_ in np.linspace(0, 2 * np.pi, N_FRIC, endpoint=False):
                    gens.append(ni + d["mu"] * (np.cos(a_) * t1 + np.sin(a_) * t2))
            A = np.array(gens).T
            lam, _ = nnls(A, F)
            Fh = A @ lam
            row["rel_res"] = float(np.linalg.norm(F - Fh) / np.linalg.norm(F))
            if np.linalg.norm(F[:2]) > 0.5:
                ca = F[:2] @ Fh[:2] / (np.linalg.norm(F[:2]) * max(np.linalg.norm(Fh[:2]), 1e-9))
                row["lat_deg"] = float(np.degrees(np.arccos(np.clip(ca, -1, 1))))
        rows.append(row)
    return rows


def summary(rows, label):
    m1 = np.mean([r["pred"] == r["meas"] for r in rows])
    rr = np.array([r["rel_res"] for r in rows if "rel_res" in r])
    la = np.array([r["lat_deg"] for r in rows if "lat_deg" in r])
    fn = sum(r["meas"] and not r["pred"] for r in rows); fp = sum(r["pred"] and not r["meas"] for r in rows)
    s = (f"{label:28s} n {len(rows):3d} | M1 {m1 * 100:5.1f} % (missed contacts {fn}, phantom {fp}) | "
         f"M2 med {np.median(rr) if len(rr) else np.nan:.3f} p90 {np.percentile(rr, 90) if len(rr) else np.nan:.3f} "
         f"(n {len(rr)}) | M3 med {np.median(la) if len(la) else np.nan:5.1f} deg (n {len(la)})")
    passed = m1 >= 0.90 and len(rr) and np.median(rr) <= 0.15 and np.percentile(rr, 90) <= 0.35 and len(la) and np.median(la) <= 20
    return s, bool(passed)


data = []
for ep in args.episodes:
    hole, off, tip, P = episode(ep)
    hs = holds(ep)
    if not hs:
        continue
    R0 = hs[0]["R"]
    for k, h in enumerate(hs):
        data.append({**h, "R0": R0, "hole": hole, "off": off, "tip": tip, "tilt": P["tilt"], "mu": P["friction"], "zcal": P["zcal"],
                     "ep": os.path.basename(ep), "k": k})
main = [d for d in data if not d["tilt"]]
tilted = [d for d in data if d["tilt"]]
rows = evaluate(main, G0)
s, ok = summary(rows, "main (untilted)")
print(s, "-> B1", "PASS" if ok else "FAIL")
print(summary(evaluate(tilted, G0), "tilted e002 / e003")[0])
# diagnostic (not a metric): missed contacts' min gap vs hold order (in-hand slip makes the grasp-time pose stale)
miss = [(r["k"], r["gap_min"] * 1000) for r in rows if r["meas"] and not r["pred"]]
if miss:
    k_, g_ = np.array(miss).T
    print(f"  diagnostic: {len(miss)} missed contacts, min gap median {np.median(g_):.2f} mm; first half of the holds "
          f"{np.median(g_[k_ < 20]) if (k_ < 20).any() else np.nan:.2f} mm, second half {np.median(g_[k_ >= 20]) if (k_ >= 20).any() else np.nan:.2f} mm")
if args.perturb:
    print("B1b sensitivity (main set, one perturbation at a time):")
    for name in ("R", "a", "c"):
        for dv in (-1e-3, -3e-4, -1e-4, 1e-4, 3e-4, 1e-3):
            g = dataclasses.replace(G0, **{name: getattr(G0, name) + dv})
            if g.c <= 0 or g.a >= g.R + 2e-3 or g.a <= g.pc:
                continue
            print(" ", summary(evaluate(main, g), f"{name} {dv * 1000:+.1f} mm")[0])
    for dx, dy in ((1e-4, 0), (3e-4, 0), (1e-3, 0), (0, 1e-3)):
        print(" ", summary(evaluate(main, G0, (dx, dy)), f"hole xy +({dx * 1000:.1f}, {dy * 1000:.1f}) mm")[0])
