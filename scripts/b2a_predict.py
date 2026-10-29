"""CI-MPC stage B2a (pre-registered in PROGRESS 2026-10-29 16:10): forward prediction of the SDF plant vs Isaac,
translation only. Inputs per episode: trace6.npz (b2_trace.py: TCP pose, 6D setpoint / ff / K), peg_truth.npz (the
known in-hand pose: TCP -> peg transform per sample), wrist_ft.csv (true contact force), run.out (hole truth).
Plant (translational, orientation held at the measured one): x+ solves
    K_t (u - x+) + ff_t + f_arm(x+ - x) + F_contact(x+) = 0,  F_contact from the rounded-primitive SDF (sdf_mpc.py)
with f_arm = -F_s tanh(dx / d_eps) per axis. F_s, d_eps are fitted on the fit episodes' free-space samples (contacts
off, |F_wrist| < 0.3 N), then frozen. Samples at 10 Hz; one step = 0.1 s (setpoint at the end of the step); multi-step
= 6 steps. Baselines: (a) contacts off, (b) hole 0.5 / 2 mm off in x. Sets: transitional (a true-geometry contact with
|n_z| < 0.9 at gap < 0.2 mm) / flat-face.
    python3 scripts/b2a_predict.py --fit ~/ci_mpc_runs/b1v2/factory_700? --score ~/ci_mpc_runs/b2a/*
"""
import argparse
import os
import re
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ci_mpc_planner"))
import wrist_csv  # noqa: E402
from ci_mpc_planner.sdf_mpc import HoleSpec, PlantParams, hole_sdf  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--fit", nargs="+", required=True)
p.add_argument("--score", nargs="+", required=True)
args = p.parse_args()
DT, H = 0.1, 6
HOLES = {"factory": HoleSpec(R=0.0045, depth=0.025, chamfer=0.001),
         "dev": HoleSpec(R=0.00405, depth=0.015, chamfer=0.0, block_half=0.015)}
PEG = {"factory": (0.003993, 0.0005), "dev": (0.004, 0.0005)}  # radius, end chamfer
PRM = PlantParams()


def qrot(q):  # xyzw
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def peg_local(a, pc, n=24):
    """Peg points in the peg frame (bottom centre origin, +z up the peg)."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda r, z: np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(n, z)])  # noqa: E731
    return np.vstack([ring(a - pc, 0), ring(a, pc), ring(0.5 * (a - pc), 0)] + [ring(a, z) for z in (0.002, 0.004, 0.007, 0.01)]
                     + [[[0.0, 0.0, 0.0]]])


def load(ep):
    kind = "dev" if "dev" in os.path.basename(ep) else "factory"
    tr = np.load(os.path.join(ep, "trace6.npz"))
    t = tr["t"] + tr["t0"]
    pt = np.load(os.path.join(ep, "peg_truth.npz"))["log"]
    tw, fw, _ = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))
    hole = np.array([float(v) for v in re.search(r"HOLE opening at \[([^\]]*)\]", open(os.path.join(ep, "run.out")).read()).group(1).split()])
    S = []
    for tk in np.arange(t[0] + 1.0, t[-1] - DT * (H + 1), DT):
        i = np.searchsorted(t, tk)
        j = np.argmin(abs(pt[:, 0] - tk))
        if abs(pt[j, 0] - tk) > 0.06:
            continue
        R = qrot(tr["q_xyzw"][i])
        Rp = qrot([pt[j, 12], pt[j, 13], pt[j, 14], pt[j, 11]])  # part quat wxyz -> xyzw
        off = R.T @ (pt[j, 8:11] - pt[j, 1:4])                    # peg origin in the TCP frame
        S.append({"t": tk, "i": i, "x": tr["x"][i], "R": R, "Rrel": R.T @ Rp, "off": off})
    E = {"kind": kind, "tr": tr, "t": t, "tw": tw, "fw": fw, "tare": np.zeros(3), "hole": hole, "S": S}
    # tare: the wrist force while the geometry puts the peg > 3 mm from every surface (free air after the grasp, the
    # carried peg's weight included); collect-mode episodes have no hover time in res.json
    air = []
    for s_ in S[::5]:
        _, (phi, _) = forces(E, s_["x"], s_["R"], s_["Rrel"], s_["off"], 0.0)
        if phi.min() > 3e-3:
            m = (tw >= s_["t"] - 0.05) & (tw <= s_["t"] + 0.05)
            if m.any():
                air.append(fw[m].mean(0))
    E["tare"] = np.median(air, 0) if air else np.zeros(3)
    return E


def forces(E, x, R, Rrel, off, dxh, contacts=True):
    if not contacts:
        return np.zeros(3), None
    kind = E["kind"]
    W = x + R @ off + (peg_local(*PEG[kind]) @ (R @ Rrel).T)
    phi, n = hole_sdf(W - (E["hole"] + [dxh, 0, 0]), HOLES[kind])
    fn = PRM.k_c * PRM.eps * np.logaddexp(0.0, -phi / PRM.eps)
    return (fn[:, None] * n).sum(0), (phi, n)


def step(E, x0, u, K, ff, R, Rrel, off, Fs, de, dxh=0.0, contacts=True):
    def res(x):
        F, _ = forces(E, x, R, Rrel, off, dxh, contacts)
        return (K * (u - x) + ff - Fs * np.tanh((x - x0) / de) + F) / K
    return least_squares(res, x0, xtol=1e-9, ftol=1e-9, max_nfev=60).x


def rollout(E, s, n_steps, Fs, de, dxh=0.0, contacts=True):
    tr, t = E["tr"], E["t"]
    x = s["x"].copy()
    for k in range(1, n_steps + 1):
        i = np.searchsorted(t, s["t"] + k * DT)
        x = step(E, x, tr["cmd"][i], tr["K"][i, :3], tr["ff"][i], s["R"], s["Rrel"], s["off"], Fs, de, dxh, contacts)
    return x, np.searchsorted(t, s["t"] + n_steps * DT)


def true_force(E, tk):
    m = (E["tw"] >= tk - 0.05) & (E["tw"] <= tk + 0.05)
    return -(E["fw"][m].mean(0) - E["tare"]) if m.sum() else np.full(3, np.nan)


# --- fit the arm friction on the fit episodes (free-space samples, contacts off)
fit = [load(ep) for ep in args.fit]
free = [(E, s) for E in fit for s in E["S"] if np.linalg.norm(true_force(E, s["t"])) < 0.3][::3]


def fit_res(theta):
    Fs, de = np.abs(theta[:3]), abs(theta[3])
    out = []
    for E, s in free:
        x1, i1 = rollout(E, s, 1, Fs, de, contacts=False)
        out.append((x1 - E["tr"]["x"][i1]) * 1000)
    return np.concatenate(out)


sol = least_squares(fit_res, np.r_[PRM.F_s[:3], 5e-5], bounds=([0, 0, 0, 1e-6], [10, 10, 10, 1e-3]))
Fs, de = np.abs(sol.x[:3]), abs(sol.x[3])
print(f"arm friction fitted on {len(free)} free-space samples: F_s {np.round(Fs, 2)} N, d_eps {de * 1000:.3f} mm; "
      f"one-step RMS {np.sqrt(np.mean(sol.fun ** 2)):.3f} mm")

# --- score
for kind in ("factory", "dev"):
    eps_ = [load(ep) for ep in args.score if ("dev" in os.path.basename(ep)) == (kind == "dev")]
    if not eps_:
        continue
    rows = {"trans": [], "flat": []}
    for E in eps_:
        for s in E["S"][::2]:
            _, geo = forces(E, s["x"], s["R"], s["Rrel"], s["off"], 0.0)
            phi, n = geo
            c = phi < 2e-4
            if not c.any():
                continue
            cls = "trans" if (np.abs(n[c, 2]) < 0.9).any() else "flat"
            r = {}
            for name, kw in (("sdf", {}), ("nocontact", {"contacts": False}), ("wrong05", {"dxh": 0.0005}), ("wrong2", {"dxh": 0.002})):
                x1, i1 = rollout(E, s, 1, Fs, de, **kw)
                x6, i6 = rollout(E, s, H, Fs, de, **kw)
                F6 = forces(E, x6, s["R"], s["Rrel"], s["off"], kw.get("dxh", 0.0), kw.get("contacts", True))[0]
                r[name] = (np.linalg.norm(x1 - E["tr"]["x"][i1]) * 1000, np.linalg.norm(x6 - E["tr"]["x"][i6]) * 1000,
                           np.linalg.norm(F6 - true_force(E, E["t"][i6])))
            rows[cls].append(r)
    for cls, rs in rows.items():
        if not rs:
            continue
        med = {k: np.nanmedian([r[k] for r in rs], 0) for k in rs[0]}
        line = " | ".join(f"{k}: 1-step {v[0]:.3f} mm, 6-step {v[1]:.3f} mm, F6 {v[2]:.2f} N" for k, v in med.items())
        print(f"{kind} {cls} (n {len(rs)}): {line}")
        if cls == "trans":
            s_, a_, b_ = med["sdf"], med["nocontact"], med["wrong05"]
            ok = (len(rs) >= 50 and s_[0] <= 0.7 * a_[0] and s_[0] <= 0.7 * b_[0] and s_[1] <= 0.7 * a_[1]
                  and s_[1] <= 0.7 * b_[1] and s_[2] <= 0.7 * b_[2])
            print(f"   -> B2a {kind}: {'PASS' if ok else ('UNTESTABLE' if len(rs) < 50 else 'FAIL')} "
                  f"(ratios vs no-contact 1-step {s_[0] / a_[0]:.2f}, 6-step {s_[1] / a_[1]:.2f}; vs wrong 0.5 mm 1-step "
                  f"{s_[0] / b_[0]:.2f}, 6-step {s_[1] / b_[1]:.2f}, F6 {s_[2] / b_[2]:.2f})")
