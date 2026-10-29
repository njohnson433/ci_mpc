"""CI-MPC stage B2a v2 (pre-registered in PROGRESS before scoring): forward prediction with the dynamic arm model.
Arm (b2a0_arm_dynamics.py --lam 1): Lambda(q) x'' = clip(K (u - x) - D (v - v_d) - I + ff, 18 N) + F_fc + b - F_s tanh(v/v_eps)
  + F_contact(x), with Lambda(q), the controller damping D, its integrator I and friction compensation F_fc from the
  trace (b2_trace.py), F_s and b fitted on the fit episodes' free space only. Contact: the rounded-primitive SDF
  (sdf_mpc.py, rho 20 um), softplus normal force; orientation held at the measured one, the peg from the known in-hand
  pose (peg_truth.npz). Sub-stepped at the trace rate (240 Hz) over 0.1 s (one step) and 0.6 s (6 steps).
Baselines: contacts off; hole 0.5 / 2 mm off in x. Sets: transitional / flat-face as B2a.
    python3 scripts/b2a2_predict.py --fit EP... --score EP...
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
V_EPS, STEP, H = 2e-3, 0.1, 6
HOLES = {"factory": HoleSpec(R=0.0045, depth=0.025, chamfer=0.001),
         "dev": HoleSpec(R=0.00405, depth=0.015, chamfer=0.0, block_half=0.015)}
PEG = {"factory": (0.003993, 0.0005), "dev": (0.004, 0.0005)}
PRM = PlantParams()


def qrot(q):  # xyzw
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def peg_local(a, pc, n=24):
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda r, z: np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(n, z)])  # noqa: E731
    return np.vstack([ring(a - pc, 0), ring(a, pc), ring(0.5 * (a - pc), 0)] + [ring(a, z) for z in (0.002, 0.004, 0.007, 0.01)]
                     + [[[0.0, 0.0, 0.0]]])


def load(ep):
    kind = "dev" if "dev" in os.path.basename(ep) else "factory"
    tr = np.load(os.path.join(ep, "trace6.npz"))
    t = tr["t"] + tr["t0"]
    dt = np.median(np.diff(t))
    v = np.gradient(tr["x"], t, axis=0)
    I = np.zeros_like(tr["x"])  # the controller integrator, reconstructed (b2a0_arm_dynamics.py)
    for k in range(1, len(t)):
        e = tr["x"][k - 1] - tr["cmd"][k - 1]
        en = tr["integral"][k - 1] > 0.5
        acc = en & (np.linalg.norm(v[k - 1]) < 0.01) & (np.abs(e) < 0.003)
        I[k] = np.clip(np.where(acc, I[k - 1] + 3000.0 * e * dt, I[k - 1] * np.exp(-(2.0 if en else 5.0) * dt)), -8, 8)
    Dc = 2.0 * np.sqrt(np.maximum(tr["K"][:, :3], 0)) * np.sqrt(np.maximum(tr["lambda_ctrl_diag"][:, :3], 0))
    D = np.where(tr["D"][:, :3] > 0, tr["D"][:, :3], Dc)
    tw, fw, _ = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))
    pt = np.load(os.path.join(ep, "peg_truth.npz"))["log"]
    run = os.path.join(ep, "run.out")
    hole = np.array([float(v_) for v_ in re.search(r"HOLE opening at \[([^\]]*)\]", open(run).read()).group(1).split()])
    E = {"kind": kind, "t": t, "dt": dt, "x": tr["x"], "v": v, "u": tr["cmd"], "vd": tr["twist_d"][:, :3], "ff": tr["ff"],
         "K": tr["K"][:, :3], "D": D, "I": I, "Ffc": tr["F_fc"][:, :3], "Lam": tr["Lambda_t"], "q": tr["q_xyzw"],
         "tw": tw, "fw": fw, "pt": pt, "hole": hole, "n": int(round(STEP / dt))}
    return E


def peg_geom(E, i):
    """-> (R_tcp, R_rel, off) of the peg at trace index i (known in-hand pose)."""
    j = np.argmin(abs(E["pt"][:, 0] - E["t"][i]))
    R = qrot(E["q"][i])
    Rp = qrot([E["pt"][j, 12], E["pt"][j, 13], E["pt"][j, 14], E["pt"][j, 11]])
    return R, R.T @ Rp, R.T @ (E["pt"][j, 8:11] - E["pt"][j, 1:4])


def contact(E, x, g, dxh):
    R, Rrel, off = g
    W = x + R @ off + peg_local(*PEG[E["kind"]]) @ (R @ Rrel).T
    phi, n = hole_sdf(W - (E["hole"] + [dxh, 0, 0]), HOLES[E["kind"]])
    fn = PRM.k_c * PRM.eps * np.logaddexp(0.0, -phi / PRM.eps)
    return (fn[:, None] * n).sum(0), phi, n


def simulate(E, i, steps, Fs, b, contacts=True, dxh=0.0):
    x, v, dt = E["x"][i].copy(), E["v"][i].copy(), E["dt"]
    g = peg_geom(E, i)
    for k in range(i, i + steps * E["n"]):
        F = np.clip(E["K"][k] * (E["u"][k] - x) - E["D"][k] * (v - E["vd"][k]) - E["I"][k] + E["ff"][k], -18.0, 18.0)
        F = F + E["Ffc"][k] + b - Fs * np.tanh(v / V_EPS)
        if contacts:
            F = F + contact(E, x, g, dxh)[0]
        v = v + np.linalg.solve(E["Lam"][k], F) * dt
        x = x + v * dt
    return x, i + steps * E["n"]


def wrist_F(E, i):
    tk = E["t"][i]
    m = (E["tw"] >= tk - 0.05) & (E["tw"] <= tk + 0.05)
    return -(E["fw"][m].mean(0) - E["tare"]) if m.sum() else np.full(3, np.nan)


# --- arm friction / bias on the fit episodes' free space (contacts off; |wrist F| < 0.3 N over the window)
fit = [load(ep) for ep in args.fit]
for E in fit + []:
    E["tare"] = np.median(E["fw"][: len(E["fw"]) // 10], 0)
starts = []
for E in fit:
    Fw = np.stack([np.interp(E["t"], E["tw"], E["fw"][:, k]) for k in range(3)], 1) - E["tare"]
    free = np.linalg.norm(Fw, axis=1) < 0.3
    starts += [(E, i) for i in range(5, len(E["t"]) - E["n"] - 1, E["n"]) if free[i:i + E["n"] + 1].all()]
sol = least_squares(lambda th: np.concatenate([(simulate(E, i, 1, np.abs(th[:3]), th[3:], contacts=False)[0]
                                                - E["x"][i + E["n"]]) * 1000 for E, i in starts]),
                    np.r_[1.0, 1.0, 1.0, 0, 0, 0], bounds=([0, 0, 0, -10, -10, -10], [20, 20, 20, 10, 10, 10]))
Fs, b = np.abs(sol.x[:3]), sol.x[3:]
print(f"arm fit on {len(starts)} free windows: F_s {np.round(Fs, 2)} N, b {np.round(b, 2)} N, one-step RMS "
      f"{np.round(np.sqrt((sol.fun.reshape(-1, 3) ** 2).mean(0)), 3)} mm")

# --- score
for kind in ("factory", "dev"):
    eps_ = [load(ep) for ep in args.score if ("dev" in os.path.basename(ep)) == (kind == "dev")]
    rows = {"trans": [], "flat": []}
    for E in eps_:
        air = []  # tare: the wrist force while the geometry puts the peg > 3 mm from every surface
        for i in range(0, len(E["t"]), 24):
            _, phi, _ = contact(E, E["x"][i], peg_geom(E, i), 0.0)
            if phi.min() > 3e-3:
                m = (E["tw"] >= E["t"][i] - 0.05) & (E["tw"] <= E["t"][i] + 0.05)
                if m.any():
                    air.append(E["fw"][m].mean(0))
        E["tare"] = np.median(air, 0) if air else np.zeros(3)
        for i in range(24, len(E["t"]) - H * E["n"] - 2, 48):  # samples every 0.2 s
            _, phi, n = contact(E, E["x"][i], peg_geom(E, i), 0.0)
            c = phi < 2e-4
            if not c.any():
                continue
            cls = "trans" if (np.abs(n[c, 2]) < 0.9).any() else "flat"
            r = {}
            for name, kw in (("sdf", {}), ("nocontact", {"contacts": False}), ("wrong05", {"dxh": 0.0005}), ("wrong2", {"dxh": 0.002})):
                x1, i1 = simulate(E, i, 1, Fs, b, **kw)
                x6, i6 = simulate(E, i, H, Fs, b, **kw)
                F6 = contact(E, x6, peg_geom(E, i), kw.get("dxh", 0.0))[0] if kw.get("contacts", True) else np.zeros(3)
                r[name] = (np.linalg.norm(x1 - E["x"][i1]) * 1000, np.linalg.norm(x6 - E["x"][i6]) * 1000, np.linalg.norm(F6 - wrist_F(E, i6)))
            rows[cls].append(r)
    for cls, rs in rows.items():
        if not rs:
            continue
        med = {k: np.nanmedian([r[k] for r in rs], 0) for k in rs[0]}
        print(f"{kind} {cls} (n {len(rs)}): " + " | ".join(f"{k}: 1-step {v_[0]:.3f} mm, 6-step {v_[1]:.3f} mm, F6 {v_[2]:.2f} N" for k, v_ in med.items()))
        if cls == "trans":
            s_, a_, w_ = med["sdf"], med["nocontact"], med["wrong05"]
            ok = (len(rs) >= 50 and s_[0] <= 0.7 * a_[0] and s_[0] <= 0.7 * w_[0] and s_[1] <= 0.7 * a_[1]
                  and s_[1] <= 0.7 * w_[1] and s_[2] <= 0.7 * w_[2])
            print(f"   -> B2a v2 {kind}: {'PASS' if ok else ('UNTESTABLE' if len(rs) < 50 else 'FAIL')} (vs no-contact "
                  f"{s_[0] / a_[0]:.2f} / {s_[1] / a_[1]:.2f}; vs wrong 0.5 mm {s_[0] / w_[0]:.2f} / {s_[1] / w_[1]:.2f}, F6 {s_[2] / w_[2]:.2f})")
