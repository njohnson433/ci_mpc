"""CI-MPC stage B2a-0 (to be pre-registered in PROGRESS before scoring): the arm's free-space dynamics alone, as the
base the contact model sits on (src-5e: validate the arm model before scoring geometry).
Per Cartesian translation axis, from trace6.npz (240 Hz):
    M x'' = K (u - x) + D (u' - x') + ff - F_s tanh(x' / v_eps)
integrated (semi-implicit Euler at the trace rate) over 0.1 s from the logged x, x' with the logged u(t), ff(t), K(t);
D from the log when non-zero, else fitted (the controller derives it from K). Fitted: M, D (if needed), F_s per axis,
on free-space segments of the fit episodes (wrist |F| < 0.3 N, the integral flag logged), then scored on the free-space
segments of other episodes.
    python3 scripts/b2a0_arm_dynamics.py --fit EP... --score EP...
"""
import argparse
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wrist_csv  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--fit", nargs="+", required=True)
p.add_argument("--score", nargs="+", required=True)
p.add_argument("--horizon", type=float, default=0.1)
p.add_argument("--integrator", type=int, default=1, help="include the reconstructed controller integrator (1) or not (0)")
p.add_argument("--fc", type=int, default=1, help="include the controller's velocity-mode friction compensation (1) or not (0)")
p.add_argument("--scene-fric", default="cart", choices=["cart", "joint"], help="scene friction: Cartesian tanh (F_s) or "
               "the joint-space form of the compensation scaled by F_s")
args = p.parse_args()
V_EPS = 2e-3  # [m/s]


def segments(ep):
    tr = np.load(os.path.join(ep, "trace6.npz"))
    t = tr["t"] + tr["t0"]
    tw, fw, _ = wrist_csv.load(os.path.join(ep, "wrist_ft.csv"))
    Fw = np.stack([np.interp(t, tw, fw[:, k]) for k in range(3)], 1)
    Fw -= np.median(Fw[: len(Fw) // 10], 0)
    free = np.linalg.norm(Fw, axis=1) < 0.3
    dt = np.median(np.diff(t))
    n = int(round(args.horizon / dt))
    v = np.gradient(tr["x"], t, axis=0)
    ud = np.gradient(tr["cmd"], t, axis=0)
    # the controller's integrator (cartesian_impedance_controller.cpp update): I += w e dt when the integral is enabled,
    # the EE is slower than 1 cm/s and |e_i| < 3 mm (not while saturated, ignored here), else it decays (leak 2 / s, or
    # 5 / s while the command disables it); clamped to +-8 N. Deterministic given the logged pose, setpoint and flag:
    # reconstructed over the whole trace and used as a known input. (MPC commands bypass the setpoint filter: the
    # callback sets position_d_ and position_d_target_ alike.)
    I = np.zeros_like(tr["x"])
    w, win, lim = 3000.0, 0.003, 8.0
    for k in range(1, len(t)):
        e = tr["x"][k - 1] - tr["cmd"][k - 1]
        en = tr["integral"][k - 1] > 0.5
        slow = np.linalg.norm(v[k - 1]) < 0.01
        acc = en & slow & (np.abs(e) < win)
        decay = np.exp(-(2.0 if en else 5.0) * dt)
        I[k] = np.clip(np.where(acc, I[k - 1] + w * e * dt, I[k - 1] * decay), -lim, lim)
    starts = [i for i in range(5, len(t) - n - 1, max(1, n // 2)) if free[i:i + n + 1].all()]
    Ffc = tr["F_fc"][:, :3] if (args.fc and "F_fc" in tr.files) else np.zeros_like(tr["x"])
    return {"t": t, "x": tr["x"], "v": v, "u": tr["cmd"], "ud": tr["twist_d"][:, :3], "I": I if args.integrator else 0 * I, "Ffc": Ffc, "ff": tr["ff"], "K": tr["K"][:, :3],
            "D": tr["D"][:, :3], "dt": dt, "n": n, "starts": starts}


def simulate(S, i, ax, M, D, Fs, b=0.0):
    x, v, dt = S["x"][i, ax], S["v"][i, ax], S["dt"]
    for k in range(i, i + S["n"]):
        Dk = S["D"][k, ax] if S["D"][k, ax] > 0 else D
        F_imp = np.clip(S["K"][k, ax] * (S["u"][k, ax] - x) - Dk * (v - S["ud"][k, ax]) - S["I"][k, ax] + S["ff"][k, ax], -18.0, 18.0)
        if args.scene_fric == "joint":  # the scene's joint friction in the same joint-space form, scale Fs (~1 = cancels the compensation)
            a = (F_imp + (1.0 - Fs) * S["Ffc"][k, ax] + b) / M
        else:
            a = (F_imp + S["Ffc"][k, ax] + b - Fs * np.tanh(v / V_EPS)) / M  # + the controller's friction compensation (known)
        v += a * dt
        x += v * dt
    return x


def errors(Ss, theta, ax):
    M, D, Fs = np.abs(theta[:3])
    b = theta[3] if len(theta) > 3 else 0.0  # constant force bias (the free-space integrator / residual gravity)
    return np.array([(simulate(S, i, ax, M, D, Fs, b) - S["x"][i + S["n"], ax]) * 1000 for S in Ss for i in S["starts"]])


fit = [segments(ep) for ep in args.fit]
score = [segments(ep) for ep in args.score]
print(f"free-space windows: fit {sum(len(S['starts']) for S in fit)}, score {sum(len(S['starts']) for S in score)}; "
      f"horizon {args.horizon * 1000:.0f} ms")
for ax, name in enumerate("xyz"):
    sol = least_squares(lambda th: errors(fit, th, ax), [3.0, 80.0, 1.0, 0.0], bounds=([0.05, 0, 0, -10], [200, 4000, 20, 10]))
    e_fit, e_sc = sol.fun, errors(score, sol.x, ax)
    zero = np.array([(S["x"][i, ax] - S["x"][i + S["n"], ax]) * 1000 for S in score for i in S["starts"]])  # hold baseline
    M, D, Fs = np.abs(sol.x[:3])
    print(f"{name}: M {M:.2f} kg, D {D:.0f} Ns/m (when not logged), F_s {Fs:.2f} N, bias {sol.x[3]:+.2f} N | one-step RMS fit {np.sqrt(np.mean(e_fit ** 2)):.3f} mm, "
          f"score {np.sqrt(np.mean(e_sc ** 2)):.3f} mm (hold-still baseline {np.sqrt(np.mean(zero ** 2)):.3f} mm)")
