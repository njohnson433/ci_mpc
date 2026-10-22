#!/usr/bin/env python3
# Fits joint friction and effective armature from a joint_id_controller log (run_joint_id.py output).
#   fit_joint_id.py id.npz [--urdf fr3.urdf] [--out fitted.yaml] [--truth fr3_joint_params_sim.yaml]
# Residual torque per joint: r = tau_cmd - RNEA(q, dq, ddq) with gravity off (the FR3 compensates gravity
# itself, the sim has robot gravity off), so r contains only friction and the inertia the URDF lacks.
#   sweep:     samples at constant velocity (|ddq| < --max-acc) -> r = coulomb*sign(v) + viscous*v + offset
#   breakaway: commanded torque when the joint starts moving -> static (and offset)
#   sine:      r - friction(v) = armature * ddq                                       (least squares)
# Selecting sweep samples by measured acceleration matters: on the heavy joints the 0.8 rad/s passes are
# still accelerating inside the reference's cruise window (armature is not in the URDF model), and those
# samples biased the viscous term by 10-23 % in sim.
import argparse
import os

import numpy as np
import pinocchio as pin
import yaml
from scipy.signal import savgol_filter

T, JOINT, PHASE, CODE = 0, 1, 2, 3
Q, DQ, TAU = slice(4, 11), slice(11, 18), slice(18, 25)
SWEEP, BREAKAWAY, SINE = 1, 2, 3
ARM = [f"fr3_joint{i}" for i in range(1, 8)]

p = argparse.ArgumentParser()
p.add_argument("log")
p.add_argument("--urdf", default="/home/njohnson/ci_mpc_assets/fr3.urdf")
p.add_argument("--out", default="")
p.add_argument("--truth", default="", help="parameter YAML to compare against (sim validation)")
p.add_argument("--min-speed", type=float, default=0.05, help="sine fit: skip |dq| below this [rad/s]")
p.add_argument("--max-acc", type=float, default=0.05, help="sweep fit: max |ddq| of a constant-velocity sample [rad/s^2]")
p.add_argument("--onset-vel", type=float, default=1e-3, help="breakaway: motion onset velocity [rad/s]")
args = p.parse_args()

log = np.load(args.log)["log"]
t = log[:, T]
dt = np.median(np.diff(t))

# joint accelerations: smoothed derivative of the measured velocity (window ~25 ms)
win = max(5, int(round(0.025 / dt)) | 1)
ddq = savgol_filter(log[:, DQ], win, 2, deriv=1, delta=dt, axis=0)

full = pin.buildModelFromUrdf(args.urdf)
model = pin.buildReducedModel(full, [j for j in range(1, full.njoints) if full.names[j] not in ARM],
                              pin.neutral(full))
model.gravity.setZero()
data = model.createData()
tau_model = np.array([pin.rnea(model, data, q, v, a).copy() for q, v, a in zip(log[:, Q], log[:, DQ], ddq)])
resid = log[:, TAU] - tau_model

truth = yaml.safe_load(open(args.truth))["joints"] if args.truth else None
fitted = {}
for j in range(7):
    name = ARM[j]
    sel = log[:, JOINT] == j + 1
    if not sel.any():
        continue
    res = {}

    # sweep: one point per constant-velocity window (first 20 % dropped as transient)
    pts = []
    for code in sorted(set(log[sel & (log[:, PHASE] == SWEEP), CODE]) - {0}):
        idx = np.flatnonzero(sel & (log[:, PHASE] == SWEEP) & (log[:, CODE] == code))
        idx = idx[np.abs(ddq[idx, j]) < args.max_acc]
        if len(idx) > 5:
            pts.append((log[idx, DQ.start + j].mean(), resid[idx, j].mean(), resid[idx, j].std()))
    if len(pts) >= 3:
        v = np.array([x[0] for x in pts])
        r = np.array([x[1] for x in pts])
        A = np.column_stack([np.sign(v), v, np.ones_like(v)])
        coef, *_ = np.linalg.lstsq(A, r, rcond=None)
        res.update(coulomb=float(coef[0]), viscous=float(coef[1]), offset=float(coef[2]),
                   sweep_rms=float(np.sqrt(np.mean((A @ coef - r) ** 2))))

    # breakaway: commanded torque at motion onset = first sample after which |dq| stays above
    # --onset-vel (the controller only stops the ramp once the joint has clearly moved, which is later)
    brk = {}
    for code in (1, -1):
        idx = np.flatnonzero(sel & (log[:, PHASE] == BREAKAWAY) & (log[:, CODE] == code))
        if len(idx):
            moving = np.abs(log[idx, DQ.start + j]) > args.onset_vel
            still = np.flatnonzero(~moving)
            k = still[-1] + 1 if len(still) else 0
            brk[code] = log[idx[min(k, len(idx) - 1)], TAU.start + j]
    if len(brk) == 2:
        res.update(static=float((brk[1] - brk[-1]) / 2), static_offset=float((brk[1] + brk[-1]) / 2))

    # sine: armature from the residual after removing the fitted friction
    idx = np.flatnonzero(sel & (log[:, PHASE] == SINE) & (np.abs(log[:, DQ.start + j]) > args.min_speed))
    if len(idx) > 20 and "coulomb" in res:
        vj = log[idx, DQ.start + j]
        fr = res["coulomb"] * np.sign(vj) + res["viscous"] * vj + res["offset"]
        y = resid[idx, j] - fr
        a = ddq[idx, j]
        res["armature"] = float(np.dot(a, y) / np.dot(a, a))
        res["sine_rms"] = float(np.sqrt(np.mean((y - res["armature"] * a) ** 2)))
    fitted[name] = res

# report
cols = ["static", "coulomb", "viscous", "offset", "armature"]
print(f"log: {len(log)} rows, {t[-1] - t[0]:.1f} s sim, dt {dt * 1000:.2f} ms")
print(f"{'joint':12s}" + "".join(f"{c:>10s}" for c in cols) + f"{'sweep rms':>11s}{'sine rms':>10s}")
for name, res in fitted.items():
    print(f"{name:12s}" + "".join(f"{res.get(c, float('nan')):10.3f}" for c in cols)
          + f"{res.get('sweep_rms', float('nan')):11.3f}{res.get('sine_rms', float('nan')):10.3f}")
    if truth:
        ref = truth[name]
        print(f"{'  truth':12s}" + "".join(f"{ref.get(c, float('nan')):10.3f}" for c in cols))
        err = [abs(res.get(c, np.nan) - ref[c]) / abs(ref[c]) * 100 if ref.get(c) else np.nan for c in cols]
        print(f"{'  error %':12s}" + "".join(f"{e:10.1f}" for e in err))

if args.out:
    out = {"source": f"fit_joint_id.py {os.path.basename(args.log)}", "joints": {}}
    for name, res in fitted.items():
        out["joints"][name] = {c: round(res[c], 5) for c in cols if c in res}
    yaml.safe_dump(out, open(args.out, "w"), sort_keys=False)
    print("written", args.out)
