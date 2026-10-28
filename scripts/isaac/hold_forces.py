"""Per probe hold of a pipeline probe episode: true force through the hand (wrist F/T) vs the controller observer
(the force the demo logs and the likelihood used), both tared over the 0.25 s before the press, mean over the last
0.3 s of the hold (vectorized step 4c/4d). Needs the episode's bag trace (bag_trace.py -> trace.npz) and wrist_ft.csv
(WRIST_FT=1). Holds = command segments with the probe's stiction feedforward (-3 N) at the search stiffness (z 500).
    python3 scripts/isaac/hold_forces.py EP_DIR [EP_DIR ...] [--out x.npz]
"""
import argparse
import os

import numpy as np

p = argparse.ArgumentParser()
p.add_argument("dirs", nargs="+")
p.add_argument("--out", default="")
p.add_argument("--lag", type=float, default=None, help="wrist CSV lag [s] (default: estimated per episode)")
args = p.parse_args()

rows = []
for d in args.dirs:
    tr = np.load(os.path.join(d, "trace.npz"))
    t = tr["t"] + tr["t0"]
    ff, K, w = tr["ff"][:, 2], tr["K"][:, 2], tr["wrench"]
    wf = np.loadtxt(os.path.join(d, "wrist_ft.csv"), delimiter=",", skiprows=1)
    # wrist_ft.csv: raw force in the hand frame + the hand quaternion (w, x, y, z); world = R_hand raw is the Franka
    # sign (force on the env, PROGRESS "Ground truth for contact"), as the observer's external_wrench
    w_, x_, y_, z_ = (wf[:, i] for i in (7, 8, 9, 10))
    R = np.stack([np.stack([1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - w_ * z_), 2 * (x_ * z_ + w_ * y_)], -1),
                  np.stack([2 * (x_ * y_ + w_ * z_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - w_ * x_)], -1),
                  np.stack([2 * (x_ * z_ - w_ * y_), 2 * (y_ * z_ + w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)], -1)], 1)
    fw = np.einsum("nij,nj->ni", R, wf[:, 1:4])
    # the wrist CSV's time stamps are ~0.23 s ahead of the joint-state stamps (wrist_csv.py): align by the
    # lag that best correlates the observer Fz with the wrist Fz (the observer's own lag, 1 / gain 33 ms, is in it)
    lags = np.arange(-0.1, 0.5, 0.005)
    cor = [np.corrcoef(w[:, 2], np.interp(t + lg, wf[:, 0], fw[:, 2]))[0, 1] for lg in lags]
    lag = lags[int(np.argmax(cor))] if args.lag is None else args.lag
    true = np.stack([np.interp(t + lag, wf[:, 0], fw[:, i]) for i in range(3)], 1)
    m = (np.abs(ff + 3.0) < 1e-6) & (K == 500.0)
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    segs = [(a, b) for a, b in zip(e[::2], e[1::2]) if 0.15 < t[b - 1] - t[a] < 1.5]
    for k, (a, b) in enumerate(segs):
        pre, hold = slice(max(a - 60, 0), a), slice(max(b - 72, a), b)
        obs = -(w[hold].mean(0) - w[pre].mean(0))  # the demo's sign: force on the peg = -(Franka sign)
        tru = -(true[hold].mean(0) - true[pre].mean(0))
        rows.append((len(rows), k, *obs, *tru))
    print(f"{d}: {len(segs)} holds, wrist lag {lag:.3f} s")
R = np.array(rows)
obs, tru = R[:, 2:5], R[:, 5:8]
print(f"holds {len(R)}: Fz observer median {np.median(obs[:, 2]):.2f}, true {np.median(tru[:, 2]):.2f}, "
      f"observer - true {np.mean(obs[:, 2] - tru[:, 2]):+.2f} +- {np.std(obs[:, 2] - tru[:, 2]):.2f} N; "
      f"lateral |F| observer {np.median(np.linalg.norm(obs[:, :2], axis=1)):.2f}, true "
      f"{np.median(np.linalg.norm(tru[:, :2], axis=1)):.2f} N")
if args.out:
    np.savez(args.out, k=R[:, 1].astype(int), obs=obs, true=tru)
