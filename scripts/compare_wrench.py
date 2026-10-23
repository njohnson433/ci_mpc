#!/usr/bin/env python3
# Compare the controller's external wrench estimate (external_wrench.npy from peg_insert_demo.py, joint_states clock)
# with the simulated wrist F/T sensor (wrist_ft.csv, sim clock; WRIST_FT=1). Both in Franka's sign convention
# (force of the robot on the environment), force at the TCP. The two clocks differ by a constant (Isaac stamps
# /joint_states ~0.24 s behind /clock); found by cross-correlation.
#   compare_wrench.py LOG_DIR [--probe]   (--probe: also the probe windows from run.out)
import re
import sys

import numpy as np

D = sys.argv[1].rstrip("/") + "/"
ft = np.loadtxt(D + "wrist_ft.csv", delimiter=",", skiprows=1)
w = np.load(D + "external_wrench.npy")


def rot(q):
    qw, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - qw * z), 2 * (x * z + qw * y)],
                     [2 * (x * y + qw * z), 1 - 2 * (x * x + z * z), 2 * (y * z - qw * x)],
                     [2 * (x * z - qw * y), 2 * (y * z + qw * x), 1 - 2 * (x * x + y * y)]])


F = np.array([rot(r[7:11]) @ r[1:4] for r in ft])  # Franka sign, world frame (calibrated with known wrenches)
t_ft = ft[:, 0]
best = None
for off in np.arange(-1.0, 1.0, 0.005):
    tt = t_ft + off
    sel = (tt > w[0, 0]) & (tt < w[-1, 0])
    if sel.sum() < 100:
        continue
    e = np.mean((np.interp(tt[sel], w[:, 0], w[:, 3]) - F[sel, 2]) ** 2)
    if best is None or e < best[0]:
        best = (e, off)
off = best[1]
tt = t_ft + off
sel = (tt > w[0, 0]) & (tt < w[-1, 0])
W = np.column_stack([np.interp(tt[sel], w[:, 0], w[:, k]) for k in (1, 2, 3)])
Ft = F[sel]
contact = np.linalg.norm(Ft, axis=1) > 0.5
E = W - Ft
print(f"clock offset {off:+.3f} s; samples {sel.sum()} ({contact.sum()} with |F| > 0.5 N)")
for name, m in (("contact", contact), ("free", ~contact)):
    if m.any():
        print(f"  {name:8s} error mean {np.round(E[m].mean(0), 2)} rms {np.round(np.sqrt((E[m] ** 2).mean(0)), 2)} N "
              f"(true force rms {np.round(np.sqrt((Ft[m] ** 2).mean(0)), 2)}), max |error| {np.linalg.norm(E[m], axis=1).max():.2f} N")
if "--probe" in sys.argv:
    for line in open(D + "run.out"):
        m = re.search(r"probe \[(.*)\] from ([\d.]+) to ([\d.]+)", line)
        if m:
            a, b = float(m.group(2)), float(m.group(3))
            s2 = (tt > a) & (tt < b)
            s3 = (w[:, 0] > a) & (w[:, 0] < b)
            print(f"  probe ff [{m.group(1)}]: wrist F/T {np.round(F[s2].mean(0), 2)}  estimate {np.round(w[s3, 1:].mean(0), 2)}")
