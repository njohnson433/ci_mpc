#!/usr/bin/env python3
# Probe measurements from the Isaac peg logs (run.out) for the learned probe likelihood (ci_mpc_planner/
# probe_likelihood.py): per probe the TCP xy and the true hole xy (model frame, relative to the perception estimate,
# mm), the tip height (mm, grasp-offset corrected as the run saw it) and the tared contact force (N, from the
# controller's joint-torque wrench estimate, like the FR3's O_F_ext_hat_K). Factory peg only (runs with --geo, i.e.
# gear / AutoMate, use other geometry and are skipped).
#   python3 scripts/probe_dataset.py OUT.npz [ROOT ...]      (default root ~/ci_mpc_runs)
import glob
import os
import re
import sys

import numpy as np

out = sys.argv[1]
roots = sys.argv[2:] or [os.path.expanduser("~/ci_mpc_runs")]
VEC = r"\[([^\]]*)\]"
PROBE = re.compile(rf"^  probe (\d+): TCP {VEC} mm \(true hole {VEC}\), tip ([-+0-9.]+) mm, force {VEC} N")

rows, runs = [], []
for root in roots:
    for path in sorted(glob.glob(os.path.join(root, "**", "run.out"), recursive=True)):
        with open(path, errors="ignore") as f:
            lines = f.readlines()
        if not lines or "peg-in-hole demo" not in lines[0] or "--geo" in lines[0]:
            continue
        calibrated = any("grasp height offset" in ln for ln in lines)
        run_rows = []
        for ln in lines:
            m = PROBE.match(ln)
            if m:
                k = int(m.group(1))
                tcp, hole, force = (np.array(m.group(i).split(), float) for i in (2, 3, 5))
                run_rows.append([len(runs), k, *tcp, *hole, float(m.group(4)), *force, calibrated])
        if run_rows:
            rows += run_rows
            runs.append(os.path.relpath(path, os.path.expanduser("~")))
d = np.array(rows)
np.savez(out, run=d[:, 0].astype(int), k=d[:, 1].astype(int), tcp=d[:, 2:4], hole=d[:, 4:6], tip=d[:, 6],
         force=d[:, 7:10], calibrated=d[:, 10].astype(bool), runs=np.array(runs))
print(f"{len(d)} probes from {len(runs)} runs ({d[:, 10].mean() * 100:.0f} % calibrated) -> {out}")
rel = np.linalg.norm(d[:, 4:6] - d[:, 2:4], axis=1)
print("hole-TCP distance mm: " + ", ".join(f"<{b}: {np.mean(rel < b) * 100:.0f} %" for b in (1, 2, 4, 6, 9)))
