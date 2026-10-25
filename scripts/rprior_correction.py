#!/usr/bin/env python3
# Add the distance-prior correction to a learned probe likelihood (ci_mpc_planner/probe_likelihood.py): the histogram
# log ratio of the hole-TCP distance r between the training positives (true holes, where the probes happened to be) and
# the negatives (hypotheses uniform on the +-6 mm belief grid), smoothed; saved as rcorr_edges / rcorr in a copy.
#   python3 scripts/rprior_correction.py DATA.npz MODEL.npz OUT.npz [--all-runs]
import sys
import numpy as np

data, model, out = sys.argv[1:4]
d = np.load(data)
keep = np.ones(len(d["run"]), bool) if "--all-runs" in sys.argv else d["calibrated"]
tcp, hole = d["tcp"][keep], d["hole"][keep]
rng = np.random.default_rng(0)
r_pos = np.linalg.norm(hole - tcp, axis=1)
h = rng.uniform(-6.0, 6.0, (len(tcp), 16, 2))
r_neg = np.linalg.norm(h - tcp[:, None], axis=2).ravel()
edges = np.arange(0.0, 15.01, 0.5)
p_pos = np.histogram(r_pos, edges)[0] + 1.0
p_neg = np.histogram(r_neg, edges)[0] + 1.0
p_pos, p_neg = p_pos / p_pos.sum(), p_neg / p_neg.sum()
rc = np.convolve(np.log(p_pos / p_neg), np.ones(3) / 3, mode="same")  # light smoothing
m = dict(np.load(model))
m["rcorr_edges"], m["rcorr"] = edges, rc - rc[np.searchsorted(edges, 6.0) - 1]  # 0 around 6 mm (constant is free)
np.savez(out, **m)
print("r mm  " + " ".join(f"{e:4.1f}" for e in edges[:-1:2]))
print("corr  " + " ".join(f"{v:4.1f}" for v in m["rcorr"][::2]))
