"""H3-100 failure modes per arm (queue_h3_100.sh output: <arm>/rep<n>/<assembly>/{res.json,run.out}).
Classes, first match wins:
    inserted       res.json success
    asset_error    no res.json, the scene / demo could not use the asset (argument or Traceback in run.out)
    no_grasp       "FAIL: grasp" (fingers did not close on the plug)
    lost_plug      xy error > 20 mm or in-hand slip > 10 mm (the plug fell out of / turned in the fingers)
    search_failed  "probing: failed" (never engaged the hole)
    jammed         engaged, xy < 1.5 mm, but > 1 mm short of seated
    off_axis       at depth but xy >= 1.5 mm (beside the hole / on a feature of the socket)
    not_kept       seated with xy < 1.5 mm, but did not stay on release
    running        no res.json yet, no error
    python3 scripts/h3_100_classify.py ~/ci_mpc_runs/h3_100 [--list]
"""
import glob
import json
import os
import sys
from collections import Counter

import numpy as np


def classify(d):
    out = open(os.path.join(d, "run.out")).read() if os.path.exists(os.path.join(d, "run.out")) else ""
    f = os.path.join(d, "res.json")
    if not os.path.exists(f):
        if "FAIL: grasp" in out:
            return "no_grasp"
        if "error: argument" in out or "Traceback" in out or "[Error]" in out:
            return "asset_error"
        return "running"
    r = json.load(open(f))
    if r["success"]:
        return "inserted"
    slip = np.linalg.norm(r["peg_slip_hand_mm"]) if r.get("peg_slip_hand_mm") else 0.0
    if r["xy_err_mm"] > 20 or slip > 10:
        return "lost_plug"
    if "probing: failed" in out:
        return "search_failed"
    if r["depth_err_mm"] > 1.0:
        return "jammed" if r["xy_err_mm"] < 1.5 else "off_axis"
    return "off_axis" if r["xy_err_mm"] >= 1.5 else "not_kept"


root = sys.argv[1]
by_arm = {}
for d in sorted(glob.glob(os.path.join(root, "*", "rep*", "*"))):
    if os.path.isdir(d):
        arm, rep, asm = d.split(os.sep)[-3:]
        by_arm.setdefault(f"{arm} {rep}", []).append((asm, classify(d)))
for k, v in by_arm.items():
    c = Counter(cl for _, cl in v)
    print(f"{k}: {len(v)} runs  " + "  ".join(f"{cl} {n}" for cl, n in c.most_common()))
    if "--list" in sys.argv:
        for asm, cl in v:
            if cl != "inserted":
                print(f"    {asm} {cl}")
