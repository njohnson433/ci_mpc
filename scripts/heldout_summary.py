"""Per task and arm summary of a held-out queue, with the in-hand peg slip (res.json "peg_slip_hand_mm": peg bottom in
the TCP frame at the end minus at the grasp, sim truth; z = along the hand axis, negative = the peg slid up into the
fingers). Successes / failures are split for the slip so a correlation shows. True force (wrist F/T, lag-corrected:
res["wrist_fixed"] from recompute_wrist_peaks.py, or runs written after the fix), median over successes of the
engaged-phase max, the 50 ms mean max, and the max before seating (wrist_force_stats.py; the max is often one sample
when the peg hits the bottom).
    python3 scripts/heldout_summary.py ~/ci_mpc_runs/heldout_realarm   # <arm>/<task>/<name>/<offset>/res.json
    python3 scripts/heldout_summary.py ~/ci_mpc_runs/h3_100            # <arm>/rep<n>/<assembly>/res.json
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrist_force_stats import stats  # noqa: E402

root = sys.argv[1]
rows = {}
for f in sorted(glob.glob(os.path.join(root, "**", "res.json"), recursive=True)):
    parts = os.path.relpath(f, root).split(os.sep)
    arm, task = parts[0], ("H3-100" if parts[1].startswith("rep") else parts[1])
    r = json.load(open(f))
    r["_wrist"] = stats(os.path.dirname(f), r) if (r.get("wrist_lag_s") or r.get("wrist_fixed")) else None
    rows.setdefault((task, arm), []).append(r)


def med(v):
    v = [x for x in v if np.isfinite(x)]
    return f"{np.median(v):5.2f}" if v else "    -"


def mx(v):
    v = [x for x in v if np.isfinite(x)]
    return f"{np.max(v):5.2f}" if v else "    -"


print(f"{'task':8s} {'arm':8s} {'inserted':>9s} {'contact s':>9s} | wrist N: {'max':>5s} {'50ms':>5s} {'preseat':>7s} "
      f"| slip |xyz| mm: {'med':>5s} {'max':>5s} {'ok med':>6s} {'fail med':>8s} | z med")
for (task, arm), rs in sorted(rows.items()):
    ok = [r for r in rs if r["success"]]
    sl = [np.linalg.norm(r["peg_slip_hand_mm"]) if r.get("peg_slip_hand_mm") else np.nan for r in rs]
    sz = [r["peg_slip_hand_mm"][2] if r.get("peg_slip_hand_mm") else np.nan for r in rs]
    s_ok = [s for s, r in zip(sl, rs) if r["success"]]
    s_fail = [s for s, r in zip(sl, rs) if not r["success"]]
    ct = med([r["contact_phase_s"] for r in ok])
    fw = [med([r["_wrist"][k] if r["_wrist"] else np.nan for r in ok]) for k in ("max", "mean50", "preseat")]
    print(f"{task:8s} {arm:8s} {len(ok):4d}/{len(rs):<4d} {ct:>9s} |          {fw[0]:>5s} {fw[1]:>5s} {fw[2]:>7s} "
          f"|              {med(sl):>5s} {mx(sl):>5s} {med(s_ok):>6s} {med(s_fail):>8s} | {med(sz)}")
