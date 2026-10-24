#!/bin/bash
# Held-out H4 for the sample-then-refine MPC (margin -1, chosen on the fair 30 starts): the same 6 true-plant
# variations and 30 starts as scripts/heldout_h4.sh; the MPC keeps its nominal model. 3 PyTorch processes at a time.
#   scripts/heldout_h4_refine.sh OUT_DIR
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; mkdir -p "$OUT"
export PYTHONPATH="$HERE/../ci_mpc_planner"
PY=~/IsaacLab/.venv/bin/python
for cond in "mu0.5:--plant-mu 0.5" "mu1.0:--plant-mu 1.0" "R4.4:--plant-R 4.4" "R4.6:--plant-R 4.6" \
            "stiction0.5:--plant-fa-scale 0.5" "stiction1.5:--plant-fa-scale 1.5"; do
  name=${cond%%:*}; opts=${cond#*:}
  for grp in "50 51 52 53 54" "55 56 57 58 59" "60 61 62 63 64"; do
    c=$((16 + (${grp%% *} - 50) / 5 * 5))
    (for s in $grp; do OMP_NUM_THREADS=2 taskset -c $c-$((c + 4)) $PY "$HERE/sampled_teacher_eval.py" --episodes 2 --seed $s \
       --policy refine --refine-margin=-1 $opts 2>&1 | grep -vE "Warning|warn"; done > "$OUT/${name}_refine_${grp%% *}.log") &
  done
  wait
  cat "$OUT/${name}_refine"_*.log | python3 -c "
import sys, re, numpy as np
n=e=0; pk=[]; st=[]
for l in sys.stdin:
    m=re.search(r'(ENTERED|stuck) after (\d+) steps.*peak force ([\d.]+) N', l)
    if m: n+=1; e+=m.group(1)=='ENTERED'; pk.append(float(m.group(3))); (st.append(int(m.group(2))) if m.group(1)=='ENTERED' else None)
print('$name refine-1: entered %d/%d, steps median %s, peak median %.2f N, p90 %.2f N' % (e, n, int(np.median(st)) if st else '-', np.median(pk), np.percentile(pk, 90)))
" | tee -a "$OUT/summary.txt"
done
