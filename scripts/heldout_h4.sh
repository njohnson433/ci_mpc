#!/bin/bash
# Held-out H4 (offline): heuristic vs GPU sampling MPC on the 30 fair starts (seeds 50-64 x 2) with the *true* plant
# varied (the MPC keeps its nominal model): friction 0.5 / 1.0, hole radius 4.4 / 4.6 mm (clearance 0.4 / 0.6 mm
# instead of 0.5), arm stiction x0.5 / x1.5. Never tune on it. Cores 16-31.
#   nohup scripts/heldout_h4.sh OUT_DIR > OUT_DIR/driver.log 2>&1 &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; mkdir -p "$OUT"
export PYTHONPATH="$HERE/../ci_mpc_planner"
PY=~/IsaacLab/.venv/bin/python
for cond in "mu0.5:--plant-mu 0.5" "mu1.0:--plant-mu 1.0" "R4.4:--plant-R 4.4" "R4.6:--plant-R 4.6" \
            "stiction0.5:--plant-fa-scale 0.5" "stiction1.5:--plant-fa-scale 1.5"; do
  name=${cond%%:*}; opts=${cond#*:}
  for pol in heuristic gpu; do
    for grp in "50 51 52 53 54" "55 56 57 58 59" "60 61 62 63 64"; do
      (for s in $grp; do OMP_NUM_THREADS=2 taskset -c 16-31 $PY "$HERE/sampled_teacher_eval.py" --episodes 2 --seed $s \
         --policy $pol $opts 2>&1 | grep -vE "Warning|warn"; done > "$OUT/${name}_${pol}_${grp%% *}.log") &
    done
    wait
    cat "$OUT/${name}_${pol}"_*.log | python3 -c "
import sys, re, numpy as np
n=e=0; pk=[]; st=[]
for l in sys.stdin:
    m=re.search(r'(ENTERED|stuck) after (\d+) steps.*peak force ([\d.]+) N', l)
    if m: n+=1; e+=m.group(1)=='ENTERED'; pk.append(float(m.group(3))); (st.append(int(m.group(2))) if m.group(1)=='ENTERED' else None)
print('$name $pol: entered %d/%d, steps median %s, peak median %.2f N, p90 %.2f N' % (e, n, int(np.median(st)) if st else '-', np.median(pk), np.percentile(pk, 90)))
" | tee -a "$OUT/summary.txt"
  done
done
