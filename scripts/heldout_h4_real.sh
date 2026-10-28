#!/bin/bash
# Held-out H4 (offline) with the real FR3 arm stiction as the true plant (2026-10-28 robot session, PROGRESS
# "2026-10-28 (afternoon)"): Cartesian breakaway at the probe pose, x y z [N], rx ry [Nm], no friction compensation
# warm / cold, and with coulomb compensation at scale 0.5. The MPC keeps its nominal model (2, 2, 3.5 N, 0.05 Nm).
# Same 30 fair starts as heldout_h4.sh; never tune on it.
#   nohup scripts/heldout_h4_real.sh OUT_DIR > OUT_DIR/driver.log 2>&1 &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; mkdir -p "$OUT"
CORES=${CORES:-16-31}
export PYTHONPATH="$HERE/../ci_mpc_planner"
PY=~/IsaacLab/.venv/bin/python
for cond in "nominal:" "real_warm:--plant-fa 1.37,0.86,1.15,1.24,0.68" "real_cold:--plant-fa 1.43,0.80,1.25,1.56,0.61" \
            "real_comp0.5:--plant-fa 0.84,0.66,0.61,0.67,0.39"; do
  name=${cond%%:*}; opts=${cond#*:}
  for pol in heuristic gpu; do
    for grp in "50 51 52 53 54" "55 56 57 58 59" "60 61 62 63 64"; do
      (for s in $grp; do OMP_NUM_THREADS=2 taskset -c $CORES $PY "$HERE/sampled_teacher_eval.py" --episodes 2 --seed $s \
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
echo "h4 real done"
