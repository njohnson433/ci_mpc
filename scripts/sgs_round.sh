#!/bin/bash
# One success-guided-start (SGS) DAgger round for the option selector (ablation of the start distribution only):
# like the earlier round 1 (16 workers x 5 episodes, beta 0.5, rolled out with SEL_PREV), but starts drawn from the
# start cache (ci_mpc_planner/start_cache.py); then train on PRIOR + this round, fair 30-start eval (seeds 50-64 x 2).
#   scripts/sgs_round.sh OUT_DIR CACHE.json SEL_PREV.npz "PRIOR_GLOB" CORES_FIRST [extra train options]
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; CACHE=$2; SEL=$3; PRIOR=$4; C0=$5; shift 5
export PYTHONPATH="$HERE/../ci_mpc_planner"
mkdir -p "$OUT/data" "$OUT/eval"
for w in $(seq 0 15); do
  taskset -c $((C0 + w)) python3 "$HERE/selector_dagger.py" --out "$OUT/data/r_$w.npz" --episodes 5 --seed $((500 + w)) \
    --selector "$SEL" --beta 0.5 --cache "$CACHE" > "$OUT/data/r_$w.log" 2>&1 &
done
wait
cp "$CACHE" "$OUT/cache_before.json"
python3 "$HERE/start_cache_update.py" "$CACHE" "$OUT/data/*_eps.json"
grep -h "entered" "$OUT"/data/r_*.log | awk '{e+=$5; split($5,a,"/"); n+=a[2]} END {print "round entered", e "/" n}'
mkdir -p "$OUT/train"; cp $PRIOR "$OUT/train/" 2>/dev/null; cp "$OUT"/data/r_*.npz "$OUT/train/"
taskset -c $C0-$((C0 + 15)) ~/IsaacLab/.venv/bin/python "$HERE/train_c3_selector.py" "$OUT/train/*.npz" "$OUT/sel.npz" "$@" \
  > "$OUT/train.log" 2>&1
tail -2 "$OUT/train.log"
for s in $(seq 50 64); do
  taskset -c $((C0 + s % 16)) python3 "$HERE/sampled_teacher_eval.py" --episodes 2 --seed $s --policy selector \
    --student "$OUT/sel.npz" 2>&1 | grep -vE "Warning|warn" > "$OUT/eval/fair_$s.log" &
done
wait
cat "$OUT"/eval/fair_*.log | python3 -c "
import sys, re, numpy as np
n=e=0; pk=[]
for l in sys.stdin:
    m=re.search(r'(ENTERED|stuck) after (\d+) steps.*peak force ([\d.]+) N', l)
    if m: n+=1; e+=m.group(1)=='ENTERED'; pk.append(float(m.group(3)))
print('fair eval: entered %d/%d, peak median %.2f N, p90 %.2f N' % (e, n, np.median(pk), np.percentile(pk, 90)))" | tee "$OUT/summary.txt"
