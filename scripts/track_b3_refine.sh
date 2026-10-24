#!/bin/bash
# Isaac D1 benchmark of the sample-then-refine MPC entry (probe --entry gpu --gpu-refine-margin -1): 3 repeats x the
# 10 offsets, 3 parallel on cores 16-31. Compare with B2 (the sampler alone, same fixed belief code) and probe 24/30.
#   setsid nohup scripts/track_b3_refine.sh ~/ci_mpc_runs/weekend/B3 3 > ~/ci_mpc_runs/weekend/B3.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; REPEATS=${2:-3}
until grep -q "B2 done" ~/ci_mpc_runs/weekend/B2.log 2>/dev/null; do sleep 60; done
for r in $(seq 1 "$REPEATS"); do
  echo "$(date '+%F %T') B3 repeat $r"
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$OUT/rep$r" probe --entry gpu \
    --gpu-refine-margin=-1 --timeout-contact 150 --max-probes 25 > "$OUT/rep${r}_probe_refine.txt" 2>&1
done
echo "$(date '+%F %T') B3 done"
grep -h inserted "$OUT"/rep*_probe_refine.txt
