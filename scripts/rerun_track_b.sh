#!/bin/bash
# Track B rerun with the GPU-phase belief fix (one belief update per engagement; 2026-10-24): probe --entry gpu,
# REPEATS x the 10 D1 offsets, 3 parallel on cores 16-31. Waits for H3 and the selector relabel job to finish.
#   setsid nohup scripts/rerun_track_b.sh ~/ci_mpc_runs/weekend/B2 3 > ~/ci_mpc_runs/weekend/B2.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; REPEATS=${2:-3}
until grep -q "H3 done" ~/ci_mpc_runs/weekend/H3/driver.log 2>/dev/null && \
      grep -q "done$" ~/ci_mpc_runs/selector/sgs/binary/run_bin_sel.log 2>/dev/null; do sleep 60; done
grep -q "Track B: belief fix" "$HERE/peg_insert_demo.py" || grep -q "one update per GPU step counted" "$HERE/peg_insert_demo.py" \
  || { echo "fix not in peg_insert_demo.py: not running"; exit 1; }
for r in $(seq 1 "$REPEATS"); do
  echo "$(date '+%F %T') B2 repeat $r"
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$OUT/rep$r" probe --entry gpu \
    --timeout-contact 150 --max-probes 25 > "$OUT/rep${r}_probe_gpu.txt" 2>&1
done
echo "$(date '+%F %T') B2 done"
grep -h inserted "$OUT"/rep*_probe_gpu.txt
