#!/bin/bash
# Weekend driver (restarted 2026-10-24 after the OOM of 2026-10-23 18:52): the phases one after the other so that at
# most ~4 Isaac instances run at once (31 GB RAM; 8 + GPU servers + the offline H4 ran out of memory).
#   setsid nohup scripts/run_weekend.sh > ~/ci_mpc_runs/weekend/run_weekend.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=~/ci_mpc_runs/weekend
log() { echo "$(date '+%F %T') $*"; }
log "phase 1: track A repeats 2-3 (probe, spiral) + H4 offline"
rm -rf "$W/A/rep2" "$W/H4"; mkdir -p "$W/H4"
"$HERE/heldout_h4.sh" "$W/H4" > "$W/H4/driver.log" 2>&1 &
H4=$!
START=2 PARALLEL=4 "$HERE/repeat_benchmark.sh" "$W/A" "probe spiral" 3 >> "$W/A/driver.log" 2>&1
wait $H4
log "phase 2: track B, probe --entry gpu, 3 repeats (3 parallel, one GPU server each)"
rm -rf "$W/B/bench_gpu"
for r in 1 2 3; do
  log "  B repeat $r"
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$W/B/rep$r" probe --entry gpu \
    --timeout-contact 150 --max-probes 25 > "$W/B/rep${r}_probe_gpu.txt" 2>&1
done
log "phase 3: H1 held-out conditions"
"$HERE/heldout_h1.sh" "$W/H1" > "$W/H1/driver.log" 2>&1
log "all done"
grep -h inserted "$W"/A/rep*_*.txt "$W"/B/rep*_probe_gpu.txt "$W"/H1/*.txt; cat "$W/A/summary.md" "$W/H4/summary.txt"
