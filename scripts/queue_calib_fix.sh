#!/bin/bash
# After B2: swap in the grasp-height calibration touch (peg_insert_demo.next.py), check it on 3 offsets, then the D1
# benchmark (10 offsets) for plain probe (A2), probe + GPU entry (B4) and probe + refine (B5), interleaved by repeat
# so the three are compared on the same code and machine state; then the H3 rerun. 3 parallel on cores 16-31.
#   setsid nohup scripts/queue_calib_fix.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/calib_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
mkdir -p "$W/A2" "$W/B4" "$W/B5"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "B2 done" "$W/B2.log" 2>/dev/null; do sleep 60; done
[ -f "$HERE/peg_insert_demo.next.py" ] && mv "$HERE/peg_insert_demo.next.py" "$HERE/peg_insert_demo.py"
grep -q "calibration touch" "$HERE/peg_insert_demo.py" || { log "calibration touch not in peg_insert_demo.py: abort"; exit 1; }
COMMON=(--timeout-contact 150 --max-probes 25)
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe "${COMMON[@]}" "$@"
}
log "calib check: 3 offsets that failed uncalibrated in B2"
OFFSETS="3,0 -3,-2 3,3" bench "$W/calib_check" > "$W/calib_check.txt" 2>&1
n_cal=$(grep -l "calibration touch, " "$W"/calib_check/probe_*/run.out 2>/dev/null | wc -l)
cat "$W/calib_check.txt" | tail -5
if [ "$n_cal" -lt 3 ]; then log "calib check: only $n_cal/3 calibrated: abort"; exit 1; fi
log "calib check: 3/3 calibrated"
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A2 (plain probe)"; bench "$W/A2/rep$r" > "$W/A2/rep${r}_probe.txt" 2>&1
  log "repeat $r: B4 (GPU entry)"; bench "$W/B4/rep$r" --entry gpu > "$W/B4/rep${r}_probe_gpu.txt" 2>&1
  log "repeat $r: B5 (refine -1)"; bench "$W/B5/rep$r" --entry gpu --gpu-refine-margin=-1 \
    > "$W/B5/rep${r}_probe_refine.txt" 2>&1
done
log "calib queue done"
for j in A2 B4 B5; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
mkdir -p "$W/H3_fix"
"$HERE/heldout_h3.sh" "$W/H3_fix" > "$W/H3_fix.log" 2>&1
log "H3_fix done"
