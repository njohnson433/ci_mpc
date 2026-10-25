#!/bin/bash
# After queue_lateral_centering.sh: grasp-height tracking (--track-height 5) for plain probe (A3, vs A2) and the GPU entry
# with lateral centering (B10, vs B8), interleaved per repeat. The peg creeps in the fingers ~0.005-0.01 mm per probe; on
# long searches the flat face read as sunk (< -0.3 mm) and started GPU phases far from the hole (B8 rep1 -5,0).
#   setsid nohup scripts/queue_track_height.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/th_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
mkdir -p "$W/A3" "$W/B10"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "lc queue done" "$W/lc_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --track-height 5 "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A3 (plain probe, height tracking)"; bench "$W/A3/rep$r" > "$W/A3/rep${r}_probe.txt" 2>&1
  log "repeat $r: B10 (GPU entry, lateral centering, height tracking)"
  bench "$W/B10/rep$r" --entry gpu --lateral-centering > "$W/B10/rep${r}_probe_gpu.txt" 2>&1
done
log "th queue done"
for j in A3 B10; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
