#!/bin/bash
# After queue_calib_fix.sh (incl. H3_fix): GPU entry (B6) and refine -1 (B7) with engagement = tip sink only
# (--engage-lateral inf), interleaved per repeat; compare with B4 / B5 (same code, lateral > 1.2 N also engages).
#   setsid nohup scripts/queue_engage_sink.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/sink_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
mkdir -p "$W/B6" "$W/B7"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "H3_fix done" "$W/calib_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --engage-lateral inf "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: B6 (GPU entry, sink-only engage)"; bench "$W/B6/rep$r" --entry gpu > "$W/B6/rep${r}_probe_gpu.txt" 2>&1
  log "repeat $r: B7 (refine -1, sink-only engage)"; bench "$W/B7/rep$r" --entry gpu --gpu-refine-margin=-1 \
    > "$W/B7/rep${r}_probe_refine.txt" 2>&1
done
log "sink queue done"
for j in B6 B7; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
