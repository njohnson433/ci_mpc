#!/bin/bash
# After queue_engage_sink.sh: GPU entry (B8) and refine -1 (B9) with --lateral-centering (a push centers as in plain
# probe, only a tip sink starts the GPU phase), interleaved per repeat; compare with B4/B5 (push -> GPU), B6/B7, A2.
#   setsid nohup scripts/queue_lateral_centering.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/lc_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
mkdir -p "$W/B8" "$W/B9"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "sink queue done" "$W/sink_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --lateral-centering "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: B8 (GPU entry, lateral centering)"; bench "$W/B8/rep$r" --entry gpu > "$W/B8/rep${r}_probe_gpu.txt" 2>&1
  log "repeat $r: B9 (refine -1, lateral centering)"; bench "$W/B9/rep$r" --entry gpu --gpu-refine-margin=-1 \
    > "$W/B9/rep${r}_probe_refine.txt" 2>&1
done
log "lc queue done"
for j in B8 B9; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
