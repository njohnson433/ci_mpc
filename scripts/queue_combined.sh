#!/bin/bash
# After queue_rcorr_belief.sh: the combined deployment candidate, learned likelihood with the distance-prior correction +
# grasp-height tracking (--track-height 5) + the centering bound (8 mm around the estimate; centering left the block in
# B9 / A4 / A5), for plain probe (A6, vs A5 / A3) and the GPU entry with lateral centering (B13, vs B12 / B10).
#   setsid nohup scripts/queue_combined.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/cb_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
MODEL=${MODEL:-$HERE/../ci_mpc_planner/models/probe_likelihood_rand_v1_rcorr.npz}
mkdir -p "$W/A6" "$W/B13"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "rc queue done" "$W/rc_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --belief-model "$MODEL" --track-height 5 --centering-bound 0.008 "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A6 (plain probe, combined)"; bench "$W/A6/rep$r" > "$W/A6/rep${r}_probe.txt" 2>&1
  log "repeat $r: B13 (GPU entry, lateral centering, combined)"
  bench "$W/B13/rep$r" --entry gpu --lateral-centering > "$W/B13/rep${r}_probe_gpu.txt" 2>&1
done
log "cb queue done"
for j in A6 B13; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
