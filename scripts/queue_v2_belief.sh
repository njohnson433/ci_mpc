#!/bin/bash
# After queue_combined.sh: as A6 / B13 but with the v2 likelihood (logs + the first 43 randomized probe-sweep episodes,
# models/probe_likelihood_v2_both_partial.npz; held-out sweeps: log p(true) -5.5 vs v1-style -10.2, hand -13.6): A7 (vs
# A6) and B14 (vs B13); only the model differs (+ height tracking + centering bound as in A6 / B13).
#   setsid nohup scripts/queue_v2_belief.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/v2_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
MODEL=${MODEL:-$HERE/../ci_mpc_planner/models/probe_likelihood_v2_both_partial.npz}
mkdir -p "$W/A7" "$W/B14"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "cb queue done" "$W/cb_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --belief-model "$MODEL" --track-height 5 --centering-bound 0.008 "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A7 (plain probe, combined, v2 model)"; bench "$W/A7/rep$r" > "$W/A7/rep${r}_probe.txt" 2>&1
  log "repeat $r: B14 (GPU entry, lateral centering, combined, v2 model)"
  bench "$W/B14/rep$r" --entry gpu --lateral-centering > "$W/B14/rep${r}_probe_gpu.txt" 2>&1
done
log "v2 queue done"
for j in A7 B14; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
