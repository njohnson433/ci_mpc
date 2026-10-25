#!/bin/bash
# After queue_track_height.sh: the learned probe likelihood (sensor-error randomized, ci_mpc_planner/models/
# probe_likelihood_rand_v1.npz) in the hole belief, for plain probe (A4, vs A2 29/30) and the GPU entry with lateral
# centering (B11, vs B8 28/30), interleaved per repeat. Offline replay (search probes, 51 held-out runs): belief error
# 2.83 -> 1.96 mm, P(within 2 mm) 0.33 -> 0.49 vs the hand model.
#   setsid nohup scripts/queue_learned_belief.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/lb_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
MODEL=${MODEL:-$HERE/../ci_mpc_planner/models/probe_likelihood_rand_v1.npz}
mkdir -p "$W/A4" "$W/B11"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "th queue done" "$W/th_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --belief-model "$MODEL" "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A4 (plain probe, learned belief)"; bench "$W/A4/rep$r" > "$W/A4/rep${r}_probe.txt" 2>&1
  log "repeat $r: B11 (GPU entry, lateral centering, learned belief)"
  bench "$W/B11/rep$r" --entry gpu --lateral-centering > "$W/B11/rep${r}_probe_gpu.txt" 2>&1
done
log "lb queue done"
for j in A4 B11; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
