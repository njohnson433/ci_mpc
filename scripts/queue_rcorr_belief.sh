#!/bin/bash
# After queue_learned_belief.sh: as A4 / B11 but with the distance-prior corrected likelihood (models/
# probe_likelihood_rand_v1_rcorr.npz): A5 (plain probe, vs A4) and B12 (GPU entry + lateral centering, vs B11). The v1
# logit includes log p_pos(r)/p_neg(r) of the strategy-chosen probe positions: a flat reading favoured holes ~4.5 mm
# away and the belief chased its own probes (A4 rep1 0,4). One lever: only the correction differs.
#   setsid nohup scripts/queue_rcorr_belief.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/rc_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
MODEL=${MODEL:-$HERE/../ci_mpc_planner/models/probe_likelihood_rand_v1_rcorr.npz}
mkdir -p "$W/A5" "$W/B12"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "lb queue done" "$W/lb_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --belief-model "$MODEL" "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: A5 (plain probe, learned belief, r-corrected)"; bench "$W/A5/rep$r" > "$W/A5/rep${r}_probe.txt" 2>&1
  log "repeat $r: B12 (GPU entry, lateral centering, learned belief, r-corrected)"
  bench "$W/B12/rep$r" --entry gpu --lateral-centering > "$W/B12/rep${r}_probe_gpu.txt" 2>&1
done
log "rc queue done"
for j in A5 B12; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
