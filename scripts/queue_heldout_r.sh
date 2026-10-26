#!/bin/bash
# Held-out closed-loop test on 10 random hole errors (seed 77, r 1-5.4 mm; not the D1 set the methods were tuned on):
# hand model vs the v3 learned likelihood (logs + 80 randomized probe-sweep episodes, models/probe_likelihood_v3_both.npz,
# + --stuck-dither 0.8 mm), both with height tracking and the centering bound, for plain probe (RA_hand / RA_v3) and
# the GPU entry with lateral centering (RB_hand / RB_v3); 3 repeats, interleaved.
#   setsid nohup scripts/queue_heldout_r.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/r_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
export OFFSETS="0.24,-4.45 -2.72,-2.08 1.41,1.52 -1.57,1.91 1.65,-1.74 2.69,-0.37 -1.20,-4.36 0.74,1.19 1.91,-1.83 2.36,3.81"
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
mkdir -p "$W/RA_hand" "$W/RA_v3" "$W/RB_hand" "$W/RB_v3"
log() { echo "$(date '+%F %T') $*"; }
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --track-height 5 --centering-bound 0.008 "$@"
}
V3=(--belief-model "$MODEL" --stuck-dither 0.0008)
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: RA_hand"; bench "$W/RA_hand/rep$r" > "$W/RA_hand/rep${r}_probe.txt" 2>&1
  log "repeat $r: RA_v3"; bench "$W/RA_v3/rep$r" "${V3[@]}" > "$W/RA_v3/rep${r}_probe.txt" 2>&1
  log "repeat $r: RB_hand"; bench "$W/RB_hand/rep$r" --entry gpu --lateral-centering > "$W/RB_hand/rep${r}_probe_gpu.txt" 2>&1
  log "repeat $r: RB_v3"; bench "$W/RB_v3/rep$r" --entry gpu --lateral-centering "${V3[@]}" > "$W/RB_v3/rep${r}_probe_gpu.txt" 2>&1
done
log "r queue done"
for j in RA_hand RA_v3 RB_hand RB_v3; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
