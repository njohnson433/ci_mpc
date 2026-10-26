#!/bin/bash
# Fresh held-out set R2 (seed 78, r 1-5.4 mm) after queue_heldout_r.sh: the GPU-phase confidence gate (--gpu-std-max 1 mm:
# hand over to the MPC only when the belief std < 1 mm, else center; motivated by an R-set run, hence a fresh set).
# R2A_v3 plain probe + v3 (reference), R2B_v3 GPU entry + lateral centering + v3, R2B_v3g the same + the gate; all with
# height tracking, centering bound, stuck dither; 3 repeats, interleaved.
#   setsid nohup scripts/queue_heldout_r2.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/r2_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
export OFFSETS="1.67,-2.30 0.05,5.03 -0.74,-3.20 -1.16,-4.54 1.79,-1.94 0.36,3.49 2.39,-1.20 1.83,1.59 -3.24,1.46 3.63,-1.59"
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
mkdir -p "$W/R2A_v3" "$W/R2B_v3" "$W/R2B_v3g"
log() { echo "$(date '+%F %T') $*"; }
until grep -q "r queue done" "$W/r_queue.log" 2>/dev/null; do sleep 60; done
bench() {  # bench OUT [extra options]
  local out=$1; shift
  PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --track-height 5 --centering-bound 0.008 --belief-model "$MODEL" --stuck-dither 0.0008 "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: R2A_v3"; bench "$W/R2A_v3/rep$r" > "$W/R2A_v3/rep${r}_probe.txt" 2>&1
  log "repeat $r: R2B_v3"; bench "$W/R2B_v3/rep$r" --entry gpu --lateral-centering > "$W/R2B_v3/rep${r}_probe_gpu.txt" 2>&1
  log "repeat $r: R2B_v3g"; bench "$W/R2B_v3g/rep$r" --entry gpu --lateral-centering --gpu-std-max 0.001 \
    > "$W/R2B_v3g/rep${r}_probe_gpu.txt" 2>&1
done
log "r2 queue done"
for j in R2A_v3 R2B_v3 R2B_v3g; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
