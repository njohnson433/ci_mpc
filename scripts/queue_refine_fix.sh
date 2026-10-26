#!/bin/bash
# Isaac rerun of the sample-then-refine entry after the C3 dual-rescale fix (2026-10-26, 6cedabc): on the held-out R2
# set with today's R2B_v3 configuration (GPU entry, lateral centering, v3 belief, height tracking, centering bound,
# stuck dither) + --gpu-refine-margin=-1. Two arms interleaved per repeat, only the ADMM differs:
#   R2C_fix: fixed C3 (default)   R2C_old: --gpu-no-dual-rescale (the C3 behind B5/B7/B9)
# References on the same offsets: R2B_v3 (sampler alone) 30/30, 39.6 s mean contact; R2A_v3 (plain probe) 30/30, 39.4 s.
# Waits until at most one other Isaac instance runs (4 at most in total, CLAUDE.md).
#   setsid nohup scripts/queue_refine_fix.sh ~/ci_mpc_runs/weekend 3 > ~/ci_mpc_runs/weekend/rf_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-3}
export OFFSETS="1.67,-2.30 0.05,5.03 -0.74,-3.20 -1.16,-4.54 1.79,-1.94 0.36,3.49 2.39,-1.20 1.83,1.59 -3.24,1.46 3.63,-1.59"
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
mkdir -p "$W/R2C_fix" "$W/R2C_old"
log() { echo "$(date '+%F %T') $*"; }
n_isaac() { ps -eo rss,cmd | awk '$2 ~ /IsaacLab\/.venv\/bin\/python/ && $1 > 3000000' | wc -l; }  # Isaac: > 3 GB RSS
bench() {  # bench OUT [extra options]
  local out=$1; shift
  until [ "$(n_isaac)" -le 1 ]; do sleep 60; done
  PARALLEL=3 DOMAIN_BASE=100 taskset -c 0-15 "$HERE/peg_benchmark.sh" "$out" probe --timeout-contact 150 \
    --max-probes 25 --track-height 5 --centering-bound 0.008 --belief-model "$MODEL" --stuck-dither 0.0008 \
    --entry gpu --lateral-centering --gpu-refine-margin=-1 "$@"
}
for r in $(seq 1 "$REPEATS"); do
  log "repeat $r: R2C_fix"; bench "$W/R2C_fix/rep$r" > "$W/R2C_fix/rep${r}_probe_refine.txt" 2>&1
  log "repeat $r: R2C_old"; bench "$W/R2C_old/rep$r" --gpu-no-dual-rescale > "$W/R2C_old/rep${r}_probe_refine.txt" 2>&1
done
log "rf queue done"
for j in R2C_fix R2C_old; do echo "$j: $(cat "$W/$j"/rep*_*.txt | grep -h inserted | tr '\n' ' ')"; done
