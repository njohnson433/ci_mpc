#!/bin/bash
# Isaac D1: plain probe + v3 (recommended setup) vs the same with the 3D C3 handover on a tip sink (--entry c3, goal
# 11 mm below the bottom; PROGRESS 2026-10-27), interleaved per offset, one Isaac instance (shared machine:
# cores 16-31, ROS domain 100). res.json carries per-phase peak observer force (phases, peak_force_engaged_N).
#   setsid nohup scripts/queue_c3_isaac.sh ~/ci_mpc_runs/c3_isaac/d1 [REPEATS] > ~/ci_mpc_runs/c3_isaac/d1.log 2>&1 < /dev/null &
# DOMAIN_BASE (env, default 100): one per parallel queue. ARMS (env, default "plain c3") selects the arms; C3_EXTRA (env) adds options to the c3 arm (e.g. "--c3-std-max 0.0005").
HERE=$(cd "$(dirname "$0")" && pwd)
W=$1; REPEATS=${2:-1}
OFFS=${OFFS:-"0,0 2,0 3,0 -3,0 5,0 -5,0 0,4 3,3 -4,2 -3,-2"}
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
COMMON=(--timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008
        --belief-model "$MODEL")
log() { echo "$(date '+%F %T') $*"; }
for r in $(seq 1 "$REPEATS"); do
  for off in $OFFS; do
    for arm in ${ARMS:-plain c3}; do
      extra=(); [ "$arm" = c3 ] && extra=(--entry c3 ${C3_EXTRA:-})
      log "rep $r $off $arm"
      OFFSETS="$off" PARALLEL=1 DOMAIN_BASE=${DOMAIN_BASE:-100} taskset -c 16-31 "$HERE/peg_benchmark.sh" "$W/$arm/rep$r" probe \
        "${COMMON[@]}" "${extra[@]}" > "$W/$arm/rep${r}_${off/,/_}.txt" 2>&1
    done
  done
done
log "c3 isaac queue done"
