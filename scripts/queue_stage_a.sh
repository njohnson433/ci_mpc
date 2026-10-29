#!/bin/bash
# Stage A baseline (CLAUDE.md "Develop in stages"): known in-hand pose, camera hole estimate (2 mm xy / 0.5 mm z noise,
# one seed per run instead of the offset grids), N(estimate, 2 mm) prior, smooth approach with observer contact, probe
# search, plain push. Held-out sets with their own seeds (D1 uses 1-20): H1 peg conditions (5 x 3, seeds 1101-1115,
# learned likelihood LIK), H2 gear (5, seeds 1201-1205), H3 AutoMate (12 assemblies, seeds 1301-1312), H5 NIST (5 tasks
# x 2, seeds 1501-1510); non-peg tasks use the hand belief (--geo, as queue_cross_task.sh). Wrist F/T on (true force).
# This is the number the CI-MPC (stage B) must beat.
#   LIK=ci_mpc_planner/models/probe_likelihood_v3_both.npz SETS="H1 H2 H3 H5" setsid nohup scripts/queue_stage_a.sh OUT &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1
mkdir -p "$OUT"
LIK=${LIK:-$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz}
SETS=${SETS:-"H1 H2 H3 H5"}
DOM=${DOMAIN_BASE:-66}
STAGEA=(--known-grasp --pos-noise 0.002 --pos-noise-z 0.0005 --prior-sigma 0.002 --smooth-approach 0.02)
COMMON=(--timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008)
log() { echo "$(date '+%F %T') $*" >> "$OUT/queue.log"; }
bench() {  # bench OUT SEEDS env... -- options...
  local out=$1 seeds=$2; shift 2
  local envs=(); while [ "$1" != "--" ]; do envs+=("$1"); shift; done; shift
  env "${envs[@]}" SEEDS="$seeds" PARALLEL=1 DOMAIN_BASE=$DOM WRIST_FT=1 taskset -c 0-15 "$HERE/peg_benchmark.sh" "$out" \
    probe "${STAGEA[@]}" "${COMMON[@]}" "$@"
}
for set in $SETS; do
  case $set in
    H1)
      s=1101
      for cond in "friction_0.5 PEG_FRICTION=0.5 --" "friction_1.0 PEG_FRICTION=1.0 --" \
                  "hole_tilt_2deg PEG_HOLE_TILT=2.0 --" "grasp_offset -- --grasp-offset=1.5,1.0" \
                  "hole_elsewhere PEG_HOLE_XY=0.45,-0.05 --"; do
        read -r -a a <<< "$cond"
        name=${a[0]}; rest=("${a[@]:1}")
        log "H1 $name start"
        mkdir -p "$OUT/H1"
        bench "$OUT/H1/$name" "$s $((s + 1)) $((s + 2))" "${rest[@]}" --belief-model "$LIK" > "$OUT/H1/$name.txt" 2>&1
        log "H1 $name done: $(grep -h inserted "$OUT/H1/$name.txt")"
        s=$((s + 3))
      done ;;
    H2)
      log "H2 start"; mkdir -p "$OUT/H2"
      bench "$OUT/H2/nominal" "1201 1202 1203 1204 1205" DEMO="$HERE/demo_gear.sh" -- > "$OUT/H2/nominal.txt" 2>&1
      log "H2 done: $(grep -h inserted "$OUT/H2/nominal.txt")" ;;
    H3)
      k=0
      for a in 00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041; do
        log "H3 $a start"; mkdir -p "$OUT/H3"
        bench "$OUT/H3/$a" "$((1301 + k))" DEMO="$HERE/demo_automate.sh" AUTOMATE=$a -- > "$OUT/H3/$a.txt" 2>&1
        log "H3 $a done: $(grep -h inserted "$OUT/H3/$a.txt")"
        k=$((k + 1))
      done ;;
    H5)
      k=0
      for a in nist_round_4mm nist_round_8mm nist_round_12mm nist_rect_8mm nist_rect_12mm; do
        log "H5 $a start"; mkdir -p "$OUT/H5"
        bench "$OUT/H5/$a" "$((1501 + k)) $((1502 + k))" DEMO="$HERE/demo_automate.sh" AUTOMATE=$a \
          AUTOMATE_ASSETS="$HOME/ci_mpc_assets/nist" -- > "$OUT/H5/$a.txt" 2>&1
        log "H5 $a done: $(grep -h inserted "$OUT/H5/$a.txt")"
        k=$((k + 2))
      done ;;
  esac
done
log "stage A queue done"
