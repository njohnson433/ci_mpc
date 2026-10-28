#!/bin/bash
# Cross-task held-out check of the C3 candidate (CLAUDE.md "Don't overfit to the peg", 2026-10-28): MF3 stiction
# feedforward 2.9/2.7 N + belief-std gate 3 mm were tuned on D1 peg only. Same tasks / offsets / seeds as
# queue_cross_task.sh (H1 peg conditions, H2 gear, H3 AutoMate), nominal scene (the real-arm friction scene is
# peg-only), two arms with the D1 parameters unchanged:
#   plain: today's probe flags (= queue_cross_task.sh "new", the D1 plain arm; v3 belief on the peg H1 only)
#   c3:    plain + --entry c3 + MF3 + --c3-std-max 0.003 (as D1 mf3g3)
# H4 (offline GPU sampling MPC) does not exercise C3. Waits for the C3 integrator queue (night/integ.log).
#   setsid nohup scripts/queue_cross_task_c3.sh ~/ci_mpc_runs/cross_c3 > ~/ci_mpc_runs/cross_c3.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
export WRIST_FT=1  # true contact force (wrist_ft.csv): the observer peak is stiction-biased (PROGRESS 2026-10-28)
OUT=$1
mkdir -p "$OUT"
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
COMMON=(--timeout-contact 150 --max-probes 25)
NEW=(--track-height 5 --centering-bound 0.008 --stuck-dither 0.0008)
C3=(--entry c3 --c3-arm-friction 2.2,1.9,2.2,0.05,0.05 --c3-stiction-ff 2.9,2.7 --c3-std-max 0.003)
log() { echo "$(date '+%F %T') $*"; }
until grep -q "night done" ~/ci_mpc_runs/c3_isaac/night/integ.log 2>/dev/null; do sleep 60; done
offsets() {  # n random offsets "dx,dy" [mm], r 1-4 mm, as heldout_h1.sh / heldout_h2.sh
  python3 -c "
import numpy as np
r = np.random.default_rng($1)
print(' '.join(f'{a:.2f},{b:.2f}' for a, b in (r.uniform(1, 4) * np.array([np.cos(t), np.sin(t)]) for t in r.uniform(0, 2 * np.pi, $2))))"
}
bench() {  # bench OUT env... -- options...
  local out=$1; shift
  local envs=(); while [ "$1" != "--" ]; do envs+=("$1"); shift; done; shift
  env "${envs[@]}" PARALLEL=3 DOMAIN_BASE=80 taskset -c 16-31 "$HERE/peg_benchmark.sh" "$out" probe "${COMMON[@]}" "$@"
}
# H1: peg under 5 untuned conditions x 3 offsets (seeds 11-15)
h1() {  # name seed env... -- options...
  local name=$1 seed=$2; shift 2
  local envs=(); while [ "$1" != "--" ]; do envs+=("$1"); shift; done; shift
  for arm in plain c3; do
    extra=("$@" "${NEW[@]}" --belief-model "$MODEL"); [ $arm = c3 ] && extra+=("${C3[@]}")
    log "H1 $name $arm"
    bench "$OUT/H1_$arm/$name" OFFSETS="$(offsets "$seed" 3)" "${envs[@]}" -- "${extra[@]}" > "$OUT/H1_$arm/$name.txt" 2>&1
  done
}
mkdir -p "$OUT/H1_plain" "$OUT/H1_c3" "$OUT/H2_plain" "$OUT/H2_c3"
h1 friction_0.5 11 PEG_FRICTION=0.5 --
h1 friction_1.0 12 PEG_FRICTION=1.0 --
h1 hole_tilt_2deg 13 PEG_HOLE_TILT=2.0 --
h1 grasp_offset 14 -- --grasp-offset=1.5,1.0
h1 hole_elsewhere 15 PEG_HOLE_XY=0.45,-0.05 --
# H2: gear mesh, nominal 5 offsets (seed 21)
for arm in plain c3; do
  extra=("${NEW[@]}"); [ $arm = c3 ] && extra+=("${C3[@]}")
  log "H2 nominal $arm"
  bench "$OUT/H2_$arm/nominal" DEMO="$HERE/demo_gear.sh" OFFSETS="$(offsets 21 5)" -- "${extra[@]}" \
    > "$OUT/H2_$arm/nominal.txt" 2>&1
done
# H3: AutoMate, same 12 assemblies and offsets (seed 300 + k) as heldout_h3.sh; plain and c3 of one assembly side by
# side, plus the next assembly's plain (3 in parallel)
ASSEMBLIES="00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041"
jobs_=()
k=0
for a in $ASSEMBLIES; do
  off=$(python3 -c "
import numpy as np
r = np.random.default_rng(300 + $k); t = r.uniform(0, 2 * np.pi); m = r.uniform(1, 4)
print(f'{m * np.cos(t):.2f},{m * np.sin(t):.2f}')")
  for arm in plain c3; do jobs_+=("$a $off $arm"); done
  k=$((k + 1))
done
i=0
for j in "${jobs_[@]}"; do
  read -r a off arm <<< "$j"
  extra=("${NEW[@]}"); [ $arm = c3 ] && extra+=("${C3[@]}")
  d="$OUT/H3_$arm/$a"; mkdir -p "$d"
  while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt 7 ]; do sleep 20; done
  log "H3 $a $arm offset $off"
  (AUTOMATE=$a LOG="$d/isaac.log" ROS_DOMAIN_ID=$((80 + i % 3)) taskset -c 16-31 timeout 2400 "$HERE/demo_automate.sh" \
     --headless --offset "$off" --strategy probe "${COMMON[@]}" "${extra[@]}" --log-dir "$d" --out "$d/res.json" \
     > "$d/run.out" 2>&1) &
  i=$((i + 1)); sleep 15
  [ $((i % 3)) -eq 0 ] && wait
done
wait
log "cross-task queue done"
python3 - "$OUT" <<'PY'
import glob, json, os, sys
out = sys.argv[1]
for task in ("H1", "H2", "H3"):
    for arm in ("plain", "c3"):
        fs = sorted(glob.glob(f"{out}/{task}_{arm}/**/res.json", recursive=True))
        ok = [json.load(open(f))["success"] for f in fs]
        ct = [json.load(open(f))["contact_phase_s"] for f in fs if json.load(open(f))["success"]]
        med = sorted(ct)[len(ct) // 2] if ct else float("nan")
        print(f"{task} {arm:4s}: {sum(ok)}/{len(ok)} inserted, median contact {med:.1f} s (successes)")
PY
