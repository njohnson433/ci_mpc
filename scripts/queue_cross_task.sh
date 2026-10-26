#!/bin/bash
# Cross-task regression after queue_heldout_r2.sh: the probe-strategy fixes since the weekend (calibration touch, height
# tracking, centering bound, stuck dither, learned v3 belief) were developed on the D1 peg set only. Re-run the held-out
# tasks with the same conditions / offsets / seeds as H1, H2 (nominal) and H3, two arms each with today's code:
#   base: --max-probes 25 (the weekend probe flags)
#   new:  + --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008, + the learned v3 belief on the peg (H1) only
#         (v3 was trained on 8 mm peg probes; gear / AutoMate use --geo, where the hand belief stays and the calibration
#         touch is off).
# Weekend numbers (old code, not comparable 1:1): H1 probe 14/15, H2 nominal probe 3/5, H3 probe 5/12 (rerun 4/12).
#   setsid nohup scripts/queue_cross_task.sh ~/ci_mpc_runs/weekend/X > ~/ci_mpc_runs/weekend/x_queue.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1
mkdir -p "$OUT"
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
COMMON=(--timeout-contact 150 --max-probes 25)
NEW=(--track-height 5 --centering-bound 0.008 --stuck-dither 0.0008)
log() { echo "$(date '+%F %T') $*"; }
until grep -q "r2 queue done" ~/ci_mpc_runs/weekend/r2_queue.log 2>/dev/null; do sleep 60; done
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
  for arm in base new; do
    extra=("$@"); [ $arm = new ] && extra+=("${NEW[@]}" --belief-model "$MODEL")
    log "H1 $name $arm"
    bench "$OUT/H1_$arm/$name" OFFSETS="$(offsets "$seed" 3)" "${envs[@]}" -- "${extra[@]}" > "$OUT/H1_$arm/$name.txt" 2>&1
  done
}
mkdir -p "$OUT/H1_base" "$OUT/H1_new" "$OUT/H2_base" "$OUT/H2_new"
h1 friction_0.5 11 PEG_FRICTION=0.5 --
h1 friction_1.0 12 PEG_FRICTION=1.0 --
h1 hole_tilt_2deg 13 PEG_HOLE_TILT=2.0 --
h1 grasp_offset 14 -- --grasp-offset=1.5,1.0
h1 hole_elsewhere 15 PEG_HOLE_XY=0.45,-0.05 --
# H2: gear mesh, nominal 5 offsets (seed 21)
for arm in base new; do
  extra=(); [ $arm = new ] && extra=("${NEW[@]}")
  log "H2 nominal $arm"
  bench "$OUT/H2_$arm/nominal" DEMO="$HERE/demo_gear.sh" OFFSETS="$(offsets 21 5)" -- "${extra[@]}" \
    > "$OUT/H2_$arm/nominal.txt" 2>&1
done
# H3: AutoMate, same 12 assemblies and offsets (seed 300 + k) as heldout_h3.sh; base and new of one assembly side by
# side, plus the next assembly's base (3 in parallel)
ASSEMBLIES="00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041"
jobs_=()
k=0
for a in $ASSEMBLIES; do
  off=$(python3 -c "
import numpy as np
r = np.random.default_rng(300 + $k); t = r.uniform(0, 2 * np.pi); m = r.uniform(1, 4)
print(f'{m * np.cos(t):.2f},{m * np.sin(t):.2f}')")
  for arm in base new; do jobs_+=("$a $off $arm"); done
  k=$((k + 1))
done
i=0
for j in "${jobs_[@]}"; do
  read -r a off arm <<< "$j"
  extra=(); [ $arm = new ] && extra=("${NEW[@]}")
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
    for arm in ("base", "new"):
        fs = sorted(glob.glob(f"{out}/{task}_{arm}/**/res.json", recursive=True))
        ok = [json.load(open(f))["success"] for f in fs]
        ct = [json.load(open(f))["contact_phase_s"] for f in fs if json.load(open(f))["success"]]
        med = sorted(ct)[len(ct) // 2] if ct else float("nan")
        print(f"{task} {arm:4s}: {sum(ok)}/{len(ok)} inserted, median contact {med:.1f} s (successes)")
PY
