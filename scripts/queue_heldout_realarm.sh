#!/bin/bash
# Held-out check (CLAUDE.md "Don't overfit to the peg") of src-54's real-robot items, 2026-10-28: phase-dependent
# friction compensation (controller 347044f, friction_compensation: hybrid) and the real-FR3 C3 values. All arms on the
# real-arm match scene fr3_scene_realfric.usd (joints 1-4 at x0.5: Cartesian breakaway 1.49 / 1.08 / 1.40 N vs real
# 1.37 / 0.86 / 1.15; src-5e: not fr3_scene_real1028.usd, 1.4-2x stickier). Same tasks / offsets / seeds as
# queue_cross_task_c3.sh: H1 peg conditions (5 x 3), H2 gear (5), H3 AutoMate (12). D1 / real values, nothing tuned.
#   base     friction_compensation none (as on the real robot today), plain probe
#   hyb0     hybrid, velocity part 0.75, rest 0 (src-54: contact should use 0 unless H1-H3 show otherwise)
#   hyb05    hybrid 0.75 / rest 0.5 (halves the breakaway at rest on the real arm; H4: raises the contact force)
#   c3nofc   base + C3 with the real warm values 1.37,0.86,1.15,1.24,0.68, ff 1.4,1.25 (the 10-27 offline pick), gate 3 mm
#   c3hyb05  hyb05 + C3 with the compensated real values 0.84,0.66,0.61,0.67,0.39, ff 0.50,0.31, gate 3 mm
# Resumable (a job with res.json is skipped). Compensation arms first, then the C3 arms (ARMS env to choose).
#   setsid nohup scripts/queue_heldout_realarm.sh ~/ci_mpc_runs/heldout_realarm > ~/ci_mpc_runs/heldout_realarm.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
export WRIST_FT=1  # true contact force (wrist_ft.csv)
export SCENE=$HOME/ci_mpc_assets/fr3_scene_realfric.usd  # hyb05nom: nominal scene (per job below)
OUT=$1; mkdir -p "$OUT"
PAR=${PAR:-2}; CORES=${CORES:-0-15}; DOM0=${DOM0:-120}
ARMS=${ARMS:-"base hyb0 hyb05 c3nofc c3hyb05"}
TASKS=${TASKS:-"H1 H2 H3"}  # e.g. TASKS=H3 for repeats into another OUT (rep 2, 3)
CFG=$HERE/../ci_mpc_sim_gripper/config
MODEL=$HERE/../ci_mpc_planner/models/probe_likelihood_v3_both.npz
COMMON=(--timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008)
log() { echo "$(date '+%F %T') $*"; }
arm_cfg() { case $1 in base|c3nofc) echo "$CFG/isaac_pick_place_nofc.yaml" ;; hyb0) echo "$CFG/isaac_pick_place_hybrid_rest00.yaml" ;;
  hyb05|c3hyb05) echo "$CFG/isaac_pick_place_hybrid_rest05.yaml" ;;
  hyb05nom) echo "$CFG/isaac_pick_place_hybrid_nominal.yaml" ;;
  velnom) echo "$CFG/isaac_pick_place_velocity.yaml" ;; esac; }
arm_extra() { case $1 in
  c3nofc)  echo "--entry c3 --c3-arm-friction 1.37,0.86,1.15,1.24,0.68 --c3-stiction-ff 1.4,1.25 --c3-std-max 0.003" ;;
  c3hyb05) echo "--entry c3 --c3-arm-friction 0.84,0.66,0.61,0.67,0.39 --c3-stiction-ff 0.50,0.31 --c3-std-max 0.003" ;;
  esac; }
offsets() {  # seed n -> n offsets "dx,dy" [mm], r 1-4 mm (as queue_cross_task_c3.sh)
  python3 -c "
import numpy as np
r = np.random.default_rng($1)
print(' '.join(f'{a:.2f},{b:.2f}' for a, b in (r.uniform(1, 4) * np.array([np.cos(t), np.sin(t)]) for t in r.uniform(0, 2 * np.pi, $2))))"
}
# job list: "task|name|offset|demo|env (space separated k=v)|task extra flags"
JOBS=()
for c in "friction_0.5|11|PEG_FRICTION=0.5|" "friction_1.0|12|PEG_FRICTION=1.0|" "hole_tilt_2deg|13|PEG_HOLE_TILT=2.0|" \
         "grasp_offset|14||--grasp-offset=1.5,1.0" "hole_elsewhere|15|PEG_HOLE_XY=0.45,-0.05|"; do
  IFS='|' read -r name seed envs fl <<< "$c"
  for off in $(offsets "$seed" 3); do JOBS+=("H1|$name|$off|demo_peg.sh|$envs|$fl --belief-model $MODEL"); done
done
for off in $(offsets 21 5); do JOBS+=("H2|nominal|$off|demo_gear.sh||"); done
k=0
for a in 00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041; do
  off=$(python3 -c "
import numpy as np
r = np.random.default_rng(300 + $k); t = r.uniform(0, 2 * np.pi); m = r.uniform(1, 4)
print(f'{m * np.cos(t):.2f},{m * np.sin(t):.2f}')")
  JOBS+=("H3|$a|$off|demo_automate.sh|AUTOMATE=$a|"); k=$((k + 1))
done
i=0
log "held-out real-arm queue: arms $ARMS, ${#JOBS[@]} jobs per arm, PAR $PAR"
for arm in $ARMS; do
  for j in "${JOBS[@]}"; do
    IFS='|' read -r task name off demo envs fl <<< "$j"
    [[ " $TASKS " == *" $task "* ]] || continue
    d="$OUT/$arm/$task/$name/${off/,/_}"
    [ -f "$d/res.json" ] && continue
    while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt 8 ]; do sleep 20; done
    while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
    mkdir -p "$d"; log "$arm $task $name $off"
    # shellcheck disable=SC2086
    sc=$SCENE; [ $arm = hyb05nom ] || [ $arm = velnom ] && sc=""  # hyb05nom: the nominal scene (vs cross_c3 plain = the Isaac default)
    (env $envs SCENE="$sc" CONTROLLER_CONFIG="$(arm_cfg $arm)" LOG="$d/isaac.log" ROS_DOMAIN_ID=$((DOM0 + i % 10)) taskset -c "$CORES" \
       timeout 2400 "$HERE/$demo" --headless --offset "$off" --strategy probe "${COMMON[@]}" $fl $(arm_extra $arm) \
       --log-dir "$d" --out "$d/res.json" > "$d/run.out" 2>&1; [ -f "$d/res.json" ] || log "no res.json: $d") &
    i=$((i + 1)); sleep 15
  done
done
wait
log "held-out real-arm queue done"
python3 - "$OUT" <<'PY'
import glob, json, sys
import numpy as np
out = sys.argv[1]
for task in ("H1", "H2", "H3"):
    for arm in ("base", "hyb0", "hyb05", "c3nofc", "c3hyb05"):
        rs = [json.load(open(f)) for f in sorted(glob.glob(f"{out}/{arm}/{task}/*/*/res.json"))]
        if not rs:
            continue
        ok = [r for r in rs if r["success"]]
        ct = np.median([r["contact_phase_s"] for r in ok]) if ok else float("nan")
        fw = np.median([r.get("peak_force_wrist_engaged_N", np.nan) for r in ok]) if ok else float("nan")
        print(f"{task} {arm:8s}: {len(ok)}/{len(rs)} inserted, median contact {ct:.1f} s, wrist peak {fw:.1f} N (successes)")
PY
