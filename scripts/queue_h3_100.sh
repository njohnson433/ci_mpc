#!/bin/bash
# H3-100: AutoMate held-out breadth set, all assemblies in ~/ci_mpc_assets/automate (user request 2026-10-28, CLAUDE.md
# "Don't overfit to the peg"). The 12 H3 assemblies keep their H3 offsets (seed 300 + k, queue_cross_task.sh order), so
# H3 stays a subset; every other assembly gets one fixed 1-4 mm offset (seed 1000 + id). D1 parameters, no tuning here.
#   stage 1  plain on every assembly (doubles as the smoke test: load / grasp / feasibility)
#   stage 2  c3 (plain + MF3 + gate 3 mm) on every assembly whose plain run got past the grasp
#   stage 3  repeats (rep2, rep3) of both arms on assemblies where either arm failed: noise estimate
# Resumable: a job with res.json (or an "excluded" marker) is skipped; rerun the same command to continue.
# Deadline: no job starts after DEADLINE (epoch s; default 05:45 tomorrow) so nothing runs at the 06:30 robot backstop.
#   setsid nohup scripts/queue_h3_100.sh ~/ci_mpc_runs/h3_100 > ~/ci_mpc_runs/h3_100.log 2>&1 < /dev/null &
# WAIT_FOR (env): "file|pattern" to wait for before starting (default: the C3 cross-task queue).
HERE=$(cd "$(dirname "$0")" && pwd)
export WRIST_FT=1  # true contact force (wrist_ft.csv): the observer peak is stiction-biased (PROGRESS 2026-10-28)
OUT=$1; mkdir -p "$OUT"
DEADLINE=${DEADLINE:-$(date -d 'tomorrow 05:45' +%s)}
PAR=${PAR:-3}
WAIT_FOR=${WAIT_FOR-"$HOME/ci_mpc_runs/cross_c3.log|cross-task queue done"}  # WAIT_FOR="" = no wait
COMMON=(--timeout-contact 150 --max-probes 25 --track-height 5 --centering-bound 0.008 --stuck-dither 0.0008)
C3=(--entry c3 --c3-arm-friction 2.2,1.9,2.2,0.05,0.05 --c3-stiction-ff 2.9,2.7 --c3-std-max 0.003)
H3="00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041"
log() { echo "$(date '+%F %T') $*"; }
if [ -n "$WAIT_FOR" ]; then
  until grep -q "${WAIT_FOR#*|}" "${WAIT_FOR%%|*}" 2>/dev/null; do sleep 60; done
fi
ALL=$(ls ~/ci_mpc_assets/automate | grep -E '^[0-9]{5}$')
offset() {  # assembly -> "dx,dy" [mm]
  python3 -c "
import numpy as np
h3 = '$H3'.split(); a = '$1'
r = np.random.default_rng(300 + h3.index(a) if a in h3 else 1000 + int(a)); t = r.uniform(0, 2 * np.pi); m = r.uniform(1, 4)
print(f'{m * np.cos(t):.2f},{m * np.sin(t):.2f}')"
}
i=0
run() {  # assembly arm rep
  local a=$1 arm=$2 rep=$3 d="$OUT/$2/rep$3/$1"
  [ -f "$d/res.json" ] || [ -f "$d/excluded" ] && return 0
  if [ "$(date +%s)" -gt "$DEADLINE" ]; then log "deadline: skip $a $arm rep$rep"; return 1; fi
  while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt 8 ]; do sleep 20; done
  while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
  local off; off=$(offset "$a"); extra=(); [ "$arm" = c3 ] && extra=("${C3[@]}")
  mkdir -p "$d"; log "$a $arm rep$rep offset $off"
  (AUTOMATE=$a LOG="$d/isaac.log" ROS_DOMAIN_ID=$((200 + i % 30)) taskset -c 16-31 timeout 1800 "$HERE/demo_automate.sh" \
     --headless --offset "$off" --strategy probe "${COMMON[@]}" "${extra[@]}" --log-dir "$d" --out "$d/res.json" \
     > "$d/run.out" 2>&1; [ -f "$d/res.json" ] || log "no res.json: $a $arm rep$rep (exit $?)") &
  i=$((i + 1)); sleep 15
}
grasped() { grep -q "grasped: fingers" "$OUT/plain/rep1/$1/run.out" 2>/dev/null; }
ok() { python3 -c "import json,sys; sys.exit(0 if json.load(open('$1')).get('success') else 1)" 2>/dev/null; }
log "H3-100: $(echo $ALL | wc -w) assemblies, deadline $(date -d @$DEADLINE '+%F %T')"
for a in $ALL; do run "$a" plain 1 || break; done; wait
for a in $ALL; do
  if ! grasped "$a"; then [ -f "$OUT/plain/rep1/$a/run.out" ] && touch "$OUT/plain/rep1/$a/no_grasp"; continue; fi
  run "$a" c3 1 || break
done; wait
for rep in 2 3; do
  for a in $ALL; do
    grasped "$a" || continue
    ok "$OUT/plain/rep1/$a/res.json" && ok "$OUT/c3/rep1/$a/res.json" && continue
    run "$a" plain $rep || break 2; run "$a" c3 $rep || break 2
  done
done; wait
log "h3-100 queue done"
