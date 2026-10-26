#!/bin/bash
# Live dashboard for the Isaac benchmark chain (B2 -> calib_check -> A2/B4/B5 -> H3_fix by default): driver status, every run's result,
# per-repeat tallies, GPU / RAM. Read-only; Ctrl-C to quit.
#   scripts/watch_runs.sh [RUN_ROOT] [JOB ...]      e.g. scripts/watch_runs.sh ~/ci_mpc_runs/weekend B2 B3 H3_fix
# INTERVAL (env, default 20 s) the refresh period.
ROOT=${1:-$HOME/ci_mpc_runs/weekend}; shift
JOBS=${*:-RA_hand RA_v3 RB_hand RB_v3}
while true; do
  out=$(
    echo "ci_mpc runs  $(date '+%F %T')  root: $ROOT"
    echo
    nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null \
      | awk -F', ' '{printf "GPU %s %%  %s / %s MiB   ", $1, $2, $3}'
    awk '/MemTotal/ {t=$2} /MemAvailable/ {a=$2} END {printf "RAM free %.1f / %.1f GB   ", a/1048576, t/1048576}' /proc/meminfo
    echo "Isaac instances: $(pgrep -c -f 'isaac-sim|isaacsim|kit/python' 2>/dev/null || echo 0)"
    for job in $JOBS; do
      echo
      last=$(tail -n 1 "$ROOT/$job.log" 2>/dev/null); echo "== $job  (${last:-waiting for the previous job})"
      # these are evaluations of a fixed controller/planner, not training: say which method this job tests
      case $job in
        B2) echo "   method: probe search + GPU sampling MPC entry (belief fix), Isaac 8 mm peg, 10 offsets x 3" ;;
        calib_check) echo "   method: gate for the grasp-height calibration touch (3 offsets); the queue aborts if not 3/3 calibrated" ;;
        A2) echo "   method: plain probe search, with the calibration touch (baseline; before: 24/30)" ;;
        B4) echo "   method: probe search + GPU sampling MPC entry, with the calibration touch" ;;
        B5) echo "   method: probe search + sample-then-refine MPC entry (margin -1), with the calibration touch" ;;
        B6) echo "   method: as B4, but engaged = tip sink only (--engage-lateral inf)" ;;
        B7) echo "   method: as B5, but engaged = tip sink only (--engage-lateral inf)" ;;
        B8) echo "   method: as B4, but a lateral push centers (plain probe), only a tip sink starts the GPU phase" ;;
        B9) echo "   method: as B5, but a lateral push centers (plain probe), only a tip sink starts the GPU phase" ;;
        A3) echo "   method: plain probe + grasp-height tracking (--track-height 5); compare A2 29/30" ;;
        B10) echo "   method: as B8 (GPU entry, lateral centering) + grasp-height tracking" ;;
        A4) echo "   method: plain probe + learned probe likelihood (randomized sensor errors); compare A2 29/30" ;;
        B11) echo "   method: as B8 (GPU entry, lateral centering) + learned probe likelihood; compare B8 28/30" ;;
        A5) echo "   method: as A4, likelihood with the distance-prior correction (rcorr)" ;;
        B12) echo "   method: as B11, likelihood with the distance-prior correction (rcorr)" ;;
        A6) echo "   method: plain probe + learned (rcorr) + height tracking + centering bound (combined candidate)" ;;
        B13) echo "   method: GPU entry + lateral centering + learned (rcorr) + height tracking + centering bound" ;;
        A7) echo "   method: as A6 with the v2 likelihood (logs + randomized probe sweeps)" ;;
        B14) echo "   method: as B13 with the v2 likelihood (logs + randomized probe sweeps)" ;;
        RA_hand) echo "   method: HELD-OUT random offsets: plain probe, hand model (+ tracking + bound)" ;;
        RA_v3) echo "   method: HELD-OUT random offsets: plain probe, v3 learned likelihood + stuck dither (+ tracking + bound)" ;;
        RB_hand) echo "   method: HELD-OUT random offsets: GPU entry + lateral centering, hand model" ;;
        RB_v3) echo "   method: HELD-OUT random offsets: GPU entry + lateral centering, v3 learned + stuck dither" ;;
        B3) echo "   method: probe search + sample-then-refine MPC entry (GPU sampler + C3 QP, margin -1), Isaac peg" ;;
        H3*) echo "   method: held-out AutoMate assemblies (12), spiral vs probe vs probe + GPU, entry-plane fix" ;;
      esac
      cmd=$(ps -eo args | grep -F "$ROOT/$job/" | grep -E "peg_benchmark|heldout_|demo_" | grep -v grep | head -n 1)
      [ -n "$cmd" ] && echo "   now: $(echo "$cmd" | sed -E 's| [^ ]*/ci_mpc_runs/[^ ]*||g; s|/[^ ]*/||g; s|^bash ||' | cut -c1-110)"
      [ -d "$ROOT/$job" ] || { echo "   waiting to start"; continue; }
      python3 - "$ROOT/$job" <<'EOF'
import glob, json, os, sys, time
root = sys.argv[1]
# a run dir is any directory holding run.out (repeat dirs rep1/rep2/..., or flat as in H3)
runs = sorted({os.path.dirname(p) for p in glob.glob(f"{root}/**/run.out", recursive=True)})
groups = {}
tot_ok = tot_n = tot_started = 0
for d in runs:
    rel = os.path.relpath(d, root)
    grp = rel.split(os.sep)[0] if os.sep in rel else "."
    groups.setdefault(grp, []).append((rel, d))
for grp, items in groups.items():
    ok = n = 0
    rows = []
    for rel, d in items:
        res = os.path.join(d, "res.json")
        if os.path.exists(res):
            try:
                r = json.load(open(res))
            except Exception:
                rows.append(f"   {rel:<34} res.json unreadable"); continue
            n += 1; ok += bool(r.get("success"))
            rows.append(f"   {rel:<34} {'OK  ' if r.get('success') else 'FAIL'}  xy {r.get('xy_err_mm', float('nan')):5.2f} mm"
                        f"  depth {r.get('depth_err_mm', float('nan')):5.2f} mm  {r.get('total_s', 0):6.1f} s")
        else:
            age = time.time() - os.path.getmtime(os.path.join(d, "run.out"))
            err = ""
            try:
                txt = open(os.path.join(d, "run.out"), errors="ignore").read()
                if any(k in txt for k in ("Traceback", "Killed", "out of memory")): err = "  !! error in run.out"
            except Exception:
                pass
            rows.append(f"   {rel:<34} running ({age/60:.1f} min since last output){err}")
    tot_ok += ok; tot_n += n; tot_started += len(items)
    pct = f"{100 * ok / n:.0f} %" if n else "-"
    print(f"  [{grp}] success {ok}/{n} = {pct}  ({len(items)} started)")
    print("\n".join(rows))
if tot_n:
    print(f"  TOTAL success {tot_ok}/{tot_n} = {100 * tot_ok / tot_n:.0f} %  ({tot_started} started)"
          + ("   [baseline: plain probe 24/30 = 80 %]" if os.path.basename(root) in ("B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9", "B10", "B11", "B12", "B13", "B14") else ""))
EOF
    done
  )
  clear; echo "$out"
  sleep "${INTERVAL:-20}"
done
