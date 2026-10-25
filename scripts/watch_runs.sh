#!/bin/bash
# Live dashboard for the Isaac benchmark chain (B2 -> B3 -> H3_fix by default): driver status, every run's result,
# per-repeat tallies, GPU / RAM. Read-only; Ctrl-C to quit.
#   scripts/watch_runs.sh [RUN_ROOT] [JOB ...]      e.g. scripts/watch_runs.sh ~/ci_mpc_runs/weekend B2 B3 H3_fix
# INTERVAL (env, default 20 s) the refresh period.
ROOT=${1:-$HOME/ci_mpc_runs/weekend}; shift
JOBS=${*:-B2 B3 H3_fix}
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
      [ -d "$ROOT/$job" ] || { echo "   waiting to start"; continue; }
      python3 - "$ROOT/$job" <<'EOF'
import glob, json, os, sys, time
root = sys.argv[1]
# a run dir is any directory holding run.out (repeat dirs rep1/rep2/..., or flat as in H3)
runs = sorted({os.path.dirname(p) for p in glob.glob(f"{root}/**/run.out", recursive=True)})
groups = {}
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
    print(f"  [{grp}] {ok}/{n} succeeded of {n} finished, {len(items)} started")
    print("\n".join(rows))
EOF
    done
  )
  clear; echo "$out"
  sleep "${INTERVAL:-20}"
done
