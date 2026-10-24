#!/bin/bash
# Held-out benchmark H3 (weekend): AutoMate plug/socket assemblies (scripts/demo_automate.sh). 12 assemblies drawn
# with seed 31 from the 56 of 100 that pass setup-only criteria (top-down grasp <= 40 mm wide, TCP >= 10 mm above
# the socket when seated, travel <= 40 mm). The plug starts seated in a source socket. Shapes are not round: the
# probe / GPU contact model is a round-peg guess from the plug tip. Nothing was tuned on these 12 (dev: 00015).
# Each assembly: 1 random hole error (r 1-4 mm, seed 300 + k), strategies spiral, probe, probe --entry gpu.
# Waits for heldout_h2.sh to finish (its log's "H2 done"). Cores 0-15, 3 in parallel (the 3 strategies).
#   setsid nohup scripts/heldout_h3.sh ~/ci_mpc_runs/weekend/H3 > ~/ci_mpc_runs/weekend/H3/driver.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1
mkdir -p "$OUT"
ASSEMBLIES="00016 00021 00028 00211 00301 00319 00388 00499 00638 00652 00700 01041"
until grep -q "H2 done" ~/ci_mpc_runs/weekend/H2/driver.log 2>/dev/null; do sleep 60; done
k=0
for a in $ASSEMBLIES; do
  off=$(python3 -c "
import numpy as np
r = np.random.default_rng(300 + $k); t = r.uniform(0, 2 * np.pi); m = r.uniform(1, 4)
print(f'{m * np.cos(t):.2f},{m * np.sin(t):.2f}')")
  echo "$(date '+%F %T') $a offset $off"
  i=0
  for spec in spiral probe "probe:--entry gpu"; do
    s=${spec%%:*}; tag=$s; extra=()
    [ "$s" = probe ] && extra=(--max-probes 25)
    [ "$spec" != "$s" ] && extra+=(${spec#*:}) && tag=gpu
    d="$OUT/$a/$tag"; mkdir -p "$d"
    while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt 7 ]; do sleep 20; done
    (AUTOMATE=$a LOG="$d/isaac.log" ROS_DOMAIN_ID=$((60 + i)) taskset -c 0-15 timeout 2400 "$HERE/demo_automate.sh" --headless \
       --offset "$off" --strategy "$s" --timeout-contact 150 --log-dir "$d" --out "$d/res.json" "${extra[@]}" \
       > "$d/run.out" 2>&1) &
    i=$((i + 1)); sleep 15
  done
  wait
  k=$((k + 1))
done
echo "$(date '+%F %T') H3 done"
python3 - "$OUT" <<'PY'
import glob, json, os, sys
out = sys.argv[1]
tags = ("spiral", "probe", "gpu")
tot = {t: [0, 0, 0] for t in tags}  # inserted, grasp/setup failures, runs
print(f"{'assembly':10s}" + "".join(f"{t:>14s}" for t in tags))
for a in sorted(os.listdir(out)):
    if not a.isdigit():
        continue
    row = f"{a:10s}"
    for t in tags:
        f = os.path.join(out, a, t, "res.json"); ro = os.path.join(out, a, t, "run.out")
        tot[t][2] += 1
        if os.path.exists(f):
            r = json.load(open(f)); tot[t][0] += r["success"]
            row += f"{('OK ' if r['success'] else 'FAIL ') + format(r['depth_err_mm'], '.1f'):>14s}"
        else:
            g = os.path.exists(ro) and "FAIL: grasp" in open(ro).read()
            tot[t][1] += g
            row += f"{'grasp fail' if g else 'no result':>14s}"
    print(row)
for t in tags:
    print(f"{t}: inserted {tot[t][0]}/{tot[t][2]} ({tot[t][1]} grasp failures)")
PY
