#!/bin/bash
# Held-out benchmark H2 (weekend): Factory gear mesh (scripts/demo_gear.sh): put the medium gear on its shaft, in
# mesh with the small and large gears. Bore r 5.0 mm on a 4.75 mm shaft (0.25 mm clearance vs the peg's 0.5), a
# yaw tolerance < 2 deg (teeth without chamfers), contact also with the neighbour gears' teeth. Strategies were
# never tuned on it (only the grasp was set up: 50 mm opening, TCP 4 mm below the hub top).
#   cond nominal:   5 random shaft-position errors (r 1-4 mm, seed 21): spiral, probe, probe --entry gpu
#   cond yaw1/yaw3: the gear 1 / 3 deg off the meshing yaw, 3 errors (seed 22 / 23): spiral, probe
# Waits for scripts/run_weekend.sh to finish (its log's "all done" line). Cores 0-15, PARALLEL 3.
#   setsid nohup scripts/heldout_h2.sh ~/ci_mpc_runs/weekend/H2 > ~/ci_mpc_runs/weekend/H2/driver.log 2>&1 < /dev/null &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1
mkdir -p "$OUT"
until grep -q "all done" ~/ci_mpc_runs/weekend/run_weekend.log 2>/dev/null; do sleep 60; done
offsets() {  # n random offsets "dx,dy" [mm] for a seed
  python3 -c "
import numpy as np
r = np.random.default_rng($1)
print(' '.join(f'{a:.2f},{b:.2f}' for a, b in (r.uniform(1, 4) * np.array([np.cos(t), np.sin(t)]) for t in r.uniform(0, 2 * np.pi, $2))))"
}
run() {  # name seed n env... -- strategy specs (strategy[:extra option])
  name=$1; seed=$2; n=$3; shift 3
  envs=(); while [ $# -gt 0 ] && [ "$1" != "--" ]; do envs+=("$1"); shift; done; shift
  for spec in "$@"; do
    s=${spec%%:*}; tag=$s; extra=()
    [ "$s" = probe ] && extra=(--max-probes 25)
    [ "$spec" != "$s" ] && extra+=(${spec#*:}) && tag=${s}_gpu
    echo "$(date '+%F %T') $name $tag"
    env "${envs[@]}" DEMO="$HERE/demo_gear.sh" PARALLEL=3 DOMAIN_BASE=60 OFFSETS="$(offsets "$seed" "$n")" taskset -c 0-15 \
      "$HERE/peg_benchmark.sh" "$OUT/$name/$tag" "$s" --timeout-contact 150 "${extra[@]}" > "$OUT/${name}_$tag.txt" 2>&1
  done
}
run nominal 21 5 -- spiral probe "probe:--entry gpu"
run yaw1 22 3 GEAR_YAW_ERROR=1.0 -- spiral probe
run yaw3 23 3 GEAR_YAW_ERROR=3.0 -- spiral probe
echo "$(date '+%F %T') H2 done"
grep -h "inserted" "$OUT"/*.txt
