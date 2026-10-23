#!/bin/bash
# Held-out benchmark H1 (weekend): peg-in-hole under conditions nothing was tuned on, each at 3 random hole errors
# (r 1-4 mm, random direction, fixed seed per condition), strategies probe and spiral, pinned to cores 0-15.
# Never tune on these results. Waits for a running track-A driver (repeat_benchmark.sh) to finish first.
#   nohup scripts/heldout_h1.sh OUT_DIR > OUT_DIR/driver.log 2>&1 &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1
mkdir -p "$OUT"
while pgrep -f "^/bin/bash .*repeat_benchmark\.sh" > /dev/null; do sleep 60; done
offsets() {  # 3 random offsets "dx,dy" [mm] for a seed
  python3 -c "
import numpy as np
r = np.random.default_rng($1)
print(' '.join(f'{a:.2f},{b:.2f}' for a, b in (r.uniform(1, 4) * np.array([np.cos(t), np.sin(t)]) for t in r.uniform(0, 2 * np.pi, 3))))"
}
run() {  # name seed env... -- demo options...
  name=$1; seed=$2; shift 2
  envs=(); while [ $# -gt 0 ] && [ "$1" != "--" ]; do envs+=("$1"); shift; done; shift
  for s in probe spiral; do
    extra=(); [ "$s" = probe ] && extra=(--max-probes 25)
    echo "$(date '+%F %T') $name $s"
    env "${envs[@]}" OFFSETS="$(offsets "$seed")" taskset -c 0-15 "$HERE/peg_benchmark.sh" "$OUT/$name" "$s" \
      --timeout-contact 150 "${extra[@]}" "$@" > "$OUT/${name}_$s.txt" 2>&1
  done
}
run friction_0.5 11 PEG_FRICTION=0.5 --
run friction_1.0 12 PEG_FRICTION=1.0 --
run hole_tilt_2deg 13 PEG_HOLE_TILT=2.0 --
run grasp_offset 14 -- --grasp-offset 1.5,1.0
run hole_elsewhere 15 PEG_HOLE_XY=0.45,-0.05 --
echo "$(date '+%F %T') H1 done"
grep -h "inserted" "$OUT"/*.txt
