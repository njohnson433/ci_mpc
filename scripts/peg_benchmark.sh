#!/bin/bash
# Peg-in-hole benchmark in Isaac (Factory 8 mm peg): one contact strategy at a set of hole position errors
# (perception error, mm), N runs in parallel (own ROS domains), then a summary table from the res.json files.
#   scripts/peg_benchmark.sh OUT_DIR STRATEGY [extra peg_insert_demo.py options]
#   e.g. scripts/peg_benchmark.sh /tmp/bench probe --timeout-contact 150 --max-probes 25
# OFFSETS (env) overrides the default set; PARALLEL (env, default 4) the batch size; DOMAIN_BASE (env, default 60)
# the first ROS domain (parallel benchmarks need disjoint ranges).
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; STRATEGY=$2; shift 2
OFFSETS=${OFFSETS:-"0,0 2,0 3,0 -3,0 5,0 -5,0 0,4 3,3 -4,2 -3,-2"}
PARALLEL=${PARALLEL:-4}
mkdir -p "$OUT"
i=0
for off in $OFFSETS; do
  # each Isaac instance takes ~3-4 GB: never start one below MIN_MEM_GB available (an OOM on 2026-10-23 killed
  # every weekend job at once)
  while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt "${MIN_MEM_GB:-7}" ]; do sleep 20; done
  name="${STRATEGY}_${off/,/_}"
  dom=$((${DOMAIN_BASE:-60} + i % PARALLEL))
  mkdir -p "$OUT/$name"
  (LOG="$OUT/$name/isaac.log" ROS_DOMAIN_ID=$dom timeout 2400 "$HERE/demo_peg.sh" --headless --offset "$off" \
     --strategy "$STRATEGY" --log-dir "$OUT/$name" --out "$OUT/$name/res.json" "$@" > "$OUT/$name/run.out" 2>&1) &
  i=$((i + 1))
  sleep 15
  if [ $((i % PARALLEL)) -eq 0 ]; then wait; fi
done
wait
python3 - "$OUT" "$STRATEGY" <<'EOF'
import glob, json, os, sys
out, strategy = sys.argv[1], sys.argv[2]
rows = []
for d in sorted(glob.glob(os.path.join(out, strategy + "_*"))):
    f = os.path.join(d, "res.json")
    if not os.path.exists(f):
        rows.append((os.path.basename(d), "no result", "", "", ""))
        continue
    r = json.load(open(f))
    rows.append((os.path.basename(d), "OK" if r["success"] else "FAIL", f"{r['depth_err_mm']:.2f}", f"{r['xy_err_mm']:.2f}",
                 f"{r['contact_phase_s']:.1f}"))
ok = sum(r[1] == "OK" for r in rows)
print(f"{strategy}: {ok}/{len(rows)} inserted")
print(f"{'run':16s} {'result':10s} {'depth err':>9s} {'xy err':>7s} {'contact s':>9s}")
for r in rows:
    print(f"{r[0]:16s} {r[1]:10s} {r[2]:>9s} {r[3]:>7s} {r[4]:>9s}")
EOF
