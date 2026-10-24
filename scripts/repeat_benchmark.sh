#!/bin/bash
# Repeated peg benchmark (weekend track A): REPEATS x peg_benchmark.sh for each strategy, pinned to cores 0-15 (the
# other half is for training / data workers), then one summary table (success per offset over the repeats, overall
# rate with a Wilson 95 % interval, median contact time). Survives session restarts when started with nohup.
#   nohup scripts/repeat_benchmark.sh OUT_DIR "probe spiral" 3 [extra demo options] > OUT_DIR/driver.log 2>&1 &
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; STRATEGIES=$2; REPEATS=${3:-3}; shift 3
mkdir -p "$OUT"
for r in $(seq "${START:-1}" "$REPEATS"); do  # START (env): first repeat (resume)
  for s in $STRATEGIES; do
    extra=()
    [ "$s" = "probe" ] && extra=(--max-probes 25)
    echo "$(date '+%F %T') repeat $r strategy $s"
    taskset -c 0-15 "$HERE/peg_benchmark.sh" "$OUT/rep$r" "$s" --timeout-contact 150 "${extra[@]}" "$@" \
      > "$OUT/rep${r}_$s.txt" 2>&1
  done
done
python3 - "$OUT" "$STRATEGIES" <<'EOF'
import glob, json, math, os, sys
from collections import defaultdict
out, strategies = sys.argv[1], sys.argv[2].split()


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, c - h), min(1.0, c + h))


lines = []
for s in strategies:
    per = defaultdict(list)
    for f in glob.glob(os.path.join(out, "rep*", f"{s}_*", "res.json")):
        off = os.path.basename(os.path.dirname(f))[len(s) + 1:]
        per[off].append(json.load(open(f)))
    k = sum(r["success"] for v in per.values() for r in v)
    n = sum(len(v) for v in per.values())
    lo, hi = wilson(k, n)
    lines.append(f"## {s}: {k}/{n} inserted ({100 * k / max(n, 1):.0f} %, 95 % CI {100 * lo:.0f}-{100 * hi:.0f} %)")
    lines.append(f"| offset | inserted | median contact s (successes) |")
    lines.append("|---|---|---|")
    for off in sorted(per):
        v = per[off]
        ok = [r for r in v if r["success"]]
        ts = sorted(r["contact_phase_s"] for r in ok)
        med = f"{ts[len(ts) // 2]:.1f}" if ts else "-"
        lines.append(f"| {off} | {len(ok)}/{len(v)} | {med} |")
    lines.append("")
open(os.path.join(out, "summary.md"), "w").write("\n".join(lines))
print("\n".join(lines))
EOF
