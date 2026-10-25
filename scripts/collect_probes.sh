#!/bin/bash
# Randomized probe sweeps for the learned probe likelihood (instead of probes chosen by the benchmark strategies, whose
# positions are biased): per episode one grasp, the calibration touch, then N probes at positions sampled around the
# true hole (peg_insert_demo.py --collect). Randomized per episode (seeded; params.json in each episode dir):
#   hole error r 0-5 mm (the probe sampling is relative to the true hole, this only moves the probe-0 / calibration spot)
#   contact friction U(0.5, 1.0) (default 0.75); hole tilt 0-1.5 deg in a third of the episodes; grasp offset dx (across the fingers) / dz (depth) +-1 mm
#   torque-sensor model in the controller's observer (the FR3's EE force comes from link-side joint torque sensors):
#     noise U(0.02, 0.12) Nm, per-joint offsets U(-0.3, 0.3) Nm, observer gain U(20, 40)
# WORKERS instances in parallel on cores 0-15 (the benchmarks use 16-31), ROS domains 90+.
#   setsid nohup scripts/collect_probes.sh ~/ci_mpc_runs/likelihood/sweeps 40 > ~/ci_mpc_runs/likelihood/sweeps.log 2>&1 < /dev/null &
# EPISODES (arg 2) per worker; PROBES (env, default 40); WORKERS (env, default 2); SEED0 (env, default 1000).
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; EPISODES=${2:-40}
WORKERS=${WORKERS:-2}; PROBES=${PROBES:-40}; SEED0=${SEED0:-1000}
BASE_YAML=$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml
mkdir -p "$OUT"
worker() {
  local w=$1
  for e in $(seq 0 $((EPISODES - 1))); do
    local seed=$((SEED0 + 1000 * w + e)) d="$OUT/w${w}_e$(printf %03d "$e")"
    [ -f "$d/res.json" ] && continue  # resumable
    mkdir -p "$d"
    # episode parameters and the randomized controller YAML
    read -r off fric tilt grasp < <(python3 - "$seed" "$BASE_YAML" "$d" <<'EOF'
import json, re, sys
import numpy as np
seed, base, d = int(sys.argv[1]), sys.argv[2], sys.argv[3]
r = np.random.default_rng(seed)
m, t = r.uniform(0, 5), r.uniform(0, 2 * np.pi)
P = dict(offset=[round(m * np.cos(t), 2), round(m * np.sin(t), 2)], friction=round(r.uniform(0.5, 1.0), 3),
         tilt=round(r.uniform(0, 1.5), 2) if r.random() < 1 / 3 else 0.0,
         grasp=[round(v, 2) for v in r.uniform(-1, 1, 2)], noise=round(r.uniform(0.02, 0.12), 3),
         bias=[round(v, 3) for v in r.uniform(-0.3, 0.3, 7)], gain=round(r.uniform(20, 40), 1), seed=seed)
y = open(base).read()
y = re.sub(r"observer_torque_noise: .*", f"observer_torque_noise: {P['noise']}", y)
y = re.sub(r"observer_torque_bias: .*", f"observer_torque_bias: {P['bias']}", y)
y = re.sub(r"observer_gain: .*", f"observer_gain: {P['gain']}", y)
open(f"{d}/controller.yaml", "w").write(y)
json.dump(P, open(f"{d}/params.json", "w"), indent=1)
print(f"{P['offset'][0]},{P['offset'][1]} {P['friction']} {P['tilt']} {P['grasp'][0]},{P['grasp'][1]}")
EOF
)
    while [ "$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)" -lt 7 ]; do sleep 20; done
    echo "$(date '+%F %T') w$w e$e seed $seed offset $off friction $fric tilt $tilt grasp $grasp"
    CONTROLLER_CONFIG="$d/controller.yaml" PEG_FRICTION=$fric PEG_HOLE_TILT=$tilt LOG="$d/isaac.log" \
      ROS_DOMAIN_ID=$((90 + w)) taskset -c 0-15 timeout 3600 "$HERE/demo_peg.sh" --headless --offset "$off" \
      --strategy probe --collect "$PROBES" --max-probes "$PROBES" --collect-seed "$seed" --timeout-contact 1e9 \
      --grasp-offset="$grasp" --log-dir "$d" --out "$d/res.json" > "$d/run.out" 2>&1
  done
}
for w in $(seq 0 $((WORKERS - 1))); do worker "$w" & sleep 20; done
wait
echo "$(date '+%F %T') collect done"
