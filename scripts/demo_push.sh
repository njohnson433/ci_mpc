#!/bin/bash
# Stage-2 pushing demo in Isaac Sim: the pushing MPC (sticking contact, ci_mpc_planner push_node) pushes an
# 8 x 8 x 5 cm box across the table with a rod on the hand.
#
#   scripts/demo_push.sh              # straight 20 cm push, Isaac window open
#   scripts/demo_push.sh arc          # 90-degree left arc (radius 15 cm), contact planned off-center
#   scripts/demo_push.sh yaw10        # straight push, box starts rotated 10 degrees (the MPC turns it back)
#   scripts/demo_push.sh arc --headless
# Extra options are passed to scripts/push_test.py (e.g. --push-args speed:=0.04).
HERE=$(cd "$(dirname "$0")" && pwd)

case=${1:-line}
[ $# -gt 0 ] && shift
case "$case" in
  line)  task=(--task line) ;;
  arc)   task=(--task arc --push-args length:=0.3) ;;
  yaw10) task=(--task line); export PUSH_YAW=100 ;;
  *) echo "usage: $0 [line|arc|yaw10] [--headless] [push_test.py options]"; exit 1 ;;
esac

export GUI=1
extra=()
for arg in "$@"; do
  case "$arg" in
    --headless) unset GUI ;;
    *)          extra+=("$arg") ;;
  esac
done

export PUSH=1
export CHECKER=$HERE/push_test.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

view=headless; [ -n "${GUI:-}" ] && view=window
echo "pushing demo: $case, $view, options: ${task[*]} ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${task[@]}" "${extra[@]}"
