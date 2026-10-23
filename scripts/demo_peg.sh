#!/bin/bash
# Factory peg-in-hole (8 mm round peg, SDF collision) in Isaac Sim with the stage-1 MPC (free space) and a
# compliant search-and-insert contact phase on the impedance controller.
#
#   scripts/demo_peg.sh                    # hole detected exactly, Isaac window open
#   scripts/demo_peg.sh --offset 2,1       # perception error of the hole position [mm]
#   scripts/demo_peg.sh --headless
# Extra options are passed to scripts/peg_insert_demo.py (e.g. --stiffness 400,400,500 --push 5).
HERE=$(cd "$(dirname "$0")" && pwd)
export GUI=1
extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --offset)   extra+=("--hole-offset=$2"); shift ;;
    --headless) unset GUI ;;
    *)          extra+=("$1") ;;
  esac
  shift
done
export PEG=1
export CONTROLLER_CONFIG=$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml
export CHECKER=$HERE/peg_insert_demo.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
view=headless; [ -n "${GUI:-}" ] && view=window
echo "peg-in-hole demo: $view, options: ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${extra[@]}"
