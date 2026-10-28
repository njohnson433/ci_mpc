#!/bin/bash
# Factory gear mesh (held-out H2): pick the medium gear, put it on the middle shaft of the gear base (it must mesh
# with the small and large gears). Same demo and strategies as the peg (scripts/peg_insert_demo.py), the contact
# model geometry set to the gear bore on its shaft (bore r 5.0 mm, shaft r 4.75 mm, 20 mm travel).
#
#   scripts/demo_gear.sh                    # shaft detected exactly, Isaac window open
#   scripts/demo_gear.sh --offset 2,1       # perception error of the shaft position [mm]
#   scripts/demo_gear.sh --headless
#   GEAR_YAW_ERROR=1.5 scripts/demo_gear.sh # the gear arrives 1.5 deg off the meshing yaw
HERE=$(cd "$(dirname "$0")" && pwd)
export GUI=1
extra=(--geo 0.005,0.0005,0.020,0.00475,0.0001 --open-width 0.05 --grasp-depth 0.004)
while [ $# -gt 0 ]; do
  case "$1" in
    --offset)   extra+=("--hole-offset=$2"); shift ;;
    --headless) unset GUI ;;
    *)          extra+=("$1") ;;
  esac
  shift
done
export GEAR=1
export CONTROLLER_CONFIG=${CONTROLLER_CONFIG:-$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml}
export CHECKER=$HERE/peg_insert_demo.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
view=headless; [ -n "${GUI:-}" ] && view=window
echo "gear mesh demo: $view, options: ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${extra[@]}"
