#!/bin/bash
# Box stacking demo in Isaac Sim with the stage-1 MPC: three 4 cm cubes, box1 (green) stays, box0 (red) and
# box2 (blue) are stacked on it, each placed on the detected top box with matching yaw.
#
#   scripts/demo_stack.sh               # Isaac window open
#   scripts/demo_stack.sh --noise       # realistic estimation error (3 mm, 2 mm, 3 deg per box)
#   scripts/demo_stack.sh --camera      # perception from the simulated Gemini 305 on the wrist (box detector on
#                                       # color + noisy depth) instead of ground truth
#   scripts/demo_stack.sh --headless    # no window (faster)
# Extra options are passed to scripts/stack_demo.py (e.g. --order box2,box0).
HERE=$(cd "$(dirname "$0")" && pwd)

export OBJECT_SIZE=0.04,0.04,0.04 OBJECT_YAW=20   # box0 at the pick spot (0.45, -0.12)
export BOXES="0.45,0.12,10;0.56,-0.01,-25"         # box1 (base), box2
export GUI=1
extra=()
for arg in "$@"; do
  case "$arg" in
    --noise)    extra+=(--pos-noise 0.003 --size-noise 0.002 --yaw-noise-deg 3 --seed 1) ;;
    --camera)   export CAMERA=1; extra+=(--perception camera) ;;
    --headless) unset GUI ;;
    *)          extra+=("$arg") ;;
  esac
done

export PICK_PLACE=1
export CONTROLLER_CONFIG=$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml
export CHECKER=$HERE/stack_demo.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

view=headless; [ -n "${GUI:-}" ] && view=window
echo "stacking demo: boxes at (0.45,-0.12,${OBJECT_YAW}) and $BOXES, $view, options: ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${extra[@]}"
