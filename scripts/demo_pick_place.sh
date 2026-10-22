#!/bin/bash
# Pick-and-place demo in Isaac Sim with the stage-1 MPC (blended motion, grasp planned from the detection).
#
#   scripts/demo_pick_place.sh                 # rotated 3 x 5 x 4 cm box, Isaac window open
#   scripts/demo_pick_place.sh cube            # 4 cm cube
#   scripts/demo_pick_place.sh box --noise     # with a realistic estimation error (3 mm, 2 mm, 3 deg)
#   scripts/demo_pick_place.sh box --stops     # stop at every waypoint instead of blending
#   scripts/demo_pick_place.sh box --headless  # no window (faster)
# Extra options are passed to scripts/pick_place_demo.py (e.g. --pos-noise 0.006 --seed 2).
HERE=$(cd "$(dirname "$0")" && pwd)

object=${1:-box}
[ $# -gt 0 ] && shift
case "$object" in
  box)  export OBJECT_SIZE=0.03,0.05,0.04 OBJECT_YAW=30 ;;
  cube) export OBJECT_SIZE=0.04,0.04,0.04 OBJECT_YAW=0 ;;
  *) echo "usage: $0 [box|cube] [--noise] [--stops] [--headless] [pick_place_demo.py options]"; exit 1 ;;
esac

export GUI=1
extra=()
for arg in "$@"; do
  case "$arg" in
    --noise)    extra+=(--pos-noise 0.003 --size-noise 0.002 --yaw-noise-deg 3 --seed 1) ;;
    --stops)    extra+=(--mode stops) ;;
    --headless) unset GUI ;;
    *)          extra+=("$arg") ;;
  esac
done

export PICK_PLACE=1
export CONTROLLER_CONFIG=$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml
export CHECKER=$HERE/pick_place_demo.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

view=headless; [ -n "${GUI:-}" ] && view=window
echo "pick-and-place demo: $object (size $OBJECT_SIZE m, yaw $OBJECT_YAW deg), $view, options: ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${extra[@]}"
