#!/bin/bash
# AutoMate plug/socket assembly (held-out H3): pick the plug, insert it into its socket. Same demo and strategies as
# the peg (scripts/peg_insert_demo.py); grasp depth, opening and the round-peg contact model guess come from
# scripts/isaac/automate_scene.py info() (the AutoMate grasp pose and insertion travel).
#
#   scripts/demo_automate.sh 00016 [--offset 2,1] [--headless] [demo options]
#   AUTOMATE=00016 scripts/demo_automate.sh ...   (as DEMO= for scripts/peg_benchmark.sh)
HERE=$(cd "$(dirname "$0")" && pwd)
if [[ "${1:-}" =~ ^[0-9]{5}$ ]]; then export AUTOMATE=$1; shift; fi  # or AUTOMATE=ID in the environment
export GUI=1
read -r GEO DEPTH WIDTH ENGAGE < <(~/IsaacLab/.venv/bin/python -c "
import runpy; i = runpy.run_path('$HERE/isaac/automate_scene.py', run_name='lib')['info']('$AUTOMATE')
print(','.join(f'{v:.5f}' for v in i['geo']), f\"{i['grasp_depth']:.4f}\", f\"{i['grasp_width']:.4f}\",
      f\"{min(0.003, 0.5 * i['entry']):.4f}\")" 2>/dev/null)
OPEN=$(python3 -c "print(f'{min(0.08, $WIDTH + 0.02):.3f}')")
extra=(--geo "$GEO" --grasp-depth "$DEPTH" --open-width "$OPEN" --engage "$ENGAGE")  # engaged: 3 mm in, or half a shallow entry
while [ $# -gt 0 ]; do
  case "$1" in
    --offset)   extra+=("--hole-offset=$2"); shift ;;
    --headless) unset GUI ;;
    *)          extra+=("$1") ;;
  esac
  shift
done
export CONTROLLER_CONFIG=${CONTROLLER_CONFIG:-$HOME/franka_ros2_ws/install/ci_mpc_sim_gripper/share/ci_mpc_sim_gripper/config/isaac_pick_place.yaml}
export CHECKER=$HERE/peg_insert_demo.py
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
view=headless; [ -n "${GUI:-}" ] && view=window
echo "AutoMate $AUTOMATE demo: $view, options: ${extra[*]}"
exec "$HERE/isaac/run_impedance_test.sh" "${extra[@]}"
