#!/bin/bash
# Real FR3: one session for the C3 stiction measurement (ci_mpc/docs/REAL_ROBOT.md 5b). Run it in your own
# terminal with the enabling device in hand; runs straight through, no questions (CONFIRM=1: ask once before
# the controller is activated).
#   1. franka_bringup franka.launch.py (franka_hardware, no controller: the arm does not move)
#   2. read-only check: pose, /joint_states rate, frame (c3_breakaway_real.py --dry-run)
#   3. activate cartesian_impedance_controller (config/real_robot_controllers.yaml): the arm holds its pose
#   4. hold:    hold test (+ optional --goto), no force ramps
#      measure: c3_breakaway_real.py
#   5. on exit (also Ctrl-C): deactivate the controller, stop the launch
#
# Usage: c3_breakaway_session.sh hold|measure [args for c3_breakaway_real.py]
#   c3_breakaway_session.sh hold
#   c3_breakaway_session.sh hold --goto 0.307,0,0.487
#   c3_breakaway_session.sh measure --goto 0.5,0.1,0.28 --dirs +x,-x,+y,-y,+z,-z,+rx,-rx,+ry,-ry \
#       --out ~/ci_mpc_runs/real/breakaway/probe_cold
# Env: ROBOT_IP (192.168.1.200), ROS_DOMAIN_ID (61), EE_PROFILE (the Desk profile next to this script).
MODE=$1; shift
case "$MODE" in hold|measure) ;; *) sed -n 2,17p "$0"; exit 1 ;; esac
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT_IP=${ROBOT_IP:-192.168.1.200}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
EE_PROFILE=${EE_PROFILE:-$HERE/desk_ee_hand_camera_mount_v0.1.endeffector-profile.json}
PARAMS=~/franka_ros2_ws/src/cartesian_impedance_control/config/real_robot_controllers.yaml
LOG=~/ci_mpc_runs/real/session_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"

source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -d ~/.local/ros_numpy1 ] && export PYTHONPATH=~/.local/ros_numpy1:$PYTHONPATH
BRK="python3 -u $HERE/c3_breakaway_real.py --ee-profile $EE_PROFILE"
BRK_GO="$BRK --yes"  # the session already asked

ping -c 1 -W 1 "$ROBOT_IP" > /dev/null || { echo "robot $ROBOT_IP not reachable"; exit 1; }
[ "$(ulimit -r)" = 0 ] && echo "warning: no realtime priority (ulimit -r 0): sudo usermod -aG realtime $USER, log in again"

LAUNCH_PID=
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "--- deactivating the controller, stopping the launch"
    timeout 10 ros2 control set_controller_state cartesian_impedance_controller inactive > /dev/null 2>&1
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null   # the launch's own process group (setsid)
    for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  fi
  echo "logs: $LOG"
}
trap cleanup EXIT INT TERM

echo "--- 1. franka_hardware on $ROBOT_IP (ROS_DOMAIN_ID $ROS_DOMAIN_ID), log $LOG/launch.log"
setsid ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:="$ROBOT_IP" load_gripper:=false \
  joint_state_rate:=200 use_rviz:=false > "$LOG/launch.log" 2>&1 &
LAUNCH_PID=$!
for i in $(seq 60); do
  timeout 5 ros2 control list_controllers > "$LOG/controllers.txt" 2>&1 && grep -q joint_state_broadcaster "$LOG/controllers.txt" && break
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "launch exited:"; grep -E "ERROR|rror" "$LOG/launch.log" | head; exit 1; }
  sleep 1
done
grep -q joint_state_broadcaster "$LOG/controllers.txt" || { echo "controller manager not up after 60 s"; exit 1; }
cat "$LOG/controllers.txt"

echo "--- 2. read-only check"
$BRK --dry-run "$@" || exit 1

echo "--- 3. activate cartesian_impedance_controller: the arm holds its current pose (K 2500/2500/1000)"
if [ -n "$CONFIRM" ]; then
  read -r -p "enabling device in hand, activate and run (${MODE}${*:+ $*})? type 'go': " ans
  [ "$ans" = go ] || exit 1
fi
ros2 run controller_manager spawner cartesian_impedance_controller -c /controller_manager --param-file "$PARAMS" \
  > "$LOG/spawner.log" 2>&1 || { echo "spawner failed:"; tail -5 "$LOG/spawner.log"; exit 1; }
ros2 control list_controllers

if [ "$MODE" = hold ]; then
  echo "--- 4. hold test (no force ramps)"
  $BRK_GO --repeats 0 --out "$LOG/hold" "$@" 2>&1 | tee "$LOG/breakaway.out"
  read -r -p "arm holding; nudge test now if wanted. Enter to stop: " _
else
  echo "--- 4. measurement"
  $BRK_GO "$@" 2>&1 | tee "$LOG/breakaway.out"
fi
