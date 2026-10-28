#!/bin/bash
# Real FR3: joint friction / armature identification session (ci_mpc_sysid, docs/SYSID.md). Run it in your own
# terminal with the enabling device in hand; no questions, runs straight through, Ctrl-C stops everything.
# Per pose: Franka's move_to_start controller drives the arm to the joint configuration, then JointIdController
# (ci_mpc_sysid/config/real_sysid.yaml: sweeps 0.05-0.4 rad/s over +-0.2 rad, torque ramps up to 3 Nm, sines)
# runs joint by joint while the other joints are held; finally fit_joint_id.py fits Coulomb / viscous / offset /
# static / armature per joint.
#
# Usage: joint_id_session.sh test | home | probe | side | all
#   test   home pose, joint 7 only (~2 min): first run, check that everything behaves
#   home   (0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4)                      TCP 0.307, 0, 0.487
#   probe  IK of the C3 probe pose (fr3_hand_tcp 0.5, 0.1, 0.28)     same posture as c3_breakaway_session.sh probe
#   side   (0.6, -0.4, 0, -2.4, 0, 2.0, pi/4)                         TCP 0.35, 0.24, 0.36 (other gravity load)
#   all    home, probe, side
# Env: JOINTS=4,5,6 (instead of all 7), ROBOT_IP (192.168.1.200), ROS_DOMAIN_ID (61),
#      NO_LAUNCH=1 (use a running Controller Manager, e.g. Isaac; no franka_bringup)
# Results: ~/ci_mpc_runs/real/joint_id/<pose>_<HHMM>/ (id.npz, fitted.yaml, fit.txt), session logs next to them.
PRESET=$1
HERE=$(cd "$(dirname "$0")" && pwd)
SYSID=$HERE/../../ci_mpc_sysid
declare -A POSES=(
  [home]="0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854"
  [probe]="0.043, -0.057, 0.153, -2.266, 0.011, 2.21, 0.975"
  [side]="0.6, -0.4, 0.0, -2.4, 0.0, 2.0, 0.7854"
)
case "$PRESET" in
  test) LIST=home; JOINTS=${JOINTS:-7} ;;
  home|probe|side) LIST=$PRESET ;;
  all) LIST="home probe side" ;;
  *) sed -n 2,20p "$0"; exit 1 ;;
esac
JOINTS=${JOINTS:-1,2,3,4,5,6,7}
ROBOT_IP=${ROBOT_IP:-192.168.1.200}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
OUT=~/ci_mpc_runs/real/joint_id; mkdir -p "$OUT"
LOG=$OUT/session_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"

source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
[ -d ~/.local/ros_numpy1 ] && export PYTHONPATH=~/.local/ros_numpy1:$PYTHONPATH
# a stale ros2 CLI daemon on this domain (started with another environment) breaks `ros2 control` / `ros2 param`
# ("xmlrpc Fault: !rclpy.ok()", 2026-10-28): restart it with this environment
ros2 daemon stop > /dev/null 2>&1

LAUNCH_PID=
cleanup() {
  trap - EXIT INT TERM
  for c in joint_id_controller move_to_start_example_controller; do
    timeout 10 ros2 control set_controller_state $c inactive > /dev/null 2>&1
  done
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "--- stopping the launch"
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null   # the launch's own process group (setsid)
    for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  fi
  grep -h "aborted by reflex" "$LOG/launch.log" 2>/dev/null | head -2 | sed 's/^/REFLEX: /'
  echo "logs: $LOG"
}
trap cleanup EXIT INT TERM

[ "$(ulimit -r)" = 0 ] && [ -z "$NO_LAUNCH" ] && echo "WARNING: no realtime priority (ulimit -r 0): log out and in again (realtime group), check: ulimit -r -> 99"
if [ -z "$NO_LAUNCH" ]; then
  ping -c 1 -W 1 "$ROBOT_IP" > /dev/null || { echo "robot $ROBOT_IP not reachable"; exit 1; }
  echo "--- franka_hardware on $ROBOT_IP (ROS_DOMAIN_ID $ROS_DOMAIN_ID), log $LOG/launch.log"
  setsid ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:="$ROBOT_IP" load_gripper:=false \
    joint_state_rate:=200 use_rviz:=false > "$LOG/launch.log" 2>&1 &
  LAUNCH_PID=$!
fi
for i in $(seq 120); do
  timeout 5 ros2 control list_controllers > "$LOG/controllers.txt" 2>&1 && grep -q joint_state_broadcaster "$LOG/controllers.txt" && break
  [ -n "$LAUNCH_PID" ] && ! kill -0 "$LAUNCH_PID" 2>/dev/null && { echo "launch exited:"; grep -E "ERROR|rror" "$LOG/launch.log" | head; exit 1; }
  sleep 1
done
grep -q joint_state_broadcaster "$LOG/controllers.txt" || { echo "controller manager not up"; exit 1; }

echo "--- error recovery (clears a reflex from an earlier run; harmless otherwise)"
timeout 20 ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery "{}" > "$LOG/recovery.txt" 2>&1 \
  || echo "error recovery did not answer (see $LOG/recovery.txt)"

move_to() {  # $1 pose name: joint-space move with Franka's move_to_start controller, then release it
  cat > "$LOG/mts_$1.yaml" <<EOF
move_to_start_example_controller:
  ros__parameters:
    robot_type: fr3
    k_gains: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
    d_gains: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]
    start_joint_configuration: [${POSES[$1]}]
EOF
  echo "--- move to $1: [${POSES[$1]}]"
  ros2 run controller_manager spawner move_to_start_example_controller -c /controller_manager \
    -t franka_example_controllers/MoveToStartExampleController --param-file "$LOG/mts_$1.yaml" \
    >> "$LOG/spawner.log" 2>&1 || { echo "move_to_start spawner failed:"; tail -5 "$LOG/spawner.log"; exit 1; }
  for i in $(seq 120); do
    ros2 param get /move_to_start_example_controller process_finished 2>/dev/null | grep -q True && break
    sleep 0.5
  done
  ros2 param get /move_to_start_example_controller process_finished 2>/dev/null | grep -q True \
    || { echo "move to $1 did not finish in 60 s"; exit 1; }
  ros2 control set_controller_state move_to_start_example_controller inactive > /dev/null
  ros2 control unload_controller move_to_start_example_controller > /dev/null
}

for pose in $LIST; do
  move_to "$pose" || exit 1
  D=$OUT/${pose}$([ "$PRESET" = test ] && echo _test)_$(date +%H%M); mkdir -p "$D"
  sed "s/^    joints: .*/    joints: [$JOINTS]/" "$SYSID/config/real_sysid.yaml" > "$D/params.yaml"
  echo "--- identification at $pose, joints $JOINTS -> $D"
  python3 -u "$SYSID/scripts/run_joint_id.py" --out "$D/id.npz" --param-file "$D/params.yaml" 2>&1 | tee "$D/run.txt"
  grep -q "RESULT DONE" "$D/run.txt" || { echo "identification at $pose did not finish (see $D/run.txt)"; exit 1; }
  python3 "$SYSID/scripts/fit_joint_id.py" "$D/id.npz" --out "$D/fitted.yaml" 2>&1 | tee "$D/fit.txt"
done
