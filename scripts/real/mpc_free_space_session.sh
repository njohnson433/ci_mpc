#!/bin/bash
# Real FR3: first stage-1 MPC session in free space (no contact; ci_mpc/docs/REAL_ROBOT.md 5). Run it in your own
# terminal with the enabling device in hand; no questions, runs straight through, Ctrl-C stops everything.
#   1. franka_bringup franka.launch.py (franka_hardware, no controller: the arm does not move)
#   2. Franka's move_to_start controller drives the arm to the libfranka home pose in joint space (~0.4 rad/s)
#   3. activate cartesian_impedance_controller (config/real_robot_controllers.yaml): the arm holds its pose
#   4. mpc_free_space_real.py: starts ci_mpc_planner/mpc_node (config/mpc_real.yaml: v 0.1 m/s, a 0.5 m/s^2, small
#      box around home, Desk-profile EE frame) and runs the tests; stops it at the end or on any safety stop
#   5. on exit (also Ctrl-C): deactivate the controllers, stop the launch; reflexes and MPC command gaps from the log
#
# Usage: mpc_free_space_session.sh test | all | steps | rotate | circle   [args for mpc_free_space_real.py]
#   test    home + 10 s MPC hold (first run: frame check, solve times, command gaps)          ~1 min
#   all     hold, home, steps (+-5 cm in x / y / z and back), rotate (+-0.3 rad z, +-0.2 rad x), circle (r 4 cm)  ~4 min
#   steps | rotate | circle   hold + that test only
#   e.g. mpc_free_space_session.sh all --step 0.08 --path-speed 0.08
# Env: ROBOT_IP (192.168.1.200), ROS_DOMAIN_ID (61).
# Results: ~/ci_mpc_runs/real/mpc_free/<preset>_<HHMM>/ (mpc_free_space.json, raw.npz, mpc_node.log), session logs next
# to them.
PRESET=$1; shift
case "$PRESET" in
  test) TESTS=hold ;;
  all) TESTS=hold,home,steps,rotate,circle ;;
  steps|rotate|circle) TESTS=hold,home,$PRESET ;;
  *) sed -n 2,20p "$0"; exit 1 ;;
esac
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT_IP=${ROBOT_IP:-192.168.1.200}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
PARAMS=~/franka_ros2_ws/src/cartesian_impedance_control/config/real_robot_controllers.yaml
OUT=~/ci_mpc_runs/real/mpc_free; mkdir -p "$OUT"
LOG=$OUT/session_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
D=$OUT/${PRESET}_$(date +%H%M)

source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -d ~/.local/ros_numpy1 ] && export PYTHONPATH=~/.local/ros_numpy1:$PYTHONPATH

ping -c 1 -W 1 "$ROBOT_IP" > /dev/null || { echo "robot $ROBOT_IP not reachable"; exit 1; }
[ "$(ulimit -r)" = 0 ] && echo "WARNING: no realtime priority (ulimit -r 0): the control loop runs at normal priority and the robot
  may stop with communication_constraints_violation. Log out and in again (realtime group), check: ulimit -r -> 99"
uptime | grep -q "load average: [0-3][.,]" || echo "WARNING: machine load $(uptime | sed 's/.*load average: //') (Isaac / MPC runs from other sessions?)"

LAUNCH_PID=
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "--- deactivating the controllers, stopping the launch"
    for c in cartesian_impedance_controller move_to_start_example_controller; do
      timeout 10 ros2 control set_controller_state $c inactive > /dev/null 2>&1
    done
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null   # the launch's own process group (setsid)
    for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  fi
  grep -h "aborted by reflex" "$LOG/launch.log" 2>/dev/null | head -2 | sed 's/^/REFLEX: /'
  n=$(grep -c "no MPC command" "$LOG/launch.log" 2>/dev/null)
  [ "${n:-0}" -gt 0 ] && echo "controller: $n x 'no MPC command' (> 50 ms gap: held the pose; 1 at the end is normal)"
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

echo "--- error recovery (clears a reflex from an earlier run; harmless otherwise)"
timeout 20 ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery "{}" > "$LOG/recovery.txt" 2>&1 \
  || echo "error recovery did not answer (see $LOG/recovery.txt)"

echo "--- 2. move to home (joint space)"
cat > "$LOG/mts_home.yaml" <<EOF
move_to_start_example_controller:
  ros__parameters:
    robot_type: fr3
    k_gains: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
    d_gains: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0]
    start_joint_configuration: [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
EOF
ros2 run controller_manager spawner move_to_start_example_controller -c /controller_manager \
  -t franka_example_controllers/MoveToStartExampleController --param-file "$LOG/mts_home.yaml" \
  >> "$LOG/spawner.log" 2>&1 || { echo "move_to_start spawner failed:"; tail -5 "$LOG/spawner.log"; exit 1; }
for i in $(seq 120); do
  ros2 param get /move_to_start_example_controller process_finished 2>/dev/null | grep -q True && break
  sleep 0.5
done
ros2 param get /move_to_start_example_controller process_finished 2>/dev/null | grep -q True \
  || { echo "move to home did not finish in 60 s"; exit 1; }
ros2 control set_controller_state move_to_start_example_controller inactive > /dev/null
ros2 control unload_controller move_to_start_example_controller > /dev/null

echo "--- 3. activate cartesian_impedance_controller: the arm holds its pose"
ros2 run controller_manager spawner cartesian_impedance_controller -c /controller_manager --param-file "$PARAMS" \
  >> "$LOG/spawner.log" 2>&1 || { echo "spawner failed:"; tail -5 "$LOG/spawner.log"; exit 1; }

echo "--- 4. MPC tests ($TESTS) -> $D"
python3 -u "$HERE/mpc_free_space_real.py" --tests "$TESTS" --out "$D" "$@" 2>&1 | tee "$LOG/mpc_free_space.out"
