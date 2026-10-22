#!/bin/bash
# End-to-end test: headless Isaac Sim + in-process ros2_control + cartesian_impedance_controller.
# Checks that the arm holds its pose after activation and reaches a reference pose.
# Usage: run_impedance_test.sh [extra args for check_impedance_tracking.py]
# Uses its own ROS_DOMAIN_ID (default 42) so it doesn't talk to other ROS nodes.
# CONTROLLER_CONFIG=/path/to.yaml overrides the controller YAML (e.g. for gain sweeps).
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
SYNC=$(mktemp -d)
LOG=${LOG:-$SYNC/isaac.log}
echo "sync dir $SYNC, Isaac log $LOG"

# Isaac side in a clean environment (no /opt/ros sourced)
env -i HOME="$HOME" USER="$USER" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" OMNI_KIT_ACCEPT_EULA=YES \
    "$HERE/../isaac_ros.sh" --python "$HERE/headless_impedance_sim.py" --sync-dir "$SYNC" ${CONTROLLER_CONFIG:+--config "$CONTROLLER_CONFIG"} > "$LOG" 2>&1 &
ISAAC_PID=$!

# ROS side
(
  source /opt/ros/humble/setup.bash
  source ~/franka_ros2_ws/install/setup.bash
  exec python3 "$HERE/../check_impedance_tracking.py" --sync-dir "$SYNC" "$@"
) &
CHECK_PID=$!
# If Isaac exits early (crash, missing prim), stop the checker instead of letting it wait forever
( while kill -0 $ISAAC_PID 2>/dev/null; do sleep 1; done
  kill -0 $CHECK_PID 2>/dev/null && echo "Isaac exited before the checker finished" && kill $CHECK_PID ) &
WATCH_PID=$!
wait $CHECK_PID
RC=$?
kill $WATCH_PID 2>/dev/null
touch "$SYNC/done"
wait $ISAAC_PID
grep -E "^\[sim\]|cartesian_impedance|ros2.control\]|Error|error" "$LOG" | grep -v -i "warn" | tail -20
exit $RC
