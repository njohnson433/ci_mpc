#!/bin/bash
# Real FR3: friction compensation with this robot's identified joint values (ci_mpc_sysid/results/real_fr3_20261028.yaml,
# 3 poses), ramped by scale (ci_mpc/docs/REAL_ROBOT.md 4). Wraps c3_breakaway_session.sh with a generated controller
# parameter file: real_robot_controllers.yaml + friction_compensation <mode>, friction_scale <scale>, the identified
# Coulomb / viscous values and the joint torque offsets (joint 2: -0.66 Nm gravity shortfall).
#
# Usage: friction_session.sh <velocity|coulomb|hybrid> <scale> hold|probe|home [args for c3_breakaway_real.py]
#        friction_session.sh <velocity|coulomb> <scale> mpc test|all|steps|rotate|circle [args]  (mpc_free_space_session.sh)
#   velocity  compensation only while a joint moves (tanh(dq / 0.02 rad/s)): nothing at rest; the safe first step
#   coulomb   compensation in the direction of the impedance torque (tanh(tau / 0.3 Nm)): also at rest, so it lowers
#             the breakaway force (what C3 sees); can drift if over-compensated; wobbled slightly during moves at 0.5
#   hybrid    per joint: velocity-direction at <scale> while moving, fading into torque-direction at REST (env,
#             default 0.5) at rest (real FR3 2026-10-28: velocity 0.75 best in motion, coulomb 0.5 halves the breakaway)
#   hold      hold at the current pose; nudge test: push the arm a little and let go. Comes back and settles = fine.
#             Keeps moving, oscillates or speeds up = over-compensated: Ctrl-C, go back to the last good scale.
#   probe / home   the breakaway measurement (as c3_breakaway_session.sh): the residual stiction with compensation
# Env: OFFSETS=0 to leave out the joint torque offsets.
MODE=$1; SCALE=$2; WHAT=$3; shift 3
case "$MODE" in velocity|coulomb|hybrid) ;; *) sed -n 2,16p "$0"; exit 1 ;; esac
[ -n "$SCALE" ] && [ -n "$WHAT" ] || { sed -n 2,16p "$0"; exit 1; }
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=~/franka_ros2_ws/src/cartesian_impedance_control/config/real_robot_controllers.yaml
P=$(mktemp --suffix=.yaml /tmp/friction_XXXX)
{
  cat "$BASE"
  echo "    friction_compensation: $MODE"
  echo "    friction_scale: $SCALE"
  echo "    friction_scale_rest: ${REST:-0.5}"
  echo "    friction_coulomb: [0.815, 0.701, 0.770, 0.882, 1.159, 0.713, 0.395]"
  echo "    friction_viscous: [0.41, 0.618, 0.393, 0.994, 0.0, 0.279, 0.075]  # j5 fits negative (Stribeck): 0"
  [ "${OFFSETS:-1}" = 1 ] && echo "    joint_torque_offset: [0.089, -0.658, 0.033, 0.232, -0.056, 0.230, -0.025]"
} | sed '/^    friction_compensation: none/d' > "$P"
echo "--- friction: $MODE x $SCALE (rest ${REST:-0.5}, hybrid only; offsets ${OFFSETS:-1}); parameters $P"
if [ "$WHAT" = mpc ]; then
  PARAMS="$P" exec "$HERE/mpc_free_space_session.sh" "$@"
fi
PARAMS="$P" exec "$HERE/c3_breakaway_session.sh" "$WHAT" "$@"
