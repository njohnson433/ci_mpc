#!/bin/bash
# Generate the FR3 (+ Franka Hand) URDF from franka_description with absolute mesh paths.
set -e
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
ROBOT=${ROBOT:-fr3}          # fr3 | fr3v2 | fr3v2_1 — must match the real arm
HAND=${HAND:-true}
OUT=${OUT:-$HOME/ci_mpc_assets}
FD=$(ros2 pkg prefix franka_description)/share/franka_description
mkdir -p "$OUT"
ros2 run xacro xacro "$FD/robots/$ROBOT/$ROBOT.urdf.xacro" hand:=$HAND ros2_control:=false > "$OUT/$ROBOT.urdf"
sed -i "s|package://franka_description|$FD|g" "$OUT/$ROBOT.urdf"
echo "written $OUT/$ROBOT.urdf  links: $(grep -c '<link' "$OUT/$ROBOT.urdf")  package:// left: $(grep -c 'package://' "$OUT/$ROBOT.urdf" || true)"
