#!/bin/bash
# Start Isaac Sim (pip/uv install in ~/IsaacLab) with the bundled ROS 2 Humble bridge.
# Run in a clean terminal: no ROS sourced, no venv active.
# Extra args are passed to isaacsim (e.g. a .kit file or --no-window).
#
# Our ros2_control plugins (cartesian_impedance_control) are made visible to the in-process
# Controller Manager (isaacsim.ros2.control) without sourcing /opt/ros: its Python 3.10 paths
# would break Isaac's Python 3.12.
cd ~/IsaacLab
ISAAC_ROS_LIB=$(find .venv/lib/python3.12/site-packages -type d -path "*isaacsim.ros2.core/humble/lib" | head -1)
if [ -z "$ISAAC_ROS_LIB" ]; then echo "bundled humble libs not found"; exit 1; fi
export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

WS_INSTALL=~/franka_ros2_ws/install
WS_PREFIXES=$(find "$WS_INSTALL" -mindepth 1 -maxdepth 1 -type d | sort | tr '\n' ':')
WS_LIBS=$(find "$WS_INSTALL" -mindepth 2 -maxdepth 2 -type d -name lib | sort | tr '\n' ':')

# pluginlib finds plugin descriptions through the ament index.
export AMENT_PREFIX_PATH="${WS_PREFIXES}/opt/ros/humble"
# Isaac's bundled Humble libs FIRST (rclcpp, controller_manager, hardware_interface, ...), then
# the workspace, then system Humble for what Isaac doesn't bundle (pinocchio, coal, libfranka deps).
export LD_LIBRARY_PATH="$PWD/$ISAAC_ROS_LIB:${WS_LIBS}/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# `isaac_ros.sh --python script.py [args]` runs a standalone (SimulationApp) script in the same environment.
if [ "$1" = "--python" ]; then
  shift
  exec uv run --extra isaacsim python "$@"
fi
uv run --extra isaacsim isaacsim --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge "$@"
