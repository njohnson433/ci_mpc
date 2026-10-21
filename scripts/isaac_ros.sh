#!/bin/bash
# Start Isaac Sim (pip/uv install in ~/IsaacLab) with the bundled ROS 2 Humble bridge.
# Run in a clean terminal: no ROS sourced, no venv active.
cd ~/IsaacLab
ISAAC_ROS_LIB=$(find .venv/lib/python3.12/site-packages -type d -path "*isaacsim.ros2.core/humble/lib" | head -1)
if [ -z "$ISAAC_ROS_LIB" ]; then echo "bundled humble libs not found"; exit 1; fi
export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH="$PWD/$ISAAC_ROS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
uv run --extra isaacsim isaacsim --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge "$@"
