"""Stage-1 MPC node.

    ros2 launch ci_mpc_planner mpc.launch.py                       # sim config, hold
    ros2 launch ci_mpc_planner mpc.launch.py config:=real task:=hold
    ros2 launch ci_mpc_planner mpc.launch.py task:=reach goal:=[0.4,0.1,0.4]
Goals at runtime: /ci_mpc_planner/goal (geometry_msgs/Point), /ci_mpc_planner/goal_pose (geometry_msgs/Pose).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context):
    share = get_package_share_directory("ci_mpc_planner")
    config = LaunchConfiguration("config").perform(context)
    params = [os.path.join(share, "config", f"mpc_{config}.yaml"), {"task": LaunchConfiguration("task").perform(context)}]
    goal = LaunchConfiguration("goal").perform(context)
    if goal:
        params.append({"goal": [float(g) for g in goal.strip("[]").split(",")]})
    return [Node(package="ci_mpc_planner", executable="mpc_node", name="ci_mpc_planner", output="screen",
                 parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="sim", description="sim | real (config/mpc_<config>.yaml)"),
        DeclareLaunchArgument("task", default_value="hold", description="hold | reach | circle | figure8"),
        DeclareLaunchArgument("goal", default_value="", description="reach goal [x,y,z] in the base frame"),
        OpaqueFunction(function=make_node),
    ])
