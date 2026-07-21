#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("reconstruct"), "config", "params.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    execute_motion = LaunchConfiguration("execute_motion")
    use_sim_time = LaunchConfiguration("use_sim_time")

    reconstructor = Node(
        package="reconstruct",
        executable="pointcloud_processing.py",
        name="tree_point_cloud_reconstructor",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    planner = Node(
        package="reconstruct",
        executable="motion_planner.py",
        name="trajectory_planner",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "execute_motion": ParameterValue(execute_motion, value_type=bool),
            },
        ],
        condition=IfCondition(execute_motion),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            reconstructor,
            # Gives subscriptions, TF, and RGB-D synchronization time to initialize.
            TimerAction(period=3.0, actions=[planner]),
        ]
    )
