#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            reconstructor,
        ]
    )