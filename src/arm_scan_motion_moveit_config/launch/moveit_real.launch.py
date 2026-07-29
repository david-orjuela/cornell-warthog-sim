#!/usr/bin/env python3
"""
MoveIt 2 Launch File for Real Hardware Execution
=================================================
Launches:
1. move_group node (configured for UR5e arm planning group and OMPL/Pilz planners).
2. RViz2 with MoveIt Motion Planning plugin display.

Connects to the REAL UR5e robot hardware & joint controllers launched via:
  ros2 launch arm_scan_motion bringup.launch.py

Excludes:
- No fake hardware or duplicate ros2_control_node.
- No duplicate robot_state_publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)


def generate_launch_description():
    # Declare launch argument to optionally toggle RViz2
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 with MoveIt Motion Planning display plugin'
    )

    # Build MoveIt 2 configuration context
    moveit_config = MoveItConfigsBuilder(
        "w200-0100",
        package_name="arm_scan_motion_moveit_config"
    ).to_moveit_configs()

    # 1. move_group node
    move_group_launch = generate_move_group_launch(moveit_config)

    # 2. RViz2 node with MoveIt config
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription([
        declare_launch_rviz,
        move_group_launch,
        rviz_launch,
    ])
