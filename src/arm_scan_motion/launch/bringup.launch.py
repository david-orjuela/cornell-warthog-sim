#!/usr/bin/env python3
"""
Bringup Launch File for Warthog + UR5e + Pruner Tool & Camera Mount
=====================================================================
Brings up:
1. Unified Warthog + UR5e + Pruner + Camera Mount robot description (warthog_ur5e_state_publisher loading warthog_ur5e_fixed_wheels.urdf).
2. UR5e control driver connected to the physical robot at 192.168.56.101 via ur_robot_driver.
3. Pre-configured RViz2 display (toggleable via launch_rviz:=true/false).

Excludes:
- No separate pruner_state_publisher or pruner_static_tf nodes (geometry integrated directly into URDF).
- No RealSense camera driver (running independently as systemd service on Jetson).
- No tree_reconstructor node (launched separately when reconstructing).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_scan_motion')

    # Launch Arguments
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 visualization automatically with saved display config'
    )

    # ------------------------------------------------------------------
    # 1. Combined Warthog + UR5e Robot Description (Fixed Wheels, TF / Visualization only)
    # Publishes to global /tf and /tf_static
    # ------------------------------------------------------------------
    warthog_urdf_path = os.path.join(pkg_share, 'urdf', 'warthog_ur5e_fixed_wheels.urdf')
    warthog_robot_description = ParameterValue(
        Command(['xacro ', warthog_urdf_path]), value_type=str
    )

    warthog_ur5e_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='warthog_ur5e_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': warthog_robot_description,
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------------------------
    # 2. UR5e Control Driver (Real Hardware connection at 192.168.56.101)
    # Remap /robot_description, /tf, and /tf_static for driver RSP node to prevent TF tree conflicts
    # ------------------------------------------------------------------
    ur5e_control_driver = GroupAction([
        SetRemap(src='/robot_description', dst='/arm_only_robot_description'),
        SetRemap(src='/tf', dst='/arm_only_tf'),
        SetRemap(src='/tf_static', dst='/arm_only_tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch', 'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': '192.168.56.101',
                'tf_prefix': 'arm_0_',
                'kinematics_params_file': os.path.join(pkg_share, 'config', 'ur5e_calibration.yaml'),
                'launch_rviz': 'false',
                'launch_dashboard_client': 'false',
                'initial_joint_controller': 'joint_trajectory_controller',
            }.items()
        )
    ])

    # ------------------------------------------------------------------
    # 3. RViz2 Node with Pre-Configured bringup.rviz Display Settings
    # ------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arm_scan_motion'), 'rviz', 'bringup.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        declare_launch_rviz,
        warthog_ur5e_description,
        ur5e_control_driver,
        rviz_node,
    ])
