#!/usr/bin/env python3
"""
Launch file: starts the tree reconstructor node and RViz2 with a pre-built config.

Camera TF:
  Publishes a static transform  arm_0_tool0 -> camera_jetson_link.
  The camera driver already publishes:
    camera_jetson_link -> camera_jetson_color_frame -> camera_jetson_color_optical_frame
  So connecting arm_0_tool0 -> camera_jetson_link is all that's needed.

  Initial guess is identity (camera at tool0 origin, axes aligned with tool0).
  Tune visually in RViz by changing the arguments below and re-launching:
    cam_x / cam_y / cam_z        – translation in metres (tool0 frame)
    cam_roll / cam_pitch / cam_yaw – rotation in DEGREES (extrinsic XYZ)

  Typical RealSense on a flange: camera is ~5 cm in front of the flange,
  slightly offset, looking along the tool's +Z or +X axis.

Robot model:
  If robot model does not appear in RViz, run:
    ros2 topic list | grep robot
    ros2 node list | grep state_pub
  and update ROBOT_DESCRIPTION_TOPIC below.

Usage:
    ros2 launch arm_scan_motion reconstruct.launch.py
"""

import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Topic remappings: robot uses /manipulators/tf[_static] not /tf[_static]
# ---------------------------------------------------------------------------
TF_REMAPS = [
    ('/tf',        '/manipulators/tf'),
    ('/tf_static', '/manipulators/tf_static'),
]




def generate_launch_description():
    pkg_share = get_package_share_directory('arm_scan_motion')
    rviz_config = os.path.join(pkg_share, 'rviz', 'bringup.rviz')



    # ------------------------------------------------------------------
    # Reconstructor node
    # ------------------------------------------------------------------
    reconstructor_node = Node(
        package='arm_scan_motion',
        executable='reconstructor',
        name='tree_reconstructor',
        output='screen',
        emulate_tty=True,
        remappings=TF_REMAPS,
    )

    # ------------------------------------------------------------------
    # RViz2
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        remappings=TF_REMAPS,
    )

    return LaunchDescription([
        reconstructor_node,
        rviz_node,
    ])
