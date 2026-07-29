cmake_minimum_required(VERSION 3.22)
project(arm_scan_motion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executable nodes
install(PROGRAMS
  arm_scan_motion/reconstructor_node.py
  arm_scan_motion/debug_reconstruction_node.py
  arm_scan_motion/scan_motion.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS arm_scan_motion/debug_reconstruction_node.py RENAME debug_reconstruction DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS arm_scan_motion/reconstructor_node.py RENAME reconstructor DESTINATION lib/${PROJECT_NAME})

# 1. Scanning Trajectory Generator & Recorder Executable (Cartesian Planning)
add_executable(scanning_trajectory_test src/scanning_trajectory_node.cpp)
target_include_directories(scanning_trajectory_test PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(scanning_trajectory_test rclcpp moveit_ros_planning_interface geometry_msgs)

# 2. Main Scanning Trajectory Action Replay Executable (Fast Action Replay)
add_executable(scanning_trajectory src/scanning_trajectory.cpp)
ament_target_dependencies(scanning_trajectory rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs)

add_executable(scanning_trajectory_node
  src/scanning_trajectory_node.cpp
)

ament_target_dependencies(scanning_trajectory_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

find_package(moveit_core REQUIRED)
install(TARGETS
  scanning_trajectory_test
  scanning_trajectory_node
  scanning_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

# Install data directories (launch, rviz, urdf, meshes, config)
install(DIRECTORY launch rviz urdf meshes config DESTINATION share/${PROJECT_NAME})


ament_package()
