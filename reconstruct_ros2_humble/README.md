# reconstruct — ROS 2 Humble / Gazebo Fortress

This package replaces the ROS 1 `rospy`, `actionlib`, `tf`, `ros_numpy`, catkin,
and global parameter-server code with ROS 2 Humble equivalents.

## What it does

1. `motion_planner.py` uses cuRobo to generate an **absolute**, snake-pattern
   raster around the UR5e's initial end-effector pose.
2. It sends each trajectory to a ROS 2
   `control_msgs/action/FollowJointTrajectory` action server.
3. After motion settles, it requests one synchronized Jetson RGB-D capture and
   waits for `/capture_done` before moving again.
4. `pointcloud_processing.py` uses the aligned Jetson depth/color streams,
   looks up `target_frame <- camera optical frame` in TF2 at the image stamp,
   performs optional local-map multi-scale ICP, publishes the global cloud, and
   continuously writes:

   `~/tree_scans/indoor_model_tree/global_point_cloud.ply`

## Required preflight checks

Your supplied topic list contains the Jetson RGB-D inputs, `/tf`, `/tf_static`,
and `/platform/joint_states`, but it does **not prove that the UR5e trajectory
action is running**. Topics do not list actions or services.

Run:

```bash
ros2 topic echo /platform/joint_states --once
ros2 action list -t | grep follow_joint_trajectory
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_link <camera_optical_frame>
ros2 topic hz /sensors/camera_jetson/color/image_raw
ros2 topic hz /sensors/camera_jetson/aligned_depth_to_color/image_raw
ros2 topic echo /sensors/camera_jetson/aligned_depth_to_color/camera_info --once
```

The joint-state message must contain all six UR5e joints. For Gazebo Fortress
with `gz_ros2_control`, the normal action is:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

For the real Universal Robots ROS 2 driver, it is commonly:

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

The selected controller must already be loaded and `active`; this package does
not guess which other controller is safe to deactivate on your Warthog.

## Gazebo clock

The provided topic list does not show `/clock`. When `use_sim_time:=true`, a
Gazebo Fortress simulation needs a ROS clock. Ensure your Gazebo launch or
`ros_gz_bridge` exposes `/clock`; otherwise launch this package with
`use_sim_time:=false` only when all publishers are using wall time.

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/reconstruct_ros2_humble ./reconstruct
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select reconstruct
source install/setup.bash
```

Install non-ROS Python dependencies in the same Python environment used by ROS:

```bash
python3 -m pip install open3d
# Install cuRobo using the NVIDIA instructions appropriate to your CUDA/Jetson setup.
```

## Safe first run: reconstruction only

Keep the arm still and trigger one frame manually:

```bash
ros2 launch reconstruct scan_tree.launch.py execute_motion:=false use_sim_time:=true
```

In another terminal:

```bash
ros2 topic pub --once /capture_alert std_msgs/msg/Bool "{data: true}"
```

Then inspect:

```bash
ls -lh ~/tree_scans/indoor_model_tree/global_point_cloud.ply
rviz2
```

In RViz, set **Fixed Frame** to `base_link`, add a **PointCloud2** display, and
select `/tree_scan/global_cloud`.

## Motion run

Before enabling motion, place the arm at a collision-checked starting pose with
the Jetson camera facing the model tree at the desired standoff. The scan keeps
that tool orientation and translates over a raster around the current pose. The
default raster axes are tool-frame X/Y; change `scan_axis_u_ee` and
`scan_axis_v_ee` if the camera mount is rotated relative to the tool.

Only after the preflight checks and a reconstruction-only capture work:

```bash
ros2 launch reconstruct scan_tree.launch.py execute_motion:=true use_sim_time:=true
```

Tune scan size, axes, depth, voxel size, and ICP in `config/params.yaml`. Use a
new `scan_id` for each experiment if you want to retain every run.

## Remaining physical-model requirements

Code cannot infer these from a topic list:

- The exact UR5e trajectory action name and whether its controller is active.
- Whether `/platform/joint_states` includes the six arm joints.
- The `base_link -> ... -> camera optical frame` TF chain.
- The calibrated camera-to-tool transform.
- The camera housing geometry in the cuRobo collision model.
- The actual room/tree/table collision geometry. The default
  `collision_table.yml` is only a floor slab.

For real hardware, add the camera mount/housing to the URDF and cuRobo collision
spheres, and replace the world YAML with measured obstacles before increasing
scan dimensions.
