# 2026-07-21 — ROS 2 Humble Migration and Indoor Scan Preparation

## Day Objective

The primary goal for the day became adapting Dawood's ROS 1 tree-reconstruction and UR5e motion pipeline to the software stack currently available on the Warthog:

```text
ROS 2 Humble
+ Gazebo Fortress
+ UR5e end-effector Jetson/RealSense camera
+ cuRobo motion planning
```

The immediate practical milestone remained deliberately limited:

1. Receive one synchronized RGB-D frame from the UR5e-mounted camera.
2. obtain the camera pose through TF.
3. create a colored Open3D point cloud.
4. transform it into a fixed robot frame.
5. save and inspect a PLY file.
6. only after that works, perform a small supervised move–settle–capture scan indoors.

The code was **not** considered ready for an unsupervised grove scan by the end of the day.

---

## Repository Review and Pipeline Separation

Finished mapping the remaining files in Dawood’s repository:

- `bisenetv1_eca_predict.py`
- `skeletonization.py`
- `tree_component.py`
- `pc_utils.py`
- `test.py`
- `skeletonization_params.yaml`
- `tfs.rviz`

The repository contains two largely separable pipelines.

### RGB-D reconstruction pipeline

```text
UR5e motion
→ synchronized RGB-D capture
→ TF-based placement
→ multiscale ICP
→ fused point cloud
→ PLY output
```

### Semantic and structural-analysis pipeline

```text
trunk/branch segmentation
→ DBSCAN clustering
→ spline skeletonization
→ branch length and diameter estimation
```

The first pipeline is directly relevant to scanning a model tree and producing geometry that can later be inspected or adapted for Gazebo. The semantic and skeletonization pipeline was designed primarily for dormant-tree structural analysis and is not required for the first leafy-tree reconstruction test.

For the initial adaptation, the complete RGB-D geometry should be retained, including foliage, rather than filtering the cloud down to only semantically classified woody components.

---

## Initial Geometry-First Adaptation Scope

### Preserved concepts

- synchronized RGB, aligned depth, and camera intrinsics
- TF-based placement into a common frame
- Open3D RGB-D point-cloud generation
- coarse-to-fine point-to-plane ICP
- voxel downsampling
- radius-outlier removal
- ROS point-cloud publishing
- PLY output
- per-frame RGB, depth, transform, and registration metadata

### Removed or bypassed for the first test

- YOLO/BiSeNet trunk and branch classification
- semantic-color background removal
- class-specific point-cloud downsampling
- skeletonization
- branch diameter and length estimation
- paper-specific visualization and reporting code

This reduced the immediate problem to validating camera data, TF, registration, fusion, and output before reintroducing any semantic processing.

---

## ROS 1 Architecture Identified as a Fundamental Blocker

Although the first edits focused on adapting the reconstruction logic, a larger issue became clear: the repository’s two main Python scripts and package configuration were built around ROS 1, while the Warthog system uses ROS 2 Humble.

The original code depended on:

- `rospy`
- `actionlib.SimpleActionClient`
- ROS 1 controller-manager service calls
- ROS 1 `tf.TransformListener`
- ROS 1 duration and timestamp APIs
- `ros_numpy`
- catkin
- ROS 1 parameter-server layout
- old Universal Robots controller names
- hardcoded ROS 1 camera topics

Therefore, incremental fixes alone would not make the pipeline runnable. The motion planner, point-cloud node, utility layer, package configuration, parameters, and launch process all required a ROS 2 migration.

---

## Available ROS 2 Interfaces

Reviewed the ROS 2 topic list on the Warthog. The camera attached to the UR5e is the `camera_jetson` device.

The three primary reconstruction inputs are:

```text
/sensors/camera_jetson/color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/camera_info
```

Relevant supporting interfaces include:

```text
/tf
/tf_static
/platform/joint_states
```

The topic list alone did **not** establish:

- whether a UR5e `FollowJointTrajectory` action server is active
- which trajectory-controller namespace is correct
- whether `/platform/joint_states` actually contains all six UR5e joints
- whether a complete TF chain exists from the selected fixed frame to the camera optical frame
- whether Gazebo is publishing or bridging `/clock`

These remained mandatory preflight checks.

---

## Full ROS 1 → ROS 2 Humble Migration

Created a first-pass ROS 2 package for the reconstruction and motion-planning pipeline.

### Package and build-system conversion

Replaced the ROS 1 catkin package with a ROS 2 package using:

- `ament_cmake`
- `ament_cmake_python`
- ROS 2 `package.xml` format 3
- installed Python executables
- declared ROS 2 runtime dependencies
- a ROS 2 launch file
- node-specific ROS 2 YAML parameters

The package now includes:

```text
scripts/motion_planner.py
scripts/pointcloud_processing.py
reconstruct/pc_utils.py
config/params.yaml
launch/scan_tree.launch.py
CMakeLists.txt
package.xml
README.md
```

The Python files were syntax-checked successfully. This only verifies parsing and basic packaging; it does not prove compatibility with the live controller, TF tree, cuRobo installation, camera driver, or Gazebo environment.

---

## ROS 2 Motion Planner

Rewrote the main motion script around `rclpy` and the ROS 2 `FollowJointTrajectory` action.

### Major changes

- Replaced `rospy` with an `rclpy.node.Node`.
- Replaced ROS 1 `actionlib` with `rclpy.action.ActionClient`.
- Made the trajectory action namespace configurable.
- Removed automatic assumptions about which controllers should be stopped or started.
- Added explicit checks for:
  - receipt of a valid UR5e joint state
  - availability of the trajectory action server
  - connection to the reconstruction node
- Added a default motion safety interlock:

```yaml
execute_motion: false
```

- Added a CUDA availability check before initializing cuRobo.
- Preserved cuRobo’s joint ordering and mapped controller or prefixed joint names safely.
- Converted cuRobo tensors to CPU Python lists before constructing ROS trajectory messages.
- Treated cuRobo quaternions using its documented `w, x, y, z` convention.
- Replaced the long sequence of accumulated relative motions with a generated absolute snake-pattern raster around the initial end-effector pose.
- Kept the initial end-effector orientation throughout the raster.
- Replaced the unsafe hardcoded home pose with an optional return to the actual scan-start pose.
- Added configurable scan dimensions, row and column counts, scan-plane axes, time scaling, settling time, and execution timeouts.

### Capture synchronization

Implemented a move–settle–capture handshake:

```text
plan trajectory
→ execute trajectory
→ wait for controller result
→ allow the arm/camera to settle
→ publish one capture request
→ wait for reconstruction acknowledgement
→ move to the next pose
```

The motion planner publishes:

```text
/capture_alert
```

The reconstruction node acknowledges each completed capture on:

```text
/capture_done
```

The planner does not proceed to the next raster pose until that acknowledgement arrives. This prevents the previous behavior in which the camera could process frames continuously while the arm was moving.

---

## ROS 2 Point-Cloud Reconstruction Node

Rewrote the reconstruction script around the UR5e Jetson camera topics and ROS 2 APIs.

### Synchronization and camera handling

- Replaced ROS 1 subscribers with ROS 2 `message_filters.Subscriber` objects.
- Used an `ApproximateTimeSynchronizer` for:
  - color image
  - aligned depth image
  - aligned-depth camera intrinsics
- Used sensor-data QoS for camera streams.
- Increased the synchronization tolerance from the overly restrictive original setting to a configurable ROS 2 value.
- Read the camera frame from the incoming message header instead of hardcoding `rgb_camera_link`.
- Added support for common color encodings such as:
  - `rgb8`
  - `bgr8`
  - `rgba8`
  - `bgra8`
  - grayscale
- Added automatic depth-unit handling:
  - `16UC1` interpreted as millimeters with scale `1000.0`
  - `32FC1` interpreted as meters with scale `1.0`
- Added configurable minimum and maximum depth thresholds.
- Corrected the BGR/RGB mismatch before passing color data to Open3D.

### TF2 conversion

- Replaced ROS 1 `tf` with `tf2_ros.Buffer` and `TransformListener`.
- Performed the transform lookup at the sensor timestamp.
- Constructed the homogeneous transform directly from the ROS 2 translation and quaternion.
- Made the fixed reconstruction frame configurable, initially:

```yaml
target_frame: base_link
```

For every frame, the initial point placement is:

```text
point in camera optical frame
→ TF transform at capture time
→ point in target frame
```

### Point-cloud generation and registration

- Constructed a colored Open3D point cloud from aligned RGB-D data.
- Applied the TF transform before ICP.
- Retained coarse-to-fine point-to-plane ICP as a bounded correction rather than using it to compensate for a missing camera calibration.
- Registered against a spatially cropped portion of the global map when enough local points are available.
- Retained configurable multiscale registration thresholds for:
  - voxel size
  - maximum correspondence distance
  - iteration count
  - relative RMSE and fitness convergence
  - acceptance RMSE
  - acceptance fitness
- Added sanity limits for the maximum translation and rotation that ICP is permitted to apply.
- Rejected frames whose registration quality or correction magnitude was implausible.
- Applied configurable local voxel downsampling and radius-outlier removal.
- Downsampled the accumulated global map after accepted frames.

### Capture state and output

- Removed continuous automatic processing.
- Each capture request now queues exactly one synchronized RGB-D capture.
- Kept a capture pending if TF is temporarily unavailable rather than silently consuming it.
- Published the TF-positioned latest frame and the fused global cloud separately:

```text
/tree_scan/latest_tf_cloud
/tree_scan/global_cloud
```

- Wrote the accumulated PLY after every processed capture.
- Saved per-frame RGB, depth, intrinsics, TF, ICP correction, acceptance state, fitness, and RMSE.
- Added a final JSON summary with accepted-frame counts, average ICP metrics, processing time, point count, and PLY path.

The default output location is:

```text
~/tree_scans/indoor_model_tree/global_point_cloud.ply
```

---

## ROS 2 PointCloud2 Utility

Replaced the original ROS 1 `ros_numpy` conversion utility.

The new helper:

- directly constructs a ROS 2 `sensor_msgs/msg/PointCloud2`
- writes `x`, `y`, and `z` as `float32`
- packs RGB into the conventional PCL-compatible field
- preserves the supplied frame ID and sensor timestamp
- removes dependencies on `rospy.Time` and `ros_numpy`

This resolves another ROS 1-only dependency that would otherwise prevent the reconstructed cloud from being published under ROS 2.

---

## Runtime Bugs and Design Problems Addressed

The migration also corrected or replaced several problems identified in the intermediate ROS 1 adaptation:

- The original camera topics did not match the Warthog’s Jetson camera topics.
- The trajectory action interface was ROS 1-specific.
- The original code depended on ROS 1 controller-manager service definitions and old controller names.
- The motion sequence accumulated relative movements and could drift away from the intended scan grid.
- The home pose was hardcoded and unverified for the current physical mounting arrangement.
- Joint-state positions could be interpreted in the wrong order.
- cuRobo trajectory tensors needed explicit CPU/list conversion.
- Quaternion ordering and normalization required explicit handling.
- The reconstruction node automatically re-enabled capture and processed frames continuously.
- First-frame registration variables could be referenced before assignment.
- `save_frame_data()` was called with arguments its signature did not accept.
- A rejected ICP frame could cause metadata-saving failures.
- The original code assumed BGRA color and 16-bit depth regardless of the incoming encoding.
- Open3D received BGR values while treating them as RGB.
- The original TF frames were hardcoded.
- The image counter and file-writing behavior risked overwriting or invalid output.
- The original output directory was hardcoded under `/datasets`.
- The old idle-shutdown thread was suited to rosbag playback but unsafe for a live camera.
- The global ICP target could grow without bound and encourage incorrect correspondences.
- The ROS 1 package did not install the Python scripts or declare the required runtime dependencies.

---

## Indoor Test Procedure Prepared

### Preflight inspection

Before moving the arm, verify the actual ROS 2 interfaces:

```bash
ros2 topic echo /platform/joint_states --once
ros2 action list -t | grep follow_joint_trajectory
ros2 control list_controllers
ros2 topic echo /sensors/camera_jetson/aligned_depth_to_color/camera_info --once
ros2 topic hz /sensors/camera_jetson/color/image_raw
ros2 topic hz /sensors/camera_jetson/aligned_depth_to_color/image_raw
```

Use the camera-info message’s `header.frame_id` to test the TF chain:

```bash
ros2 run tf2_ros tf2_echo base_link <camera_optical_frame>
```

### Reconstruction-only test

Launch the reconstruction node without arm motion:

```bash
ros2 launch reconstruct scan_tree.launch.py \
  execute_motion:=false \
  use_sim_time:=true
```

Trigger one capture manually:

```bash
ros2 topic pub --once \
  /capture_alert \
  std_msgs/msg/Bool \
  "{data: true}"
```

Then inspect:

```text
~/tree_scans/indoor_model_tree/global_point_cloud.ply
```

In RViz:

- set the fixed frame to `base_link`
- add `/tree_scan/latest_tf_cloud`
- add `/tree_scan/global_cloud`

The latest-frame topic helps distinguish a TF or calibration problem from an ICP/fusion problem.

### Initial motion test

Only after a stationary PLY capture works:

1. place the UR5e at a safe manually verified start pose facing the indoor model tree
2. reduce the raster to a very small area
3. confirm that the configured scan axes move across the camera image plane rather than toward the tree
4. run with supervision and low speed
5. verify every capture before increasing scan dimensions

---

## Remaining External Blockers

The code cannot determine these from the topic list alone.

### Trajectory controller

Need to identify the active ROS 2 action name. Likely candidates include:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

for Gazebo/`gz_ros2_control`, or:

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

for the physical Universal Robots ROS 2 driver.

The controller must already be loaded and active.

### Joint states

Need to confirm whether `/platform/joint_states` includes all six UR5e joints or only Warthog platform joints. If not, the correct arm joint-state topic must be supplied.

### Simulation time

The provided topic list did not show `/clock`. A Gazebo Fortress run using:

```yaml
use_sim_time: true
```

requires a working ROS clock bridge. Camera and TF timestamps must use the same time source.

### TF and hand–eye calibration

Need a complete and accurate chain from the fixed robot frame through the UR5e and tool mount to the camera optical frame. ICP should only correct small residual alignment errors; it cannot replace a correct camera extrinsic calibration.

### cuRobo collision models

The default `collision_table.yml` is only a generic example environment. Before physical motion, the collision model should include:

- the actual Warthog-mounted UR5e configuration
- the camera housing and mount
- the floor at the correct height
- nearby walls, tables, and equipment
- a conservative exclusion volume around the model tree

### Physical scan logistics

Still need confirmation from Divyanth or Dawood regarding:

- the correct controller and startup sequence
- current robot and camera calibration files
- whether the original scan path was ever used on this exact mounting configuration
- supervision requirements
- indoor success criteria
- grove transportation, power, weather, permissions, and safety constraints

---

## Version-Control Summary

A suitable commit scope for the day’s work was:

```text
refactor: migrate tree scanning pipeline to ROS2 Humble/Gazebo Fortress
```

The commit encompasses the ROS 2 motion-planning and reconstruction nodes, Jetson camera integration, TF2 conversion, capture synchronization, package migration, PLY output, and runtime/safety fixes.

---

## End-of-Day Status

By the end of Tuesday, July 21:

- The complete repository structure and both major pipelines were understood.
- The reconstruction problem was narrowed to a geometry-first leafy-tree scan.
- The original ROS 1 architecture was identified as incompatible with the Warthog’s ROS 2 Humble stack.
- The two main Python scripts were rewritten as a first-pass ROS 2 implementation.
- The ROS 1 point-cloud conversion utility was replaced.
- The package was converted from catkin to ROS 2 `ament_cmake`/`ament_cmake_python`.
- ROS 2 launch and parameter files were created.
- The UR5e-mounted `camera_jetson` topics were integrated.
- A move–settle–capture acknowledgement sequence was implemented.
- The unsafe relative scan sequence was replaced with a configurable absolute raster.
- The global PLY and per-frame metadata output paths were defined.
- The Python code was syntax-checked.
- The code had **not yet** been validated against the live UR5e controller, the Gazebo Fortress action server, the actual TF tree, or a real RGB-D capture.
- No physical motion or successful complete scan was claimed.

The next immediate milestone is:

```text
verify camera topics and TF
→ capture one stationary PLY
→ inspect cloud orientation and scale
→ verify the UR5e controller and joint-state interfaces
→ run a very small supervised indoor raster
→ tune depth, voxel, outlier, and ICP parameters
→ record a rosbag
→ prepare for a later grove scan
```
