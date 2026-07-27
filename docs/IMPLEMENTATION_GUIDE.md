# Discrete RGB-D Acquisition and Repeatable Filter Evaluation

## What changed

The revised path keeps Dawood's validated motion controller and adds one
pose-specific capture transaction:

1. The trajectory action reports success.
2. The executor waits `capture_settle_time_ms`.
3. It publishes pose ID `N` as `std_msgs/msg/UInt32`.
4. Reconstruction discards synchronized frames timestamped before the request.
5. It collects five new exact-TF RGB-D samples.
6. It computes a zero-aware per-pixel median depth.
7. It saves the raw burst before depth cropping, voxelization, outlier removal,
   or ICP.
8. It returns `+N` when the reconstructed pose is accepted or `-N` when it is
   rejected.
9. The arm advances only after `+N`. A rejection or timeout stops before the
   next motion.

The topics are:

```text
/tree_scan/capture_request  std_msgs/msg/UInt32
/tree_scan/capture_result   std_msgs/msg/Int32
/tree_scan/scan_finished    std_msgs/msg/Bool
```

## File placement

Place the revised files at:

```text
cornell-warthog-sim/src/reconstruct/scripts/pointcloud_processing.py
cornell-warthog-sim/src/reconstruct/scripts/compare_filters.py
cornell-warthog-sim/src/reconstruct/config/params.yaml
cornell-warthog-sim/src/reconstruct/launch/scan_tree.launch.py

dawood_tree_scanning_ur5e/src/arm_scan_motion/src/scanning_trajectory.cpp
dawood_tree_scanning_ur5e/src/arm_scan_motion/src/scanning_trajectory_node.cpp
```

The two package `CMakeLists.txt` files and `package.xml` files still need to be
checked. The C++ targets now require `std_msgs`; the reconstruction package must
install `compare_filters.py`.

## First validation sequence

Build and source both workspaces in the same order used by the lab. Start
Dawood's bringup first:

```bash
ros2 launch arm_scan_motion bringup.launch.py
```

Start reconstruction with the revised parameter file:

```bash
ros2 launch reconstruct scan_tree.launch.py \
  params_file:=$HOME/dev/cornell-warthog-sim/src/reconstruct/config/params.yaml \
  use_sim_time:=false
```

Set a unique `scan_id` in `params.yaml` for each physical acquisition. Reusing
one directory across separate 32-pose runs would mix capture attempts in the
offline comparison.

Before moving the robot, test one stationary capture:

```bash
ros2 topic echo /tree_scan/capture_result
```

In another terminal:

```bash
ros2 topic pub --once \
  /tree_scan/capture_request \
  std_msgs/msg/UInt32 "{data: 1}"
```

Expected result:

```text
data: 1
```

A negative result means the capture was rejected; inspect
`~/tree_scans/<scan_id>/raw/capture_0001/capture.json`.

For the first supervised motion test, use one step:

```bash
ros2 run arm_scan_motion scanning_trajectory --ros-args \
  --params-file $HOME/dev/cornell-warthog-sim/src/reconstruct/config/params.yaml \
  -p max_steps:=1
```

Only proceed to all 32 steps after the result handshake and saved raw data have
been confirmed.

## Raw baseline

Each capture directory contains:

```text
frame_00_color.png
frame_00_depth.npy
frame_00_depth.png       # when the source is 16UC1
...
selected_color.png
median_depth_m.npy
median_depth_mm.png
median_valid_counts.npy
raw_camera_cloud.ply
raw_target_cloud.ply
capture.json
```

`capture.json` contains the camera intrinsics, every sensor timestamp, each
exact `target_frame <- camera_frame` transform, a joint-state snapshot, depth
units, crop bounds, ICP result, and acceptance reason.

This pose-level archive supports filter comparisons. A rosbag remains useful
as the complete audit trail:

```bash
ros2 bag record -o ~/tree_scans/raw_scan_$(date +%Y%m%d_%H%M%S) \
  /sensors/camera_jetson/color/image_raw \
  /sensors/camera_jetson/aligned_depth_to_color/image_raw \
  /sensors/camera_jetson/aligned_depth_to_color/camera_info \
  /tf /tf_static /joint_states \
  /tree_scan/capture_request \
  /tree_scan/capture_result
```

Add color/depth metadata topics if they exist in `ros2 topic list`.

## Offline filter comparison

Run:

```bash
python3 /path/to/compare_filters.py \
  ~/tree_scans/indoor_model_tree
```

It creates a timestamped directory containing:

```text
none/global_point_cloud.ply
current_radius/global_point_cloud.ply
dawood_radius/global_point_cloud.ply
statistical/global_point_cloud.ply
comparison.csv
```

Profiles:

| Profile | Local voxel | Outlier method |
| --- | ---: | --- |
| `none` | Current value, initially 6 mm | None |
| `current_radius` | 6 mm | 6 neighbors within 20 mm |
| `dawood_radius` | 8 mm | 8 neighbors within 14 mm |
| `statistical` | 6 mm | 20 neighbors, standard-deviation ratio 2.0 |

To isolate TF placement from ICP:

```bash
python3 /path/to/compare_filters.py \
  ~/tree_scans/indoor_model_tree \
  --no-icp
```

Point count, ICP fitness, and RMSE are recorded, but visual sharpness,
thin-branch retention, apple completeness, and doubled-surface thickness must
also be inspected.

## Target-relative crop

The optional crop is a fixed axis-aligned box in `target_frame` (`base_link`).
It is disabled by default because the target has not yet been measured.

The easiest initial mode is:

```yaml
target_crop_enabled: true
target_center_mode: camera_forward
target_distance_m: <camera optical center to canopy center>
target_box_size_m: [<base X size>, <base Y size>, <base Z size>]
```

At the first capture, the code projects `[0, 0, target_distance_m]` along the
camera optical `+Z` axis into `base_link`, then locks that center for the whole
scan. Later arm poses do not move the crop.

Measuring camera-to-trunk distance is a reasonable first estimate if the trunk
plane is near the canopy center. It is not itself the crop's far limit. For an
estimated canopy thickness `T` and safety margin `M` on each side:

```text
box depth = T + 2M
near target depth ≈ center distance - box depth / 2
far target depth  ≈ center distance + box depth / 2
```

Start with a deliberately generous width/depth/height box that removes the lab
wall, floor, and robot while retaining all foliage. This is per-scan target
localization, not fitting filter parameters to one laboratory image.

If the first camera pose is not aimed through the target center, use:

```yaml
target_center_mode: fixed_target_frame
target_center_target_frame_m: [x, y, z]
```

## RealSense resolution and tuning

The `640 x 480` values in `warthog_ur5e_fixed_wheels.urdf` configure the Gazebo
sensor only. The physical stream is configured by the RealSense ROS node on the
Jetson. The reconstruction code does not resize either image; it uses the
received dimensions and `CameraInfo`.

Inspect the physical stream:

```bash
ros2 topic echo \
  /sensors/camera_jetson/aligned_depth_to_color/camera_info \
  --once

ros2 topic hz \
  /sensors/camera_jetson/aligned_depth_to_color/image_raw

ros2 node list | grep -i camera
ros2 param get <camera_node> depth_module.depth_profile
ros2 param get <camera_node> rgb_camera.color_profile
```

On the Jetson, `rs-enumerate-devices -c` lists supported profiles. Intel's D435
tuning guidance identifies `848 x 480` as the optimal native depth resolution
and warns that lower native resolution reduces depth precision. Because this
pipeline uses depth aligned to color, the aligned topic normally takes the
color stream's dimensions. Test matching supported color/depth profiles and
confirm throughput before changing the systemd service.

The D400 datasheet makes Min-Z resolution-dependent for the
D430/D435/D435i/D435f/D435if family: approximately 280 mm at `1280 x 720`,
195 mm at `848 x 480`, and 175 mm at `640 x 480` under its base settings.
`min_depth_m: 0.30` is therefore a conservative software validity guard, not a
claim that the 640-by-480 stream physically cannot see anything closer.

Do not enable several RealSense filters at once. First archive an unfiltered
scan, then test one setting at a time under the same range and lighting.

## Clock synchronization

Run these read-only checks on the laptop, Jetson, and any Linux robot computer
that publishes timestamped ROS data:

```bash
hostnamectl --static
date --iso-8601=ns
timedatectl show -p NTPSynchronized -p NTP -p TimeUSec

systemctl is-active chrony || \
systemctl is-active chronyd || \
systemctl is-active systemd-timesyncd

chronyc tracking
chronyc sources -v
chronyc sourcestats -v

grep -RHE \
  '^[[:space:]]*(server|pool|peer|refclock|allow|local|makestep)[[:space:]]' \
  /etc/chrony/chrony.conf /etc/chrony/conf.d 2>/dev/null
```

The selected source in `chronyc sources -v` is marked `^*`. Compare its
hostname/IP across the computers and compare the reported system offsets. Do
not replace the configuration with public NTP until the lab confirms whether
one host is the local master and whether the robot network is isolated.

Also identify which host stamps `/joint_states`:

```bash
ros2 topic info /joint_states -v
```

In the supplied bringup, the UR driver runs on the laptop, so the laptop and
Jetson are the critical clocks unless another computer publishes robot TF or
joint data. Do not try to install chrony on the UR control box merely because it
is a "robot computer."

## Calibration validation

There are three separate calibrations:

1. **UR5e kinematics** — `ur5e_calibration.yaml`, extracted from the physical
   robot.
2. **RealSense intrinsics and RGB-depth calibration** — supplied by the camera
   and reported in `CameraInfo`.
3. **Eye-in-hand extrinsic** — the rigid
   `pruner_link -> camera_jetson_*` transform in the URDF.

The URDF currently says the camera location was CAD-measured and instantiates:

```xml
<xacro:sensor_d435 parent="pruner_link" name="camera_jetson"
                    use_nominal_extrinsics="true">
  <origin xyz="0.069957 0.003698 0.101951" rpy="3.18 -1.57 0"/>
</xacro:sensor_d435>
```

That is a candidate extrinsic, not a validation result.

### Practical validation

1. Fix an AprilTag/ChArUco/checkerboard target rigidly in the workspace.
2. Verify intrinsics and depth against its measured plane at several ranges:
   valid-depth percentage, median range error, and plane-fit RMSE.
3. Acquire at least 8-15 varied arm poses with substantial orientation change.
4. Solve the eye-in-hand transform with a ROS 2 hand-eye tool such as
   `easy_handeye2`, or an equivalent lab procedure.
5. Compare the solved transform with the URDF, then run a TF-only reconstruction
   (`use_icp: false`). A rigid target should coincide across poses without
   systematic double surfaces.
6. Re-enable ICP and inspect the saved correction distribution. A consistent
   correction in one direction indicates an extrinsic problem; it should not be
   hidden by larger ICP limits.

Only after those checks should `max_icp_correction_translation_m` and
`max_icp_correction_rotation_deg` be evaluated near `0.03` and `5.0`. Those are
rejection guards, not calibration tolerances.

## Controller choice

The revised executor deliberately continues to send:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

This matches Dawood's `bringup.launch.py`, whose
`initial_joint_controller` is `joint_trajectory_controller`. The earlier
`scaled_joint_trajectory_controller` requirement came only from the generated
cuRobo planner's parameter file.

The UR scaled controller can account for teach-pendant and safety speed scaling
when advancing trajectory time, but it does not improve RGB-D or point-cloud
quality. Changing controllers is unnecessary for these acquisition changes.
The revised replay instead gives an unscaled trajectory up to ten times its
nominal duration plus ten seconds to finish (important with the pendant at
20%); if that limit is exceeded, it requests goal cancellation before stopping.

Useful read-only checks:

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

If the lab later chooses the scaled controller, it normally comes from the
installed UR driver/controller packages and can be selected as
`initial_joint_controller:=scaled_joint_trajectory_controller`; the action
name in the executor must then change too. Make that as a separate supervised
motion-system change, not as part of reconstruction tuning.

## Primary references

- Universal Robots scaled controller:
  <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html>
- Universal Robots calibration extraction:
  <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_calibration/doc/index.html>
- RealSense ROS wrapper launch parameters:
  <https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py>
- RealSense D435 tuning guidance:
  <https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/>
- RealSense D400 series datasheet:
  <https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf>
- RealSense D400 calibration tools:
  <https://dev.realsenseai.com/docs/calibration/>
- chrony documentation:
  <https://chrony-project.org/documentation.html>
- ROS 2 hand-eye calibration:
  <https://github.com/marcoesposito1988/easy_handeye2>
