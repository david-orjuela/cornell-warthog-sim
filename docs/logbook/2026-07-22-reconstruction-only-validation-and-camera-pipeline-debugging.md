# 2026-07-22 — Reconstruction-Only Validation and Camera Pipeline Debugging

## Day Objective

The primary goal for Wednesday was to validate the ROS 2 reconstruction pipeline independently of UR5e motion.

The planned validation sequence was:

1. Run only the RGB-D reconstruction node.
2. Receive synchronized color, aligned depth, and camera-intrinsics messages from the UR5e-mounted Jetson/RealSense camera.
3. create one colored point cloud.
4. place it in a consistent coordinate frame.
5. save the result as a PLY file.
6. inspect the cloud before enabling any robotic-arm motion.
7. only after the stationary pipeline works, proceed to controlled multi-view captures with the UR5e.

This separation was intentional. A stationary-camera test isolates RGB-D conversion, camera intrinsics, filtering, point-cloud generation, file output, and basic fusion from the additional complexity of arm motion, dynamic TF, hand–eye calibration, and trajectory execution.

---

## Progress Check and Reconstruction Strategy

Prepared a concise technical explanation for a progress meeting with Divyanth.

The reconstruction workflow was framed as two validation stages:

### Stage 1 — Stationary camera

Validate:

- synchronized RGB and aligned depth
- camera intrinsics
- depth scale
- color conversion
- point-cloud creation
- filtering
- PLY export
- repeated-frame behavior

Capturing two nearly identical stationary frames is only a weak ICP test because the point clouds should already overlap. It is mainly a pipeline sanity check.

### Stage 2 — Controlled camera motion

Once the stationary pipeline works:

- move the UR5e to a controlled second viewpoint
- use robot TF to place the new cloud approximately into a shared frame
- use ICP to refine the remaining alignment error
- inspect whether the fused result preserves trunk, branches, leaves, and apples without duplication or blur

The key architectural summary was:

> TF provides the pose prior; ICP provides the registration refinement.

ICP should not estimate the complete arm motion from scratch. The robot kinematics and TF tree should provide the approximate camera pose, while ICP corrects smaller residual errors caused by calibration, timing, depth noise, and imperfect transforms.

---

## Registration and ICP Review

Clarified the meaning of point-cloud registration:

> Registration is the process of finding the rigid rotation and translation that align one point cloud with another representation of the same physical scene.

The reconstruction node uses coarse-to-fine point-to-plane ICP.

The implementation confirms this through:

- voxel downsampling at multiple scales
- normal estimation using a hybrid KD-tree neighborhood
- `TransformationEstimationPointToPlane()`
- iterative convergence tests based on RMSE, fitness, and maximum iterations

### Point-to-plane ICP

For each source point, ICP finds a nearby target correspondence and minimizes the source point’s distance to the target surface’s local tangent plane.

This requires surface normals, which are estimated from neighboring points.

Point-to-plane ICP is generally more suitable than basic point-to-point ICP for RGB-D surface reconstruction because it uses local surface orientation and often converges faster when the initial estimate is already close.

---

## Multiscale Registration Interpretation

Confirmed that the reconstruction node runs the scales in this order:

`SN3 → SN2 → SN1 → SN0`

This is a coarse-to-fine sequence.

| Scale | Voxel size | Correspondence distance | Maximum iterations | Role |
|---|---:|---:|---:|---|
| `SN3` | 0.05 m | 0.05 m | 50 | Coarse 5 cm alignment |
| `SN2` | 0.02 m | 0.03 m | 30 | Intermediate refinement |
| `SN1` | 0.01 m | 0.018 m | 20 | Fine 1 cm refinement |
| `SN0` | 0.0 m | 0.012 m | 15 | Full-resolution final refinement |

`SN0` having `voxel_size_m: 0.0` does not mean zero resolution. It is used as a sentinel meaning:

> Do not voxel-downsample; use the original point cloud.

The transform estimated at each scale is passed as the initial guess for the next finer scale.

The current ROS 2 thresholds are somewhat more permissive than the original ROS 1 values, especially at intermediate and fine scales. This was intended to reduce unnecessary rejection during initial testing but may later need tightening after real reconstruction quality is measured.

---

## ROS 2 Preflight Results

Ran the live ROS 2 interface checks.

### ROS domain behavior

The UR5e controller and joint-state interfaces appeared only when using:

`ROS_DOMAIN_ID=26`

Without the correct domain, only a smaller camera-oriented ROS graph was visible.

This explained why topics and actions seemed to disappear between terminals. Every terminal used for testing must share the same ROS domain and sourced workspace.

### UR5e action servers discovered

The following `FollowJointTrajectory` actions were available when the UR stack was online:

- `/joint_trajectory_controller/follow_joint_trajectory`
- `/passthrough_trajectory_controller/follow_joint_trajectory`
- `/scaled_joint_trajectory_controller/follow_joint_trajectory`

### Controller state

`ros2 control list_controllers` showed:

- `joint_state_broadcaster` active
- `io_and_status_controller` active
- `speed_scaling_state_broadcaster` active
- `force_torque_sensor_broadcaster` active
- `tcp_pose_broadcaster` active
- `scaled_joint_trajectory_controller` inactive
- `joint_trajectory_controller` inactive

For later physical motion, the scaled controller is the likely target, but it must first be activated and verified through the UR external-control startup sequence.

### Correct joint-state topic

The active UR5e joint-state topic is:

`/joint_states`

The earlier configured topic, `/platform/joint_states`, was not present in the current arm-only ROS graph.

### Camera topics confirmed

The reconstruction inputs were active:

- `/sensors/camera_jetson/color/image_raw`
- `/sensors/camera_jetson/aligned_depth_to_color/image_raw`
- `/sensors/camera_jetson/aligned_depth_to_color/camera_info`

Observed image rates were approximately 16–17 Hz after startup.

The aligned-depth camera information reported:

- frame: `camera_jetson_color_optical_frame`
- resolution: 640 × 480
- focal lengths near 605.6 pixels
- principal point near `(323.95, 247.77)`
- `plumb_bob` distortion model with zero reported coefficients

### Time configuration

No `/clock` topic was present during the physical-camera test. The reconstruction launch was therefore changed to:

`use_sim_time:=false`

The camera timestamps were already wall-clock timestamps.

---

## TF Validation

When the UR stack was active, a transform from `base_link` to a camera optical frame could be resolved after an initial TF discovery delay.

Later in the day, the robot appeared to be powered off or its ROS driver was no longer active. At that point, the available ROS graph contained the camera topics and `/tf_static`, but no:

- `/tf`
- `/joint_states`
- controller topics
- active robot-state TF tree

The reconstruction node then correctly reported:

`TF unavailable: base_link <- camera_jetson_color_optical_frame`

This confirmed that the RGB-D synchronizer was functioning and that processing had reached the TF lookup stage. The remaining failure was specifically the absence of the robot-side `base_link` frame.

---

## Camera-Only Reconstruction Mode

To continue testing without the robot, the reconstruction target was changed from `base_link` to the camera’s own optical frame:

`target_frame: camera_jetson_color_optical_frame`

The camera frame override was also set or allowed to resolve to the same frame.

An identity-transform case was added to `_lookup_transform_matrix()`:

`if target_frame == source_frame: return np.eye(4, dtype=np.float64)`

This allows the point cloud to remain in camera optical coordinates without querying TF.

In camera optical coordinates:

- positive X points right
- positive Y points down
- positive Z points forward

This mode can validate point-cloud creation and PLY output, but it does not validate:

- robot kinematics
- dynamic TF
- hand–eye calibration
- camera motion
- multi-view placement in `base_link`

---

## Reconstruction-Only Launch and Capture Trigger

Launched the node with motion disabled:

`ros2 launch reconstruct scan_tree.launch.py execute_motion:=false use_sim_time:=false`

The node started successfully and reported:

- capture topic: `/capture_alert`
- configured Jetson RGB-D inputs
- output path: `~/tree_scans/indoor_model_tree/global_point_cloud.ply`

Manual capture requests were sent using:

`ros2 topic pub --once /capture_alert std_msgs/msg/Bool "{data: true}"`

The node received and queued the capture requests correctly.

Initially, the queued count increased without producing `/capture_done` or a PLY. Once the camera and TF conditions changed, the callback progressed further and exposed the next runtime issue.

This confirmed that the trigger path itself was functional.

---

## RGB-D Conversion Failure

After enabling camera-only operation, the callback successfully reached RGB-D point-cloud creation but Open3D raised:

`Image can only be initialized from buffer of uint8, uint16, or float!`

The failure occurred at:

`o3d.geometry.Image(depth_array)`

### Depth diagnostics

Additional logging confirmed:

- encoding: `16UC1`
- shape: 480 × 640
- dtype: `uint16`
- native little-endian representation: `<u2`
- contiguous memory: true
- depth scale: `1000.0`

These values are conceptually valid for Open3D RGB-D input.

The depth conversion code was updated to:

- determine units independently from dtype normalization
- preserve `16UC1` as `uint16`
- preserve `32FC1` as `float32`
- reject unsupported or negative integer depth values
- ensure native byte order
- ensure contiguous arrays
- log encoding, dtype, shape, and scale

Despite this normalization, Open3D continued to reject the valid `uint16` array.

---

## Python Environment Warning

Every launch produced a SciPy warning:

`SciPy requires NumPy < 1.25.0, but NumPy 1.26.4 is installed`

The observed environment included:

- NumPy 1.26.4
- SciPy 1.8.0
- Open3D 0.19.0

The Open3D image-constructor failure may be related to binary incompatibility or mixed system/pip Python packages, although this was not proven conclusively during the day.

The warning itself did not prevent the ROS node from starting, but the Python environment should be made internally consistent before relying on it for repeatable experiments.

---

## Proposed Direct NumPy Back-Projection

Because Open3D rejected an otherwise valid depth array, a fallback was designed to bypass `o3d.geometry.Image` and `RGBDImage.create_from_color_and_depth()`.

The proposed replacement directly back-projects each valid depth pixel using the pinhole-camera equations:

`X = (u - cx) Z / fx`

`Y = (v - cy) Z / fy`

`Z = depth`

The matching RGB values are normalized to `[0, 1]`, and the resulting arrays are assigned directly to:

- `o3d.utility.Vector3dVector` for points
- `o3d.utility.Vector3dVector` for colors

This would preserve the remainder of the pipeline:

- TF or identity placement
- filtering
- multiscale point-to-plane ICP
- global fusion
- ROS `PointCloud2` publication
- PLY output

This fallback was proposed but had not yet been implemented and validated by the end of the recorded work.

---

## Parameter Concepts Reviewed

Reviewed the main registration and filtering parameters in preparation for later tuning.

### Voxel size

A voxel is a 3D grid cell. Voxel downsampling replaces all points within each cell with a representative point.

Larger voxels:

- reduce point count
- increase speed
- suppress fine detail
- broaden the ICP convergence basin

Smaller voxels:

- preserve detail
- increase runtime and memory use
- require a better initial pose

### Maximum correspondence distance

This is the largest source-to-target separation allowed when forming ICP matches.

If too small:

- valid overlap may be ignored
- fitness may become low
- registration may fail

If too large:

- unrelated leaves or branches may be matched
- ICP may converge to an incorrect alignment

### Convergence criteria

The Open3D ICP loop stops when one of these occurs:

- relative RMSE improvement becomes sufficiently small
- relative fitness improvement becomes sufficiently small
- maximum iterations are reached

These stopping thresholds are different from the final acceptance thresholds.

### Fitness and RMSE acceptance

Fitness is the fraction of source points that obtain valid target correspondences.

Higher fitness is generally better.

Inlier RMSE measures the geometric error among accepted correspondences.

Lower RMSE is generally better.

A completed ICP result can converge internally and still be rejected if its final fitness or RMSE fails the configured acceptance thresholds.

### Outlier rejection

The node uses radius-outlier removal.

A point survives only if it has at least a configured number of neighboring points within a configured radius.

This removes isolated depth noise but can also erase thin leaves, apple edges, and small branches if configured too aggressively.

### Normal estimation

Point-to-plane ICP requires local surface normals.

Normals are estimated from neighboring points using a hybrid KD-tree search with:

- a search radius linked to voxel size and correspondence distance
- a maximum of 40 neighbors

The neighborhood must be large enough to estimate a stable surface direction but not so large that it averages across separate leaves, branches, or curved structures.

---

## End-of-Day Status

By the end of Wednesday, July 22:

- The ROS 2 workspace built successfully.
- The reconstruction node launched successfully with motion disabled.
- The correct Jetson RGB-D topics were configured.
- `ROS_DOMAIN_ID=26` was identified as necessary for seeing the UR5e ROS graph.
- The correct arm joint-state topic was identified as `/joint_states`.
- Three trajectory action servers were discovered.
- The relevant trajectory controllers were present but inactive.
- The camera streams and intrinsics were verified.
- Physical-camera testing used `use_sim_time:=false`.
- The reconstruction trigger path worked.
- The synchronized RGB-D callback worked when the camera streams were available.
- TF placement into `base_link` worked only while the robot-side TF tree was active.
- A camera-only identity-frame mode was added to permit testing with the robot off.
- The pipeline advanced through image synchronization, frame selection, TF handling, and depth conversion.
- Depth input was confirmed as valid `16UC1`, 640 × 480, `uint16`, with a 1000.0 scale.
- Open3D still rejected the normalized depth array during `geometry.Image` construction.
- No successful PLY was produced during the recorded work.
- No UR5e motion was attempted.
- A direct NumPy pinhole back-projection was prepared as the next workaround.
- The mixed NumPy/SciPy/Open3D environment remained a probable technical risk.

---

## Next Steps

1. Replace or temporarily bypass Open3D’s RGB-D image constructor using direct NumPy back-projection.
2. rebuild and confirm that one camera-only capture produces a colored Open3D point cloud.
3. verify `/capture_done` publication.
4. verify creation of `global_point_cloud.ply`.
5. inspect the first cloud in Open3D, CloudCompare, or RViz for:
   - orientation
   - scale
   - color correctness
   - depth cutoff
   - missing regions
   - near-depth saturation
   - noise and isolated points
6. capture several stationary frames and verify that fusion does not blur or duplicate surfaces.
7. restore `target_frame: base_link` when the UR5e TF tree is online.
8. activate and verify the selected trajectory controller.
9. run a very small supervised move–settle–capture sequence.
10. only after a successful multi-view test, tune voxel, outlier, correspondence, fitness, RMSE, and depth parameters for leaves, branches, and apples.
