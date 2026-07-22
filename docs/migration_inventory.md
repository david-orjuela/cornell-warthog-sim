# Migration and Repository Inventory

## Purpose

This document tracks two related but distinct efforts:

1. the active migration of Dawood's ROS 1 tree-scanning and reconstruction workflow to ROS 2 Humble; and
2. the disposition of differences found between the clean inherited simulator repository and the recovered desktop workspaces.

Status terms:

- **Implemented — unvalidated:** code exists and parses, but has not been proven against the live system.
- **Pending:** required work or verification remains.
- **Deferred:** preserve for later; not part of the immediate reconstruction milestone.
- **Archive:** keep as evidence or output, but do not treat as active source.
- **Exclude:** do not import into the active codebase unless the scope changes.

---

## A. Active ROS 1 → ROS 2 Humble Migration

| Area | Inherited state | Current action | Status | Required validation |
| --- | --- | --- | --- | --- |
| Package build system | ROS 1 catkin package | Convert to `ament_cmake` + `ament_cmake_python`; install Python executables, launch, and config | Implemented — unvalidated | Clean `colcon build` in the target workspace |
| Node API | `rospy` | Replace with `rclpy.node.Node` | Implemented — unvalidated | Start both nodes and inspect parameters/logging |
| Parameters | ROS 1 global parameter server and hardcoded values | Use node-scoped ROS 2 YAML parameters | Implemented — unvalidated | Confirm launch-time overrides and parameter types |
| Trajectory action | `actionlib.SimpleActionClient` | Use ROS 2 `rclpy.action.ActionClient` and configurable action name | Implemented — unvalidated | Identify and connect to the actual active action server |
| Controller handling | Old controller names and ROS 1 controller-manager calls | Do not switch controllers automatically; require an already active controller | Implemented — unvalidated | Run `ros2 control list_controllers` and verify namespace |
| Joint-state ordering | Positional assumptions and manual swapping | Map positions by joint name into cuRobo/controller order | Implemented — unvalidated | Confirm the selected topic includes all six UR5e joints |
| cuRobo tensors | GPU tensors passed toward ROS messages | Convert trajectory values to CPU Python lists | Implemented — unvalidated | Plan a trajectory without executing motion |
| CUDA dependency | Hardcoded `cuda:0` assumption | Check CUDA availability before cuRobo initialization | Implemented — unvalidated | Test on the actual laptop/workstation GPU |
| Quaternion handling | ROS and cuRobo conventions mixed implicitly | Treat cuRobo as `wxyz`; ROS transforms as `xyzw`; normalize where required | Implemented — unvalidated | Verify a known orientation and scan-plane direction |
| Scan path | Long accumulated relative-motion sequence | Generate a parameterized absolute snake/raster around the verified start pose | Implemented — unvalidated | Visualize or dry-run all waypoints before motion |
| Home behavior | Hardcoded joint pose | Optional return to recorded scan-start pose | Implemented — unvalidated | Confirm controller acceptance and safe workspace |
| Motion safety | Main code could run the wrong mode | Default `execute_motion: false`; require explicit enablement | Implemented — unvalidated | Confirm launch default on the target system |
| Capture coordination | Continuous processing; incomplete alert publisher | Add move–settle–capture request and `/capture_done` acknowledgement | Implemented — unvalidated | Trigger several manual captures without arm motion |
| Camera topics | ROS 1 Azure Kinect topic names | Parameterize and default to `camera_jetson` ROS 2 topics | Implemented — unvalidated | Verify frequency, QoS, encodings, and frame IDs |
| Camera synchronization | ROS 1 message filters with 1 ms slop | ROS 2 `message_filters` with sensor-data QoS and configurable tolerance | Implemented — unvalidated | Measure real timestamp offsets and accepted callback rate |
| Image encodings | BGRA and `16UC1` assumed | Support common RGB/BGR/RGBA/grayscale encodings and `16UC1`/`32FC1` depth | Implemented — unvalidated | Capture representative messages from the live camera |
| Depth units and limits | Hardcoded values; incorrect 1 m/2 m comment | Infer depth scale by encoding; parameterize minimum/maximum depth | Implemented — unvalidated | Inspect reconstructed metric scale and clipping |
| Camera intrinsics | ROS 1 CameraInfo | Read aligned-depth-to-color CameraInfo in ROS 2 | Implemented — unvalidated | Confirm `K`, width, height, and matching image dimensions |
| TF | ROS 1 `tf.TransformListener` and hardcoded frames | Use `tf2_ros.Buffer`/`TransformListener`; read camera frame from headers | Implemented — unvalidated | Verify fixed-frame-to-optical-frame transform at capture timestamps |
| Point-cloud generation | Open3D RGB-D back-projection | Retain geometry-first colored reconstruction; correct BGR/RGB ordering | Implemented — unvalidated | Produce one stationary colored PLY |
| ROS cloud publishing | `ros_numpy` / ROS 1 helper | Construct ROS 2 `PointCloud2` directly with packed RGB | Implemented — unvalidated | View latest and global clouds in RViz2 |
| Semantic segmentation | YOLO/BiSeNet trunk/branch masks; black geometry discarded | Bypass for initial leafy-tree reconstruction | Implemented by omission | Confirm full foliage/background geometry is retained |
| ICP | Global multiscale point-to-plane ICP | Retain as bounded correction with configurable thresholds and local target crop | Implemented — unvalidated | Compare TF-only vs. TF+ICP on two controlled views |
| ICP safety limits | No explicit correction bound | Reject implausible translation/rotation corrections | Implemented — unvalidated | Log and inspect corrections across controlled captures |
| Filtering | Hardcoded voxel/radius values | Parameterize local/global voxel and radius-outlier settings | Implemented — unvalidated | Tune without deleting small leaves and twigs |
| Capture state | Continuous automatic re-enable | Queue exactly one synchronized frame per request; retain pending request on temporary TF failure | Implemented — unvalidated | Repeated manual trigger test |
| Output | Published cloud only; no persistent final model | Save fused PLY after each capture plus per-frame RGB/depth/intrinsics/TF/ICP/metrics | Implemented — unvalidated | Verify unique paths, file integrity, and metadata consistency |
| Live timeout | Eight-second watchdog suitable only for bag replay | Remove unsafe live idle shutdown | Implemented — unvalidated | Leave node idle between manual captures |
| Launch | ROS 1 launch files and Dawood-specific paths | Add ROS 2 Python launch file and configurable output path | Implemented — unvalidated | Launch reconstruction-only with motion disabled |
| Time source | ROS 1/live assumptions | Parameterize `use_sim_time`; require consistent camera, TF, and controller clocks | Pending | Confirm `/clock` bridge in Gazebo and wall time on hardware |
| Rosbag | ROS 1 `rosbag record/play` launch | Design ROS 2 `ros2 bag` recording and replay workflow | Pending | Record and replay all required topics |
| Calibration/config | Physical UR calibration, camera mount, `ur5e.yml`, and `collision_table.yml` external or missing | Obtain and verify authoritative files | Pending | Validate against the actual Warthog-mounted UR5e |
| Dependencies | ROS 1 packages plus YOLO/BiSeNet remnants | Declare ROS 2 runtime dependencies; isolate optional semantic stack | Pending | Fresh-machine installation test |

---

## B. Clean Simulator Repository vs. Recovered Desktop Workspace

| Difference or artifact | Current interpretation | Disposition | Priority / next action |
| --- | --- | --- | --- |
| `.gitmodules` missing for `src/LIO-SAM` | Repository tracked a submodule commit but lacked its source URL | Keep the repair branch; do not silently merge | Ask Nidhish whether he wants a PR, retained branch, or removal |
| `src/LIO-SAM` commit | Expected commit is `08af3f32f01725372d4269838dc44c19c6d9e76b` from TixiaoShan/LIO-SAM | Preserve exact pointer | Reverify during a clean recursive clone |
| Git LFS pointer files | Orchard geometry appeared as ~130-byte pointer files until `git lfs pull` | Keep LFS tracking; do not duplicate binaries outside the intended storage model | Document Git LFS as a setup requirement |
| LIO-SAM `config/params.yaml` | Project-specific configuration differs between clean and recovered copies | Preserve both versions and diff | Deferred unless reconstruction or later mapping depends on it |
| LIO-SAM `launch/run.launch.py` | Recovered launch changes may include machine/project-specific behavior | Preserve patch; do not choose a winner without context | Deferred |
| LIO-SAM `config/rviz2.rviz` | Visualization-only differences | Retain useful display settings if later needed | Low priority |
| `orchard_description/scripts/generate_orchard_new_trees.py` | Competing generator versions may encode later local work | Preserve both and identify authoritative/latest intent | Downstream priority for seasonal asset integration |
| `orchard_description/worlds/orchard_final_high_res.sdf` | Includes structural and absolute-path differences | Preserve; replace home paths only when simulator integration resumes | Deferred |
| `orchard_perception/launch/infrastructure.launch.py` | Recovered orchestration differs from clean baseline | Preserve patch | Deferred |
| `orchard_perception/launch/main_launcher.py` | Recovered orchestration differs from clean baseline | Preserve patch | Deferred |
| `orchard_perception/rviz/orchard_mapping.rviz` | Low-risk visualization differences | Retain externally or as a documented patch | Low priority |
| `orchard_perception/src/temporal_projector.cpp` | Two substantially different experimental implementations | Preserve both; do not merge or overwrite | Ask Nidhish which experiment each version represents |
| Absolute `/home/divyanth`, `/home/nidhish`, and `/home/nidhish27` paths | Machine-specific portability defects | Replace with package share paths, `model://`, resource paths, and launch arguments only when touched | Deferred except where they block a required run |
| `/etc/clearpath` generated files | Root-owned setup path causes permission failures; old generated files can mask the issue | Use a user-writable `setup_path`; never launch ROS/Gazebo with `sudo` | Document for future simulator reproduction |
| `final_colored_orchard.pcd` | Experimental result, not source | Archive outside normal source tree or under a results store | Preserve with context and generation metadata if recoverable |
| `frames_*.gv` and `frames_*.pdf` | Generated TF diagrams | Archive under experiment artifacts/docs | Preserve; do not treat as authoritative without date/config |
| `build-reproduction.log` | Locally generated reproduction evidence | Archive or attach to an issue; do not maintain as source | Keep only if useful for handoff |
| `meshes/` | Actual use and authorship not fully known | Preserve; do not redistribute externally until provenance is verified | Inspect references and licenses before release |
| `urdf/` | May contain dormant or machine-specific material | Preserve | Low priority unless required by current TF/controller setup |
| `orchard_navigation_rl_ws` | Earlier behavior-cloning/learned-navigation work, separate from current authoritative workspace | Archive and exclude from active reconstruction work | Revisit only if Divyanth redirects the project |

---

## C. Component Disposition for the Current Project

| Component | Disposition | Reason |
| --- | --- | --- |
| Dawood RGB-D acquisition and Open3D reconstruction concepts | **Adapt** | Core of the active deliverable |
| Dawood ROS 1 interfaces | **Replace** | Incompatible with ROS 2 Humble system |
| Dawood YOLO/BiSeNet semantic coloring | **Defer** | Specialized for dormant trunk/branch analysis and likely removes foliage |
| Dawood skeletonization and branch measurement | **Archive / optional later** | Not required for a geometry-first leafy reconstruction |
| ROS 2 migrated motion/reconstruction package | **Active** | Current implementation layer; runtime validation pending |
| Clean `Spatio-Temporal-Mapping` repository | **Preserve as baseline** | Authoritative simulator baseline identified during handoff |
| Recovered `cornell_orchard_ws` | **Preserve as recovery source** | May contain uncommitted experimental changes; originals should remain untouched |
| Orchard worlds, meshes, and generators | **Preserve for downstream integration** | Eventual destination for reconstructed assets |
| LIO-SAM and mapping configuration | **Defer** | Not required for the stationary-frame or small indoor reconstruction milestones |
| Learned navigation stack | **Exclude from current scope** | Separate prior work with no immediate dependency |
| Generated PCD/PLY, bags, images, logs, and TF diagrams | **Store as artifacts/data** | Outputs and evidence, not source code |

---

## D. Validation Order

1. clean-build the ROS 2 package;
2. launch with `execute_motion:=false`;
3. inspect RGB, depth, CameraInfo, encodings, rates, and QoS;
4. verify the fixed-frame-to-camera optical TF at the image timestamp;
5. trigger and inspect one stationary PLY;
6. publish and view latest/global ROS 2 point clouds;
7. identify the UR5e controller action and six-joint state source;
8. dry-run or visualize the raster without executing motion;
9. perform one supervised low-speed move and second capture;
10. compare TF-only and TF+ICP alignment;
11. complete a small indoor raster;
12. record a ROS 2 bag and replay offline;
13. collect orchard data only after the indoor pipeline is repeatable.
