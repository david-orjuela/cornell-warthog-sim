# Cornell Orchard Reconstruction and Simulation

A ROS 2 Humble project for reconstructing real orchard trees from RGB-D data collected with a Warthog-mounted UR5e camera and preparing the resulting 3D assets for use in Gazebo Fortress orchard environments.

The repository also preserves and documents inherited Cornell AgRobotics Lab simulation work involving a Clearpath Warthog, orchard worlds, mapping, perception, and navigation.

## Current objective

The active project goal is to create a reproducible pipeline that:

1. receives synchronized RGB, aligned depth, and camera-intrinsic data;
2. obtains the camera pose through TF;
3. generates a metric colored point cloud;
4. transforms each capture into a fixed robot frame;
5. registers and fuses multiple views;
6. exports the reconstruction as PLY;
7. records the raw data and metadata needed for offline reproduction; and
8. prepares selected reconstructions for later Gazebo integration.

The immediate validation target is an indoor artificial-tree scan. Real orchard-tree collection and simulator asset generation follow only after the indoor workflow is verified.

## Project status

This repository is under active reconstruction and migration.

### Inherited simulation baseline

* [x] Recovered the primary inherited ROS 2 workspace
* [x] Built the workspace cleanly with ROS 2 Humble
* [x] Launched the inherited Gazebo Fortress orchard world
* [x] Spawned the Clearpath Warthog
* [x] Initialized the simulated wheel controllers
* [x] Started the inherited LiDAR, RGB-D camera, IMU, and point-cloud bridges
* [x] Identified machine-specific paths and portability defects
* [x] Restored missing LIO-SAM submodule metadata on a repair branch
* [x] Retrieved Git LFS model and mesh assets
* [ ] Make all inherited launch and asset paths portable
* [ ] Revalidate mapping and perception workflows
* [ ] Revalidate autonomous navigation
* [ ] Add automated simulation evaluation

### RGB-D reconstruction pipeline

* [x] Reviewed the inherited ROS 1 reconstruction architecture
* [x] Separated geometry reconstruction from dormant-tree skeletonization
* [x] Identified the required Warthog RGB-D camera topics
* [x] Migrated the main reconstruction node to ROS 2 Humble
* [x] Migrated the UR5e motion-planning node to ROS 2
* [x] Replaced ROS 1 TF usage with TF2
* [x] Replaced ROS 1 point-cloud conversion utilities
* [x] Added move–settle–capture synchronization
* [x] Added configurable RGB-D, TF, filtering, and ICP parameters
* [x] Added persistent PLY and per-frame metadata output
* [x] Added a motion-safety interlock with motion disabled by default
* [x] Syntax-checked the migrated Python code
* [ ] Validate one live stationary RGB-D capture
* [ ] Verify the complete camera-to-robot TF chain
* [ ] Confirm metric scale and cloud orientation
* [ ] Confirm the active UR5e joint-state and trajectory-controller interfaces
* [ ] Complete a small supervised indoor raster scan
* [ ] Record and replay a complete ROS 2 bag
* [ ] Reconstruct real orchard trees
* [ ] Generate simplified visual and collision assets
* [ ] Insert a reconstructed tree into Gazebo

The ROS 2 reconstruction implementation is a first-pass migration. It has not yet been validated end to end against the live UR5e controller, physical camera calibration, or a completed multi-view scan.

## System overview

The intended runtime architecture is:

```text
UR5e-mounted RGB-D camera
          ↓
Jetson publishes ROS 2 camera data
          ↓
RGB + aligned depth + CameraInfo
          ↓
TF2 camera pose in the selected fixed frame
          ↓
Open3D RGB-D point-cloud generation
          ↓
TF-based initial placement
          ↓
Coarse-to-fine ICP correction
          ↓
Filtering and global fusion
          ↓
PLY point cloud + frame metadata
          ↓
Future mesh generation and Gazebo integration
```

Robot motion follows a coordinated capture workflow:

```text
plan motion
→ execute trajectory
→ wait for completion
→ allow the arm to settle
→ request one capture
→ process and save the frame
→ acknowledge completion
→ move to the next viewpoint
```

## Supported environment

The current target environment is:

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo Fortress / Ignition Gazebo 6
* Python 3.10
* Open3D
* OpenCV
* NumPy
* Clearpath ROS 2 simulation packages
* Universal Robots ROS 2 interfaces
* cuRobo with a CUDA-capable GPU for motion planning

Some inherited packages and assets may require additional dependencies documented in the migration inventory.

## Primary reconstruction inputs

The current Warthog UR5e camera configuration uses:

```text
/sensors/camera_jetson/color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/camera_info
```

Supporting interfaces include:

```text
/tf
/tf_static
/platform/joint_states
```

The actual UR5e joint-state topic and trajectory-controller action must be confirmed on the target system before enabling robot motion.

## Reconstruction outputs

The ROS 2 reconstruction node publishes:

```text
/tree_scan/latest_tf_cloud
/tree_scan/global_cloud
```

The default reconstruction output is:

```text
~/tree_scans/indoor_model_tree/global_point_cloud.ply
```

Per-frame artifacts may include:

* RGB image;
* depth image;
* camera intrinsics;
* camera-to-target-frame transform;
* ICP correction;
* registration fitness and RMSE;
* acceptance or rejection state;
* and a final reconstruction summary.

Large point clouds, rosbags, datasets, maps, and generated meshes should remain outside normal Git history unless a specific artifact is intentionally versioned.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --recursive \
  --yes

colcon build --symlink-install

source install/setup.bash
```

The exact external dependencies required by cuRobo, the physical UR driver, and the inherited simulator may require separate installation.

## Reconstruction-only launch

Begin with robot motion disabled:

```bash
ros2 launch reconstruct scan_tree.launch.py \
  execute_motion:=false
```

Request one synchronized capture:

```bash
ros2 topic pub --once \
  /capture_alert \
  std_msgs/msg/Bool \
  "{data: true}"
```

Before triggering a capture, verify that the configured RGB, aligned-depth, camera-info, and TF interfaces are active.

## Required preflight checks

Check the camera streams:

```bash
ros2 topic hz \
  /sensors/camera_jetson/color/image_raw
```

```bash
ros2 topic hz \
  /sensors/camera_jetson/aligned_depth_to_color/image_raw
```

Inspect camera information:

```bash
ros2 topic echo \
  /sensors/camera_jetson/aligned_depth_to_color/camera_info \
  --once
```

Use the reported camera frame to verify TF:

```bash
ros2 run tf2_ros tf2_echo \
  base_link \
  <camera_optical_frame>
```

Inspect the available trajectory actions and controllers before any physical motion:

```bash
ros2 action list -t | grep follow_joint_trajectory
```

```bash
ros2 control list_controllers
```

```bash
ros2 topic echo /platform/joint_states --once
```

Do not assume that `/platform/joint_states` contains the six UR5e joints merely because the topic exists.

## Motion safety

Physical UR5e motion is disabled by default:

```yaml
execute_motion: false
```

Do not enable it until all of the following have been verified:

* the correct trajectory-controller action;
* all six UR5e joint states;
* the robot-specific kinematic calibration;
* the camera hand–eye transform;
* the cuRobo robot configuration;
* the collision environment;
* the scan-plane axes;
* the start pose;
* the camera standoff distance;
* and the available physical workspace.

Initial motion tests must use a small raster, low speed, conservative limits, and direct supervision.

ICP is intended only to correct small residual alignment errors. It must not be treated as a replacement for accurate TF or camera calibration.

## Repository documentation

The main project records are under `docs/`.

```text
docs/
├── project_charter.md
├── migration_inventory.md
├── provenance.md
├── inherited_work.md
├── archive/
└── logbook/
```

Important documents include:

* [`docs/project_charter.md`](docs/project_charter.md)
  Current project objective, deliverables, scope, blockers, and definition of done.

* [`docs/migration_inventory.md`](docs/migration_inventory.md)
  ROS 1-to-ROS 2 migration status, inherited repository differences, and remaining technical work.

* [`docs/inherited_work.md`](docs/inherited_work.md)
  Disposition of inherited components: adapt, preserve, defer, archive, or exclude.

* [`docs/provenance.md`](docs/provenance.md)
  Sources, authorship, modifications, licenses, data lineage, and unresolved provenance questions.

* [`docs/logbook/`](docs/logbook/)
  Dated technical notes documenting repository reconstruction, project handoff, scope changes, debugging, and migration work.

* [`docs/archive/`](docs/archive/)
  Historical planning documents retained for context but no longer treated as the active project definition.

## Scope

### Active scope

* ROS 2 RGB-D acquisition
* camera and robot TF validation
* metric point-cloud generation
* multi-view registration and fusion
* persistent PLY output
* rosbag-based offline reproducibility
* supervised indoor scanning
* real orchard-tree data collection
* mesh and Gazebo asset preparation
* documentation and evaluation

### Deferred scope

* full cleanup of every inherited Gazebo asset;
* broad mapping and navigation benchmarking;
* learned-policy deployment;
* Nav2 integration;
* automated multi-season evaluation;
* complete dormant-versus-leafy navigation experiments;
* and publication-scale seasonal robustness studies.

These remain potential downstream applications of the reconstruction and simulation infrastructure.

## Data and large artifacts

The following should generally be stored outside the source repository:

* rosbags;
* raw RGB-D datasets;
* generated PLY and PCD files;
* high-resolution meshes;
* trained model weights;
* maps;
* experiment videos;
* and large intermediate outputs.

Each external dataset or artifact should retain:

* collection date;
* camera and robot configuration;
* relevant calibration files;
* topic names;
* coordinate frames;
* parameter file;
* source commit;
* operator notes;
* and processing history.

## Attribution and provenance

This repository combines:

* inherited Cornell AgRobotics Lab work;
* upstream open-source robotics packages;
* recovered workstation files;
* project-specific ROS 2 migration work;
* and newly created documentation.

Do not assume that every inherited file is original Cornell code or cleared for redistribution.

See [`docs/provenance.md`](docs/provenance.md) before publishing, redistributing, or incorporating inherited models, meshes, scripts, or third-party code into a release.

## Project history

The repository was initially scaffolded around a broad orchard-simulation and autonomous-navigation objective.

Following the technical handoff on July 17, 2026, the active assignment changed to:

> Inherit and migrate the existing tree-reconstruction workflow, connect it to the Warthog’s UR5e-mounted RGB-D camera, validate it indoors, collect real orchard-tree data, and prepare reusable 3D assets for later simulation use.

The original simulation-centered charter is retained under:

```text
docs/archive/pre-2026-07-17-simulation-project-charter.md
```

It is preserved as project history and should not be interpreted as the current definition of done.

