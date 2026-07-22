# Cornell Warthog RGB-D Tree Reconstruction Pipeline

## Document Status

This is the active project charter as of July 21, 2026. It supersedes the earlier charter centered on building a general-purpose Warthog orchard-navigation simulation platform.

The earlier simulation work remains relevant as inherited infrastructure and as the eventual destination for reconstructed tree assets, but it is no longer the immediate project deliverable.

---

## Project Objective

Develop a reproducible ROS 2 Humble pipeline that adapts Dawood's inherited RGB-D tree-reconstruction workflow to the Cornell Warthog's UR5e-mounted camera, validates the pipeline on an indoor model tree, and produces reusable three-dimensional reconstructions of leafy orchard trees.

The immediate output is a metrically scaled colored point cloud, preserved with the raw data and metadata needed for repeatable offline processing. Mesh generation and Gazebo integration are downstream stages after the reconstruction itself is validated.

---

## Motivation

The inherited Gazebo orchard primarily represents dormant or winter trees. The laboratory needs real-tree-derived assets representing denser leafy canopies and current-season orchard conditions so that future perception and navigation systems can be evaluated under seasonal visual change.

Because David's remaining time at Cornell provides direct access to the physical Warthog, UR5e, RGB-D camera, indoor model tree, orchard, and lab personnel, hardware validation and raw data collection take priority over broad simulator cleanup.

---

## Active Technical Scope

The project will:

1. preserve the reusable acquisition and registration concepts from Dawood's ROS 1 code;
2. migrate the required nodes and package structure to ROS 2 Humble;
3. consume synchronized RGB, aligned depth, and camera intrinsics from the UR5e-mounted `camera_jetson` device;
4. obtain the camera pose from TF at the sensor timestamp;
5. create colored Open3D point clouds in a fixed robot frame;
6. use bounded coarse-to-fine ICP only as a refinement to valid TF-based placement;
7. coordinate arm movement through a move–settle–capture acknowledgement sequence;
8. save per-frame inputs, transforms, registration metrics, and fused PLY output;
9. validate the process first on a stationary frame, then on a small supervised indoor scan;
10. record raw RGB-D, TF, joint-state, and calibration data for offline replay;
11. collect several real leafy-tree datasets while physically at Cornell; and
12. prepare successful reconstructions for later cleaning, meshing, simplification, and Gazebo packaging.

---

## Intended System Architecture

```text
UR5e-mounted RGB-D camera
        ↓
ROS 2 camera driver on the Jetson
        ↓
RGB + aligned depth + CameraInfo
        ↓
TF2 lookup at the image timestamp
        ↓
Open3D colored point cloud in camera frame
        ↓
Transform into a fixed robot frame
        ↓
Bounded multiscale ICP refinement
        ↓
Filtering and global fusion
        ↓
PLY/PCD + per-frame metadata + ROS PointCloud2
        ↓
Offline cleanup and mesh generation
        ↓
Future Gazebo visual and collision assets
```

The motion side follows:

```text
verified UR5e joint state
        ↓
cuRobo plan to an absolute raster viewpoint
        ↓
ROS 2 FollowJointTrajectory execution
        ↓
settling delay
        ↓
capture request
        ↓
reconstruction acknowledgement
        ↓
next viewpoint
```

---

## Supported Environment

The active target environment is:

```text
Ubuntu 22.04 / Pop!_OS 22.04 base
ROS 2 Humble
Gazebo Fortress where simulation is used
Python 3.10
Open3D
cuRobo with CUDA-capable hardware for motion planning
Warthog + UR5e + Jetson/RealSense RGB-D camera
```

Primary camera inputs currently identified:

```text
/sensors/camera_jetson/color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/camera_info
```

Supporting interfaces include TF, static TF, a verified UR5e joint-state source, and an active ROS 2 `FollowJointTrajectory` action server.

---

## Definition of Done

### Minimum deliverable for the Cornell visit

The minimum deliverable is complete when:

- the required reconstruction logic exists in a buildable ROS 2 package;
- one synchronized stationary RGB-D frame is converted into a correctly scaled and oriented PLY in the selected fixed frame;
- the live TF chain and camera intrinsics are verified;
- the final output and per-frame metadata are written without manual source edits;
- a short rosbag preserves the required RGB-D, TF, joint-state, and calibration inputs;
- one small supervised indoor model-tree scan completes through the move–settle–capture workflow;
- raw data from several real orchard trees is collected, subject to safe robot access and field conditions;
- at least two usable reconstructed tree point clouds are produced; and
- another researcher can reproduce the stationary-frame and offline-processing workflows from the documentation.

### Target deliverable

A stronger result includes:

- three to six successfully captured real trees;
- multiple accepted multi-view reconstructions;
- validated metric scale, orientation, and coordinate-frame conventions;
- preserved RGB, depth, CameraInfo, TF, ICP corrections, acceptance metrics, and parameter snapshots;
- a documented naming and data-retention convention;
- one cleaned reconstruction prepared for mesh generation; and
- one candidate visual asset and simplified collision representation prepared for later Gazebo insertion.

### Stretch deliverable

A stretch result includes:

- six to ten captured trees;
- multiple reconstructed models integrated into the inherited orchard world;
- an automated model/world-generation step; and
- an initial dormant-versus-leafy simulation experiment.

---

## Current State as of July 21, 2026

Completed or established:

- the inherited `cornell_orchard_ws` simulator was clean-built successfully under ROS 2 Humble;
- Gazebo Fortress loaded the orchard world and spawned the Warthog with controllers and simulated sensors;
- the GitHub `Spatio-Temporal-Mapping` repository was identified as the clean simulator baseline;
- the missing LIO-SAM submodule metadata was repaired on a separate branch and verified with a recursive clone;
- Git LFS assets were downloaded and compared with the recovered desktop workspace;
- the dedicated Linux laptop discovered and displayed the remote Warthog and Jetson camera streams;
- the remaining image issue was narrowed to low delivery rate and transport/QoS performance rather than discovery;
- Dawood's repository was separated into a reusable RGB-D reconstruction pipeline and a dormant-tree semantic/skeletonization pipeline;
- first-pass ROS 2 versions of the motion planner, point-cloud node, PointCloud2 helper, package files, parameters, and launch file were created;
- a default `execute_motion: false` interlock was added;
- an absolute raster and move–settle–capture handshake were designed; and
- the migrated Python code was syntax-checked.

Not yet demonstrated:

- a successful stationary PLY from the live UR5e-mounted camera;
- runtime compatibility with the actual trajectory controller;
- a verified six-joint UR5e joint-state topic;
- a validated fixed-frame-to-camera optical TF chain and hand–eye calibration;
- a complete indoor raster scan;
- physical arm motion using the migrated node;
- a real orchard-tree dataset; or
- a mesh or Gazebo-ready model.

---

## Immediate Milestones

### Milestone 1 — Stationary frame

```text
verify RGB and aligned-depth rates
→ inspect CameraInfo and frame IDs
→ verify TF at the image timestamp
→ trigger one capture with motion disabled
→ save PLY
→ inspect scale, orientation, color, and background
```

### Milestone 2 — Two-view validation

```text
capture first view
→ move once under supervision
→ settle
→ capture second view
→ compare TF-only and TF+ICP alignment
→ inspect fitness, RMSE, and duplicate surfaces
```

### Milestone 3 — Small indoor raster

Use a deliberately small, low-speed scan with a verified start pose, verified scan-plane axes, conservative collision geometry, and one acknowledged capture per viewpoint.

### Milestone 4 — Data preservation

Record a rosbag and save the parameter, calibration, TF, camera-info, environment, and output metadata needed to reproduce the scan offline.

### Milestone 5 — Orchard collection

Only after the indoor pipeline succeeds, schedule and perform supervised capture of several leafy trees while preserving raw data even if online reconstruction is incomplete.

---

## Primary Blockers and Required Validation

- identify the active UR5e `FollowJointTrajectory` action namespace;
- confirm that the selected joint-state topic contains all six arm joints;
- verify that camera, TF, controller, and Gazebo data use a consistent time source;
- verify the complete transform from the fixed robot frame through the UR5e and camera mount to the optical frame;
- obtain or confirm the physical robot calibration, `ur5e.yml`, `collision_table.yml`, and camera-mount calibration;
- compare camera publishing rate on the Jetson with the rate received on the laptop;
- configure Best Effort sensor QoS and compressed transport where appropriate;
- validate the collision model for the actual Warthog-mounted UR5e, camera housing, floor, nearby equipment, and specimen; and
- confirm supervision, field access, power, transport, weather, and operating permissions.

---

## Safety Constraints

- Physical motion remains disabled by default.
- No inherited hardcoded home pose or full scan path should be executed without validating the current robot configuration.
- The first motion test must use a small workspace, low speed, active supervision, and a known-safe start pose.
- ICP must not be used to conceal a missing or incorrect hand–eye calibration.
- The robot must stop and settle before each requested capture.
- Raw data must be recorded so that reconstruction tuning does not require repeating unsafe or expensive hardware motion.

---

## Deferred or Out-of-Scope Work

The following work is intentionally deferred unless the project is redirected:

- broad Gazebo portability cleanup;
- general Warthog navigation-platform development;
- Nav2 benchmarking;
- learned-policy integration or evaluation;
- LIO-SAM benchmarking;
- deep comparison of the two inherited `temporal_projector.cpp` variants;
- full cleanup of historical orchard worlds and assets;
- dormant-tree trunk/branch semantic segmentation;
- DBSCAN skeletonization and branch measurements;
- photorealistic or physically deformable foliage;
- support for multiple ROS distributions; and
- validated simulation-to-real equivalence.

These components should be preserved for future work but must not displace the immediate reconstruction and data-collection milestones.

---

## Expected Repository Deliverables

- ROS 2 package containing the migrated motion and reconstruction nodes;
- parameterized launch and configuration files;
- documented camera, controller, joint-state, TF, and time assumptions;
- stationary-frame and small-scan validation procedures;
- rosbag recording and offline-replay instructions;
- PLY/PCD and per-frame metadata outputs;
- sample results kept outside normal source-control history where large;
- provenance and inherited-work records;
- known-issues and future-work documentation; and
- downstream notes for cleaning, meshing, collision simplification, and Gazebo packaging.

---

## Future Research Direction

The larger research opportunity is not merely to scan trees. It is to create a reproducible, real-tree-derived, multi-season orchard simulation environment and use it to evaluate seasonal domain shift in agricultural perception and autonomous navigation.

Potential follow-on contributions include:

- reconstruction-quality evaluation;
- simulator integration;
- seasonal asset generation;
- dormant-versus-leafy experiments;
- multi-season training or domain randomization;
- perception and navigation robustness measurements;
- figures, ablations, and methods documentation; and
- publication writing.
