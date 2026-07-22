# 2026-07-17 — Simulation Reproduction, Technical Handoff, and Project Scope Pivot

## Objective

Reproduce enough of the inherited Warthog orchard-simulation stack to determine whether it is technically viable, complete the formal handoff with Divyanth and Nidhish, identify the authoritative source repository, and obtain a concrete definition of success for the remainder of the Cornell visit.

The day began with the assumption that my primary contribution would involve reproducing and extending the Gazebo orchard environment.

Following the handoff meeting, the project scope changed substantially. My primary responsibility is now to inherit Dawood’s existing tree-reconstruction algorithm, remove unnecessary components, migrate the required workflow to ROS 2, connect it to the Warthog’s RGB-D camera, and produce several reconstructed leafy-tree models before leaving Cornell.

Gazebo integration is now a downstream objective rather than the immediate focus.

---

## Work Completed

### Reproduced the inherited ROS workspace from a clean build

Performed a clean build of the recovered:

```text
cornell_orchard_ws
```

using:

```bash
source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  2>&1 | tee build-reproduction.log
```

The workspace contains three ROS 2 packages:

```text
lio_sam
orchard_description
orchard_perception
```

The build completed successfully:

```text
Summary: 3 packages finished
```

This established that the recovered workspace is fundamentally buildable under:

* Ubuntu 22.04;
* ROS 2 Humble;
* Ignition Gazebo Fortress;
* Python 3.10;
* the Clearpath Humble simulation stack.

The remaining warnings were primarily package-maintenance and dependency warnings rather than compilation failures.

---

### Reproduced the core Warthog simulation workflow

Launched the inherited orchard world through:

```bash
ros2 launch clearpath_gz simulation.launch.py
```

using the recovered Clearpath Warthog configuration and the orchard world under `orchard_description`.

The launch progressed far enough to confirm that:

* Gazebo Fortress loaded the requested orchard world;
* the Warthog entity was created successfully;
* the generated Warthog URDF was published;
* `robot_state_publisher` loaded the robot frame tree;
* `gz_ros2_control` initialized the four wheel joints;
* the controller manager started;
* the LiDAR bridge started;
* three RGB-D camera bridges started;
* RGB-D point-cloud bridges started;
* IMU bridges started;
* teleoperation and command-multiplexing nodes started.

This demonstrated that the inherited simulator is not fundamentally broken.

The primary remaining failures were:

1. hard-coded absolute file paths;
2. Clearpath generator permissions under `/etc/clearpath`;
3. machine-specific launch configuration;
4. unresolved differences between the GitHub repository and the recovered desktop workspace.

---

## Simulation Defects Identified

### Clearpath setup-directory permissions

The simulator attempted to generate files under:

```text
/etc/clearpath/
```

including:

```text
robot.urdf.xacro
robot.srdf.xacro
generated sensor launch files
generated camera parameter files
```

These writes failed under the normal user account because `/etc/clearpath` is root-owned.

The simulator continued only because previously generated files were already present.

The portable solution is to copy the simulation configuration to a user-writable directory such as:

```text
~/clearpath_sim/
```

and pass that directory through the `setup_path` launch argument.

ROS and Gazebo should not be launched with `sudo`.

---

### Absolute paths tied to previous users

The world, model, and launch files contain paths associated with multiple machines and accounts, including:

```text
/home/divyanth/...
/home/nidhish/...
/home/nidhish27/...
```

This suggests that the project was moved between Divyanth’s laptop and Nidhish’s workstation without being fully made portable.

Replacing these paths with `/home/david/...` would only create another machine-specific version.

The correct long-term solution is to use:

* `model://` URIs for Gazebo assets;
* `IGN_GAZEBO_RESOURCE_PATH` for model discovery;
* `ament_index_python` and `get_package_share_directory()` for ROS package files;
* launch arguments for Clearpath setup directories;
* installed package paths instead of source-tree home-directory paths.

Because the project scope changed during the meeting, this portability cleanup is no longer the immediate research priority. The findings should still be preserved for later simulator integration.

---

## Formal Handoff Meeting

Met with:

* Divyanth Loganathan Girija;
* Nidhish Kumar.

This was the first full technical handoff with Nidhish.

I explained that, after receiving administrator access to the lab desktop, I located his ROS workspaces while searching for the existing ROS and Gazebo installation. I copied the workspaces into my own directory for inspection and left the original directories untouched.

Nidhish accepted this explanation and clarified the organization of his work.

---

## Workspace Authority Clarified

Nidhish identified:

```text
cornell_orchard_ws
```

as the primary workspace relevant to the simulator.

Its three packages are:

### `lio_sam`

Used to combine LiDAR and IMU measurements for pose estimation and map creation.

The primary project-specific configuration is contained in:

```text
src/LIO-SAM/config/params.yaml
```

Most of the remaining package is inherited upstream LIO-SAM code.

### `orchard_description`

Contains:

* orchard worlds;
* tree models;
* ground meshes;
* visual and collision assets;
* scripts used to generate orchard layouts;
* multiple dormant and high-resolution world variants.

Nidhish identified the dormant-orchard and new-tree generation scripts as the most relevant environment-generation code.

### `orchard_perception`

Contains:

* camera–LiDAR projection and fusion;
* the principal launch orchestration;
* RViz configuration;
* the temporal-projector implementation.

Nidhish indicated that the main launcher coordinates the simulator, mapping, perception, and visualization components.

---

## Workspace Deprioritized

The second recovered workspace:

```text
orchard_navigation_rl_ws
```

contains the prior behavior-cloning and learned-navigation work, including:

* camera-only navigation;
* LiDAR-only navigation;
* multimodal navigation;
* model checkpoints;
* evaluation scripts;
* data collection;
* DAgger infrastructure;
* odometry diagnostics;
* offline policy metrics.

Nidhish characterized this as separate and earlier work.

It is not currently the authoritative workspace for my assignment and should not receive additional attention during the remaining Cornell visit unless Divyanth explicitly redirects the project again.

Therefore, the following tasks are now paused:

* learned-policy metric analysis;
* camera/LiDAR/multimodal ablations;
* closed-loop policy integration;
* Nav2 benchmarking;
* navigation-policy deployment;
* broad Gazebo evaluation infrastructure.

---

## Project Scope Changed

The most important result of the meeting was a substantial narrowing of the assignment.

My primary responsibility is no longer:

> Complete a general-purpose Gazebo navigation platform or benchmark autonomous navigation methods.

The revised assignment is:

> Build a reproducible 3D tree-reconstruction pipeline, use it to reconstruct several real leafy orchard trees while physically at Cornell, and export usable 3D assets that can later be inserted into Nidhish’s Gazebo orchard.

The immediate workflow is now:

```text
Dawood’s existing reconstruction algorithm
                ↓
Understand and reproduce the current pipeline
                ↓
Remove obsolete or unnecessary components
                ↓
Migrate the required workflow from ROS 1 to ROS 2
                ↓
Connect it to the Warthog RGB-D camera
                ↓
Validate it on an indoor artificial tree
                ↓
Collect real orchard-tree data
                ↓
Reconstruct several leafy tree models
                ↓
Export simulator-ready assets
                ↓
Future Gazebo integration and evaluation
```

Divyanth indicated that, with approximately ten working days remaining, a successful internship outcome would be:

* a functioning reconstruction codebase;
* successful integration with the Warthog’s RGB-D data;
* several reconstructed tree models;
* reusable documentation and code;
* preservation of the raw data required for later processing.

Insertion of every model into Gazebo is secondary and may be completed remotely after I return to UCF.

---

## Research Motivation

The current virtual orchard primarily represents dormant or winter trees.

The laboratory wants to evaluate agricultural perception or navigation systems under seasonal visual changes. Models trained or tested only in a dormant orchard may not generalize to:

* leafy canopies;
* green-fruit stages;
* denser foliage;
* different lighting and occlusion patterns;
* other seasonal appearances.

The immediate tree scans collected in July should be labeled by their actual phenological condition rather than automatically being called “spring” or “fall” models.

A realistic label may be:

```text
current-season leafy trees
```

or:

```text
green-fruit-stage trees
```

depending on the orchard condition at collection time.

---

## Publication Framing

Scanning trees and exporting meshes alone would probably be too small for a strong publication.

The broader publishable direction is:

> A multi-season, real-tree-derived orchard simulation environment for evaluating the seasonal robustness of agricultural perception and autonomous-navigation systems.

A complete future paper could include:

* reconstructed tree assets representing multiple seasonal stages;
* a reproducible data-capture and reconstruction methodology;
* reconstruction-quality measurements;
* integration of the assets into a simulation environment;
* evaluation of one or more algorithms across seasonal conditions;
* measurement of performance degradation caused by seasonal domain shift;
* experiments involving multi-season training or domain randomization.

My immediate work would provide the data and reconstruction infrastructure required for that larger experiment.

To preserve publication potential, I should continue owning more than the initial data collection. Potential continued responsibilities include:

* ROS 2 pipeline development;
* reconstruction evaluation;
* simulator integration;
* seasonal experiments;
* figures and visualizations;
* documentation;
* ablation studies;
* writing the methods and experimental sections.

---

## Revised Definition of Done

### Minimum deliverable

By the end of the Cornell visit:

* Dawood’s reconstruction workflow is reproduced;
* the required parts are migrated to ROS 2;
* unnecessary or obsolete code is removed or isolated;
* the node can consume the Warthog RGB-D stream;
* at least one indoor tree reconstruction succeeds;
* raw data from several real orchard trees is collected;
* at least two usable tree models are generated;
* the pipeline and required commands are documented.

### Target deliverable

* Three to six real trees are captured successfully;
* several reconstructed models are produced;
* visual and collision representations are considered separately;
* model scale and orientation are validated;
* raw inputs, camera information, parameters, and metadata are preserved;
* the complete pipeline can be repeated by another researcher;
* at least one model is prepared for later Gazebo insertion.

### Stretch deliverable

* Six to ten trees are captured;
* multiple tree models are integrated into the orchard;
* automated model/world generation is added;
* a dormant-versus-leafy comparison is performed;
* a preliminary seasonal-robustness experiment is completed.

---

## Repository Handoff and Audit

Nidhish shared a GitHub repository named:

```text
Spatio-Temporal-Mapping
```

The GitHub repository is now treated as the clean baseline, while the recovered desktop workspace is retained as a recovery source for possible uncommitted changes.

---

### Restored missing LIO-SAM submodule metadata

The repository tracked:

```text
src/LIO-SAM
```

as a Git submodule pointer at commit:

```text
08af3f32f01725372d4269838dc44c19c6d9e76b
```

However, the repository was missing the corresponding `.gitmodules` entry. A fresh clone therefore knew which commit was expected but did not know which repository contained it.

The recovered desktop copy confirmed the intended upstream repository:

```text
https://github.com/TixiaoShan/LIO-SAM.git
```

and was checked out at the exact expected commit.

Created a repair branch:

```text
fix/restore-lio-sam-submodule
```

Added:

```ini
[submodule "src/LIO-SAM"]
    path = src/LIO-SAM
    url = https://github.com/TixiaoShan/LIO-SAM.git
```

Verified the repair by performing a fresh recursive clone and confirming that LIO-SAM was automatically checked out at the intended commit.

The repair branch was pushed to Nidhish’s repository but was not merged into the default branch.

I should notify Nidhish, explain the repair, and ask whether he wants:

* a pull request;
* the branch retained without merging;
* or the branch removed.

---

### Downloaded Git LFS assets

Many tree meshes initially appeared as files of approximately 130 bytes.

These were Git LFS pointer files rather than actual geometry.

Installed Git LFS:

```bash
sudo apt install git-lfs
git lfs install
git lfs pull
```

After pulling the LFS objects, the files became full binary assets measured in megabytes.

This allowed the GitHub repository to be compared meaningfully against the recovered desktop workspace.

---

### Compared GitHub repository with recovered desktop copy

After initializing the submodule and downloading LFS assets, the two workspaces were largely similar.

The recovered desktop contained additional result artifacts:

```text
final_colored_orchard.pcd
frames_*.gv
frames_*.pdf
build-reproduction.log
```

These should be preserved as experimental outputs rather than merged directly into source control.

Important code differences remain in:

```text
LIO-SAM/config/params.yaml
LIO-SAM/config/rviz2.rviz
LIO-SAM/launch/run.launch.py

orchard_description/scripts/generate_orchard_new_trees.py
orchard_description/worlds/orchard_final_high_res.sdf

orchard_perception/launch/infrastructure.launch.py
orchard_perception/launch/main_launcher.py
orchard_perception/rviz/orchard_mapping.rviz
orchard_perception/src/temporal_projector.cpp
```

The two `temporal_projector.cpp` files appear to represent substantially different experimental implementations rather than simple path updates.

Because the project has pivoted toward reconstruction, these differences should be documented and preserved but not investigated deeply unless required later.

---

## RGB-D Camera and ROS 2 Integration Setup

Began preparing to connect Dawood’s reconstruction algorithm to the Warthog’s RGB-D camera.

The likely runtime architecture is:

```text
Warthog RGB-D camera
         ↓
Jetson Orin onboard computer
         ↓
ROS 2 camera driver publishes image and depth topics
         ↓
Laptop joins the same ROS 2 network
         ↓
RViz and reconstruction node subscribe to camera topics
         ↓
Reconstruction pipeline generates tree point cloud or mesh
```

The camera driver should run on the Jetson because the camera is physically connected to it.

The laptop should primarily be used for:

* visualization;
* development;
* reconstruction;
* rosbag recording;
* debugging;
* offline processing.

---

## WSL and ROS Environment Setup

My laptop initially used:

* a WSL Ubuntu environment;
* ROS 2 Jazzy.

The Jetson uses:

* Ubuntu 22.04;
* ROS 2 Humble.

Because cross-distribution communication is not a reliable basis for field data collection, installed a separate:

```text
Ubuntu-22.04
```

WSL distribution and configured the official ROS 2 repository.

Installed:

```text
ros-humble-desktop
ros-dev-tools
```

The intended environment is now:

```text
Laptop: Ubuntu 22.04 + ROS 2 Humble
Jetson: Ubuntu 22.04 + ROS 2 Humble
```

Configured WSL mirrored networking through:

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

Set the provisional ROS network configuration:

```bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

---

## Connected to the Warthog ROS Network

After restarting WSL and correcting the ROS environment, remote ROS topics became visible using:

```bash
ros2 topic list --no-daemon
```

The discovered topics confirmed access to:

* Warthog platform status;
* motor status;
* battery information;
* emergency-stop status;
* odometry;
* filtered odometry;
* command velocity;
* joint states;
* robot description;
* TF;
* IMU;
* front RGB-D camera;
* Jetson RGB-D camera.

Relevant Jetson camera topics include:

```text
/sensors/camera_jetson/color/image_raw
/sensors/camera_jetson/color/camera_info

/sensors/camera_jetson/depth/image_rect_raw
/sensors/camera_jetson/depth/camera_info

/sensors/camera_jetson/aligned_depth_to_color/image_raw
/sensors/camera_jetson/aligned_depth_to_color/camera_info
```

Other available camera data include:

```text
/sensors/camera_0/color/image
/sensors/camera_0/depth/image
/sensors/camera_0/points
```

The full topic discovery proves that:

* WSL mirrored networking is functioning;
* the laptop is on the robot network;
* the ROS domain is correct;
* remote ROS 2 participant discovery works;
* the Jetson and Warthog publishers are visible.

---

## Current ROS Networking Issue

The normal ROS daemon remained stale or did not reflect the remote graph.

This command showed only local topics:

```bash
ros2 topic list
```

while this command showed the full remote graph:

```bash
ros2 topic list --no-daemon
```

Therefore, remote discovery is working, but the local ROS 2 daemon must be reset after the correct ROS domain and networking variables are loaded.

Planned reset:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0

ros2 daemon stop
pkill -f _ros2_daemon 2>/dev/null || true
ros2 daemon start
```

All environment variables should be set before starting the daemon.

---

## Current Camera Visualization Issue

Although the camera topics are visible, the image stream has not yet been displayed successfully in RViz.

Possible causes include:

* stale ROS daemon state;
* incompatible RViz QoS settings;
* best-effort camera publishing versus reliable subscription;
* camera topic visible through discovery but image packets not yet received;
* incorrect RViz image topic;
* use of the wrong camera stream;
* Wi-Fi limitations for large image or point-cloud messages.

The next validation command should measure whether actual messages are arriving:

```bash
ros2 topic hz \
  /sensors/camera_jetson/color/image_raw \
  --qos-profile sensor_data
```

Then test one message:

```bash
ros2 topic echo \
  /sensors/camera_jetson/color/image_raw \
  --qos-profile sensor_data \
  --once
```

In RViz, the Image display should use:

```text
Topic: /sensors/camera_jetson/color/image_raw
Reliability: Best Effort
Durability: Volatile
```

`rqt_image_view` can be used as a simpler camera-feed validation tool before debugging RViz further.

---

## Revised Priorities

### Highest priority

1. Obtain Dawood’s complete reconstruction code and documentation.
2. Identify the exact input topics and expected message types.
3. Determine what parts of the ROS 1 system are still necessary.
4. Remove unrelated or obsolete code.
5. Create a clean ROS 2 package.
6. Confirm live RGB and aligned-depth data from the Warthog.
7. Record a short rosbag containing all required inputs.
8. Run one indoor artificial-tree reconstruction.
9. Measure the time and quality of one complete reconstruction.
10. Schedule real orchard data collection.
11. Capture several trees while physically at Cornell.
12. Preserve all raw data and metadata.
13. Generate several usable tree models.
14. Document the complete workflow.

### Deprioritized

* broad Gazebo cleanup;
* Nav2 study;
* SLAM benchmarking;
* learned-navigation policy evaluation;
* simulation metrics;
* full seasonal orchard generation;
* optimization of Nidhish’s temporal projector;
* cleaning every historical world and model asset;
* general-purpose navigation evaluation.

---

## Questions for Dawood

1. Which repository contains the latest reconstruction algorithm?
2. Which branch or commit should be treated as authoritative?
3. Is the current implementation ROS 1, ROS 2, or partially independent of ROS?
4. What files and nodes are no longer required?
5. What exact RGB-D camera model was used?
6. Which topics does the algorithm subscribe to?
7. Does it consume:

   * RGB images;
   * aligned depth;
   * camera intrinsics;
   * point clouds;
   * TF;
   * odometry;
   * camera poses?
8. Does reconstruction run online or after data collection?
9. What output format does it produce?
10. How is the final mesh generated?
11. What coordinate frame, scale, and orientation are expected?
12. How was the camera moved around the tree?
13. How are multiple views registered?
14. Is external pose estimation required?
15. What calibration files are required?
16. What was the most recent successful test?
17. Which parameters are most sensitive?
18. What failure cases should I expect?
19. How long does one tree take to capture and process?
20. What minimum tree coverage is needed?
21. Should processing run on the Jetson, laptop, or lab desktop?
22. Which parts should I rewrite rather than directly port?

---

## Questions for Divyanth

1. What exact demonstration should I provide before leaving Cornell?
2. How many reconstructed trees are required for the minimum deliverable?
3. What actual seasonal label should be assigned to the July orchard trees?
4. Which orchard and tree variety should be scanned?
5. Who will help operate the Warthog during field collection?
6. Which robot platform and camera configuration should be used?
7. Should I collect six to ten trees only after validating one complete indoor reconstruction?
8. What metadata should be collected for every tree?
9. What future experiment will use the reconstructed assets?
10. Can I continue owning simulator integration and seasonal evaluation after returning to UCF?
11. What contribution would support authorship on a future paper?
12. Should the reconstruction work live in my repository, Dawood’s repository, or a lab-owned repository?

---

## Decisions

* The primary project is now tree reconstruction, not general Gazebo development.
* Dawood’s reconstruction algorithm should be inherited rather than rebuilt from scratch.
* Only the necessary components should be migrated to ROS 2.
* Data collection should be prioritized while I am physically at Cornell.
* A working indoor reconstruction should precede large-scale orchard collection.
* Raw RGB-D data should be recorded even if live reconstruction is available.
* The Jetson should publish hardware camera data.
* The laptop or lab desktop should perform heavier reconstruction unless the existing system requires Jetson execution.
* The GitHub `Spatio-Temporal-Mapping` repository is the clean simulator baseline.
* The recovered desktop workspace should remain an unmodified recovery source.
* Gazebo portability findings should be documented but not allowed to distract from the reconstruction deliverable.
* Publication potential depends on continuing from asset creation into evaluation, experiments, figures, and writing.

---

## Next Actions

1. Reset the ROS 2 daemon with the correct domain already exported.
2. Confirm a live frequency for the Jetson RGB topic.
3. Confirm aligned-depth frequency.
4. Display the image through `rqt_image_view` or RViz with best-effort QoS.
5. Inspect camera-info messages and frame IDs.
6. Record a short RGB-D rosbag.
7. Obtain Dawood’s code and latest run instructions.
8. Audit the code and separate:

   * required reconstruction logic;
   * ROS 1 interfaces;
   * obsolete scripts;
   * visualization-only code;
   * training or experimental remnants.
9. Design the ROS 2 package structure.
10. Run the pipeline on an indoor artificial tree.
11. Estimate capture and processing time per tree.
12. Set a realistic field-collection target.
13. Schedule the orchard trip and robot access.
14. Define the raw-data and model naming convention.
15. Prepare a Monday progress update with:

* live camera proof;
* reconstruction architecture;
* code-migration plan;
* blockers;
* field-data schedule.

---

## End-of-Day Status

The inherited simulator was reproduced far enough to prove that its core architecture is viable. All three primary packages built successfully, Gazebo loaded the orchard world, the Warthog spawned, its controllers initialized, and its simulated sensors were bridged into ROS 2.

The formal meeting with Divyanth and Nidhish then clarified that broad simulator development is not the best use of the remaining time at Cornell.

The project has been narrowed to a more realistic and physically dependent contribution:

> Inherit and clean Dawood’s tree-reconstruction algorithm, migrate the required workflow to ROS 2, connect it to the Warthog RGB-D camera, collect real leafy-tree data, and produce several reusable 3D tree models.

The simulator work completed earlier in the day remains valuable because it established the longer-term destination for the reconstructed assets and documented the inherited infrastructure. However, the immediate research priority is now the reconstruction pipeline and field-data collection.

The laptop is now running Ubuntu 22.04 with ROS 2 Humble, can discover the Warthog and Jetson ROS graph, and can see the required RGB-D topic names. The remaining immediate technical issue is validating actual image delivery and displaying the stream reliably in RViz or `rqt_image_view`.

The day ended with a substantially clearer and more achievable research objective than the one held at the start of the week.

