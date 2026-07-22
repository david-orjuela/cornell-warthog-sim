# 2026-07-20 — RGB-D Transport Debugging and Dawood Reconstruction Pipeline Review

## Objective

Continue directly from Friday’s successful ROS 2 Humble networking setup and remote topic discovery, validate that the Warthog and Jetson RGB-D streams could be viewed from the laptop, and begin understanding Dawood’s UR5e tree-scanning and point-cloud reconstruction code well enough to adapt it for a new use case.

The longer-term objective is to reuse the relevant portions of Dawood’s algorithm to:

1. scan spring and fall trees with the UR5e-mounted RGB-D camera;
2. reconstruct full tree geometry without relying on dormant-tree skeletonization;
3. save the reconstruction as a persistent 3D asset;
4. convert it into a mesh or another Gazebo-compatible model format; and
5. import the model into the Warthog simulation so it can operate in environments beyond the existing dormant-tree scene.

The work therefore had two connected tracks:

* **Track A:** validate RGB-D visualization and transport over the current ROS 2 network;
* **Track B:** reverse-engineer Dawood’s ROS 1 motion, reconstruction, registration, semantic segmentation, and skeletonization pipeline.

---

## Executive Summary

### RGB-D transport

The laptop successfully displayed images from both the Warthog camera and the Jetson-connected RGB-D camera in RViz2. This confirmed that ROS 2 discovery, cross-machine communication, image message transport, and basic decoding were working.

The remaining visualization problem was performance rather than discovery. RViz received only about:

```text
4 messages at approximately 0.2 Hz
```

or roughly one frame every five seconds.

The likely bottlenecks were narrowed to:

* raw image bandwidth over Wi-Fi;
* Reliable QoS retransmitting old packets;
* multiple simultaneous image, depth, and point-cloud streams;
* a low publisher rate on the Jetson;
* missing compressed image transport support in the local ROS 2 Humble environment;
* or a WSL (earlier)/network transport limitation.

### Dawood reconstruction code

Dawood’s repository contains two conceptually separate pipelines:

```text
Pipeline A — Relevant to this project

UR5e motion
→ synchronized RGB-D capture
→ camera-to-base TF
→ RGB-D point-cloud generation
→ coarse-to-fine point-to-plane ICP
→ filtering and global fusion
→ reconstructed point cloud
```

```text
Pipeline B — Mostly not needed initially

semantic trunk/branch labels
→ DBSCAN component extraction
→ spline skeleton fitting
→ parent/child branch relationships
→ branch length and diameter estimation
```

The reusable core is the acquisition, TF, RGB-D reconstruction, ICP, filtering, and rosbag workflow. The semantic reconstruction and skeletonization layers are specialized for dormant woody trees and would likely remove most leaves from a spring or fall tree.

The current code does **not** save a final surface model, generate an STL, or create a Gazebo model. Those steps still need to be added.

---

# Track A — RGB-D Camera Visualization and Transport Debugging

## Switched to a Dedicated Linux Laptop

Today, I received a Linux laptop running Pop!_OS 22.04 for the project. This replaced the WSL environment on my personal laptop, which had been slow and only had ROS 2 Jazzy installed rather than ROS 2 Humble.

The ROS distribution mismatch and WSL performance limitations likely contributed to some of the networking, visualization, and package-compatibility errors encountered on Friday. Using a native Ubuntu 22.04-based system with ROS 2 Humble provides a more appropriate and consistent environment for the Warthog and Jetson setup.

## Displayed Remote Camera Streams in RViz2

Launched RViz2 from the Ubuntu 22.04 / ROS 2 Humble environment and added Image displays for:

```text
/sensors/camera_0/color/image
```

and:

```text
/sensors/camera_jetson/color/image_raw
```

Both topics produced visible images.

This confirmed that:

* the laptop could discover the remote camera publishers;
* ROS image messages could cross from the Warthog and Jetson network into my environment;
* the camera topics used compatible ROS message types;
* RViz could subscribe to and decode at least some frames;
* the issue was no longer a complete ROS networking or discovery failure.

---

## Identified a Very Low Received Image Rate

Although images appeared, they did not update reliably in real time.

RViz reported approximately:

```text
4 messages received at 0.2 Hz
```

This corresponds to approximately one frame every five seconds.

The displayed image could therefore remain several seconds behind the physical camera view. Standing in front of the camera did not consistently appear immediately in RViz.

RViz reporting:

```text
Status: OK
```

only established that valid messages were being received and decoded. It did not mean the stream was operating at a usable live rate.

---

## Inspected RViz Image QoS Settings

The Image displays were initially configured with:

```text
Reliability Policy: Reliable
History Policy: Keep Last
Depth: 5
Durability Policy: Volatile
```

For high-bandwidth sensor streams over Wi-Fi, Reliable delivery may repeatedly retransmit missing data and deliver stale frames.

The preferred live-visualization configuration is:

```text
Reliability Policy: Best Effort
History Policy: Keep Last
Depth: 1
Durability Policy: Volatile
```

This prioritizes the newest available frame and limits stale-frame buildup.

---

## Attempted to Use Compressed Image Transport

Attempted to switch RViz from raw image transport to compressed image transport.

RViz returned:

```text
Error subscribing: Unable to load plugin for transport
'image_transport/compressed_sub'
```

The plugin loader reported that only:

```text
image_transport/raw_sub
```

was available.

This established that the local ROS 2 Humble installation did not have the compressed image subscriber plugin installed.

Expected packages include:

```bash
ros-humble-image-transport-plugins
```

and possibly:

```bash
ros-humble-compressed-image-transport
```

---

## RGB-D Transport Findings

ROS 2 networking is functioning. The remaining slowdown is likely related to transport, QoS, publishing rate, or bandwidth.

Potential causes include:

* raw RGB traffic overwhelming Wi-Fi;
* Reliable QoS retransmissions;
* simultaneous RGB, depth, aligned depth, and point-cloud traffic;
* a low publishing rate at the source;
* receiving fewer frames than the Jetson publishes locally;
* or competing traffic from the Warthog and Jetson.

Compressed topics may exist remotely, but RViz cannot use them locally until the appropriate transport plugin is installed.

---

# Track B — Dawood Code Review

## Intended Adaptation

Dawood’s code was reviewed to determine which components can be reused to scan leafy seasonal trees.

The goal is **not** to reproduce his full dormant-tree analysis. The immediate goal is to reuse:

* UR5e camera motion;
* synchronized RGB-D acquisition;
* camera pose retrieval;
* per-frame point-cloud generation;
* frame-to-map registration;
* point-cloud fusion;
* and offline replay.

The desired output is a dense tree reconstruction suitable for later mesh generation and Gazebo import.

---

## Repository-Level Pipeline

The reviewed files form the following pipeline:

```text
motion_planner.py
    UR5e movement commands
            ↓
camera moves around the tree
            ↓
RGB image + registered depth + CameraInfo + TF
            ↓
pointcloud_processing.py
    segmentation
    → RGB-D point cloud
    → base-frame transform
    → multiscale ICP
    → filtering
    → accumulated point cloud
            ↓
/output_point_cloud
            ↓
skeletonization.py
    semantic class separation
    → DBSCAN
    → fitted tree components
            ↓
tree_component.py
    spline centerlines
    → parent/child relationships
    → lengths and diameters
            ↓
/final_pointcloud and /skeleton_visualizer
```

The repository supports both live and offline workflows.

---

# `motion_planner.py`

## Purpose

Controls the UR5e using:

* the ROS controller manager;
* a `FollowJointTrajectory` action server;
* cuRobo motion planning;
* a UR5e robot configuration;
* a collision-world configuration;
* and hardcoded relative tool motions.

The selected controller is:

```text
scaled_pos_joint_traj_controller
```

which is appropriate for a physical UR robot because it respects UR speed scaling.

---

## Main Motion Flow

The motion execution path is:

```text
/joint_states
→ current joint positions
→ cuRobo forward kinematics
→ current end-effector pose
→ relative pose multiplication
→ cuRobo motion plan
→ ROS JointTrajectory
→ UR5e trajectory action server
```

The code uses:

```text
ur5e.yml
collision_table.yml
```

as cuRobo configuration files.

These are still required to fully reproduce the motion setup.

---

## Important Motion-Planning Findings

### The current main block runs pruning, not scanning

The active code calls:

```python
trajectory_controller.pruning_mode()
```

while:

```python
trajectory_controller.scanning_mode()
```

is commented out.

The file must not be run expecting the scanning pattern without changing this.

### Joint ordering is handled unsafely

The callback records both joint names and positions, but the planner later performs:

```python
joint_positions[0], joint_positions[2] = (
    joint_positions[2],
    joint_positions[0],
)
```

This assumes a fixed incoming order. A safer implementation should map positions by joint name.

### CUDA is hardcoded

The code explicitly uses:

```text
cuda:0
```

and therefore expects a CUDA-capable GPU.

### The action client does not explicitly wait for its server

The controller-manager service is awaited, but the trajectory action client should also call:

```python
wait_for_server()
```

before sending the first goal.

### Relative-pose conventions must be verified

The target pose is constructed by multiplying the current end-effector pose by a relative pose. The relative translation is therefore likely expressed in the tool/end-effector frame rather than directly in `base_link`.

This must be confirmed before designing a new scan pattern.

### Quaternion handling needs cleanup

The code uses rotation values such as:

```text
[1, -0.2, 0, 0]
```

which are not normalized unit quaternions.

cuRobo appears to use a quaternion order consistent with:

```text
[w, x, y, z]
```

because `[1, 0, 0, 0]` is used as identity.

ROS static TF uses:

```text
[x, y, z, w]
```

These conventions must not be mixed.

### `go_home()` is not planned through cuRobo

The home motion is a hardcoded joint trajectory:

```text
[-0.06, -1.25, 2.3, -3.76, -1.57, -3.14]
```

executed over two seconds.

It may not receive the same collision checking as the cuRobo-planned motions. It must be validated for the current robot, camera mount, table, cables, and specimen placement before execution.

### The scan path is a hardcoded relative raster

`scanning_mode()` uses a long series of relative motions that approximately form horizontal sweeps separated by vertical moves.

This resembles a raster scan, but it is difficult to:

* verify absolute workspace limits;
* restart midway;
* adapt to another tree size;
* reason about final camera positions;
* or guarantee repeatability after a failed move.

A better adaptation would generate named absolute viewpoints from parameters such as:

```text
scan width
scan height
rows
columns
camera standoff
camera orientation
```

---

# `pointcloud_processing.py`

## Purpose

This file performs most of the reconstruction work.

It:

* synchronizes RGB, depth, and camera-information messages;
* runs semantic segmentation;
* retrieves the camera pose from TF;
* creates Open3D RGB-D point clouds;
* transforms them into `base_link`;
* refines alignment with multiscale ICP;
* filters and fuses accepted frames;
* publishes the accumulated cloud;
* records reconstruction metrics;
* and invokes skeletonization at shutdown.

---

## Subscribed Topics

The ROS 1 reconstruction node expects:

```text
/rgb/image_raw
/depth_to_rgb/hw_registered/image_rect_raw
/rgb/camera_info
```

The depth image is already registered to the RGB image, allowing a color or segmentation pixel to correspond to the same physical ray as its depth value.

---

## Synchronization

The code uses:

```python
ApproximateTimeSynchronizer(..., queue_size=10, slop=0.001)
```

The allowed timestamp difference is only:

```text
1 millisecond
```

This may be unnecessarily strict and could reject valid RGB-D pairs. Actual message timestamp differences should be measured before preserving this setting.

---

## Segmentation

The active model is YOLO:

```python
self.model = YOLO(rospy.get_param("/model_path"))
```

The alternative BiSeNet wrapper is present but commented out.

The YOLO masks are painted into three exact semantic colors:

```text
trunk:            [0, 0, 1] in Open3D
primary branch:   [0, 1, 0]
secondary branch: [1, 0, 0]
```

Unsegmented pixels remain black.

The reconstruction later removes black points, which means it discards any geometry not classified as trunk or branch.

This is likely unsuitable for spring and fall trees because a dormant-tree model may classify leaves as background and delete most canopy geometry.

The first adapted reconstruction should therefore bypass class-specific segmentation.

---

## Hardcoded Image Resolution

The segmentation image is allocated as:

```text
1280 × 720
```

This should instead use the incoming image shape.

---

## TF-Based Camera Placement

At each camera timestamp, the code requests:

```text
base_link ← rgb_camera_link
```

The transform is converted into a 4×4 homogeneous matrix and applied to the new point cloud.

Conceptually:

```text
point in base frame
=
T(base_link ← rgb_camera_link)
× point in camera frame
```

This provides the initial frame placement before ICP.

Accurate robot kinematics, camera extrinsics, and timestamp synchronization are therefore central to reconstruction quality.

---

## Depth Handling

The depth image is decoded as:

```text
16UC1
```

and all values greater than:

```text
1000 mm
```

are set to zero.

The code comment incorrectly says this represents two meters. It is actually one meter.

The cutoff should be a ROS parameter rather than a hardcoded value.

---

## RGB-D Back-Projection

Open3D combines the semantic color image and registered depth image into an RGB-D image, then uses the RGB camera intrinsics:

```text
fx
fy
cx
cy
```

to back-project pixels into 3D.

The resulting cloud is initially in the RGB camera frame, then transformed into `base_link`.

The code should explicitly specify Open3D’s depth scale and truncation instead of relying on defaults.

---

## ICP Registration

Each new frame is registered against the accumulated global cloud.

The process begins with the TF-aligned cloud and identity ICP correction:

```text
new camera frame
→ transform using TF
→ ICP correction against global cloud
```

The registration is recursive and coarse-to-fine:

```text
SN3 → SN2 → SN1 → SN0
```

Every scale must pass its fitness and RMSE acceptance criteria. If any scale fails, the entire frame is rejected.

The method is point-to-plane ICP, which requires estimated normals.

---

## Filtering and Fusion

For accepted frames, the code:

1. applies the ICP correction;
2. voxel-downsamples the frame;
3. performs radius outlier removal;
4. adds it to the accumulated global cloud;
5. applies semantic class-based downsampling;
6. removes black points for publication;
7. publishes `/output_point_cloud`.

The first frame is downsampled to 10 mm, while later frames are downsampled to 8 mm.

Radius outlier removal uses:

```text
minimum neighbors: 8
radius: 14 mm
```

These values may remove small leaf and twig structures and will need evaluation on leafy trees.

---

## Capture Coordination Problem

The motion node has a `capture_alert` publisher, and the reconstruction node subscribes to it.

However:

* most `capture_alert` publications are commented out;
* `capture_data` starts as `True`;
* and the reconstruction node sets it back to `True` after each processed frame.

The nodes therefore do not currently implement a reliable:

```text
move
→ settle
→ capture exactly one synchronized RGB-D frame
→ confirm completion
→ move again
```

workflow.

The robot could be moving while frames are fused, increasing the risk of:

* motion blur;
* depth distortion;
* stale or mismatched TF;
* low ICP fitness;
* or registration drift.

A capture-request/capture-complete handshake should be added.

---

## No Persistent Point-Cloud Export

The node publishes the reconstruction but does not save the fused point cloud.

At minimum, the adapted node should write:

```text
PLY
PCD
```

files.

PLY is a good master output because it can preserve geometry and color.

The existing `save_frame_data()` function is currently commented out and saves only:

* color image;
* camera intrinsics;
* TF placement;
* ICP correction.

It should also save the depth image or preserve the source rosbag.

---

## Offline Shutdown Behavior

The node starts a wall-clock watchdog that shuts down after eight seconds without a message.

This is useful for rosbag playback but unsafe for live experiments because a normal pause between robot views could terminate the node.

At shutdown, it:

* logs frame and ICP metrics;
* writes JSON and CSV files;
* and runs final skeletonization.

Skeletonization should be disabled in the initial leafy-tree adaptation.

---

# Launch Files

## `system_bringup.launch`

Starts the physical system:

```text
UR5e driver
Azure Kinect driver
camera mounting transform
```

The physical UR5e uses a robot-specific calibration file:

```text
ur_calibration/etc/my_robot_calibration.yaml
```

This is preferable to nominal UR5e kinematics.

The Azure Kinect launch supplies the RGB, registered depth, and camera-info topics.

The live camera mount transform is:

```text
tool0 → camera_base
translation: 0.0000, 0.0485, 0.1325 m
quaternion:  0.5, 0.5, 0.5, -0.5
```

Because ROS static TF uses quaternion order:

```text
qx qy qz qw
```

this should not be interpreted using cuRobo’s apparent `wxyz` ordering.

The full expected TF chain is approximately:

```text
base_link
→ robot links
→ tool0
→ camera_base
→ Azure Kinect camera frames
→ rgb_camera_link
```

---

## `reconstruct_online.launch`

Loads:

```text
params.yaml
tree_yolov8n.pt
```

and starts:

```text
pointcloud_processing.py
motion_planner.py
RViz
```

The smaller YOLO model appears intended for online speed.

This launch file does not start the robot or camera drivers, so `system_bringup.launch` must already be running.

The architecture intends the motion and reconstruction nodes to run together, but the exact Python versions reviewed are not yet configured for clean live scanning because the motion main block uses `pruning_mode()` and the capture handshake is incomplete.

---

## `record_bag.launch`

Records:

### Required reconstruction topics

```text
/rgb/image_raw
/rgb/camera_info
/depth_to_rgb/hw_registered/image_rect_raw
/joint_states
/tf
/tf_static
```

### Additional diagnostic topics

```text
controller goals
controller feedback
controller results
robot mode
safety mode
speed scaling
tool data
wrench
I/O states
```

This is valuable because the physical scan can be performed once and reconstruction can be tuned repeatedly offline.

The bag output path is Dawood-specific and must be changed.

The `rosbag record` launch syntax may also contain a redundant `record` argument, and `-o` may create a timestamped prefix rather than the exact requested `.bag` filename. This should be tested before relying on it.

---

## `reconstruct_offline.launch`

Enables simulated ROS time and replays a bag with:

```text
--clock
--delay 5
--rate 1.0
```

It loads:

```text
tree_yolo11m.pt
params.yaml
skeletonization_params.yaml
```

The larger YOLO model appears intended for higher-accuracy offline processing.

The offline launch defaults to nominal UR5e kinematics, while the live launch uses the physical robot calibration. The same physical calibration should ideally be used offline if TF is reconstructed from recorded joint states.

A major unresolved discrepancy exists between the live and offline camera transforms.

Live:

```text
tool0 → camera_base
nonidentity rotation
```

Offline active transform:

```text
tool0 → azure
identity rotation
```

A commented offline line matches the live transform.

This suggests the active offline transform may be a workaround for a particular recorded bag or TF hierarchy. The correct chain must be verified with a live or replayed TF tree before trusting reconstructed dimensions.

---

# ICP Configuration in `params.yaml`

The registration stages are:

| Stage |      Resolution / correspondence scale | Maximum iterations | Maximum accepted RMSE | Minimum fitness |
| ----- | -------------------------------------: | -----------------: | --------------------: | --------------: |
| SN3   |                            50 mm voxel |                 50 |                 25 mm |            0.55 |
| SN2   |                            20 mm voxel |                 20 |                 12 mm |            0.55 |
| SN1   |                            10 mm voxel |                 10 |                  6 mm |            0.55 |
| SN0   | full cloud, 10 mm correspondence limit |                  5 |                  6 mm |            0.55 |

The coarse-to-fine sequence is:

```text
50 mm
→ 20 mm
→ 10 mm
→ full resolution
```

At each stage:

* higher fitness is better;
* lower inlier RMSE is better;
* and failure at any stage rejects the frame.

The convergence-change thresholds are:

```text
relative RMSE:    1e-7
relative fitness: 1e-7
```

These determine when ICP stops changing, not whether the frame is accepted.

The YAML also contains downsampling settings that the reviewed Python file does not use. The Python code instead hardcodes several voxel sizes. This indicates stale configuration or an incomplete refactor.

For leafy trees, the uniform minimum fitness of `0.55` may be difficult to maintain because:

* leaves move;
* depth edges are noisy;
* neighboring views see different occluded surfaces;
* and wind violates ICP’s rigid-scene assumption.

These values should be measured first rather than changed blindly.

---

# Alternative Segmentation: `bisenetv1_eca_predict.py`

This is an alternative semantic segmentation wrapper.

It:

* loads a BiSeNetV1 + Efficient Channel Attention model;
* normalizes an RGB image;
* resizes dimensions to multiples of 32;
* predicts a per-pixel class;
* returns either the integer mask or a colored overlay.

The active reconstruction code uses YOLO instead.

BiSeNet performs semantic segmentation directly, while YOLO produces instance masks that are later reduced to class colors.

For this project, neither model should be required for the initial geometry-only scan. A future model might instead classify:

```text
tree or vegetation
versus
background
```

for leafy canopies.

---

# Skeletonization Pipeline

## `skeletonization.py`

This file converts the semantically colored fused cloud into:

```text
trunk
primary branches
secondary branches
```

It uses:

* exact semantic colors;
* voxel downsampling;
* DBSCAN clustering;
* distance-to-parent filtering;
* minimum-length filtering;
* branch-height filtering;
* spline fitting;
* diameter estimation;
* and ROS marker visualization.

The code can operate either as a standalone subscriber to `/output_point_cloud` or be invoked directly at reconstruction shutdown.

The launch files reviewed do not start the standalone skeletonization node, so the shutdown path appears to be the active path.

---

## `skeletonization_params.yaml`

### Trunk

```text
color: [0, 0, 1]
voxel size: 8 mm
minimum height: 0.5 m
DBSCAN radius: 20 mm
minimum samples: 15
slice thickness: 20 mm
spline samples: 400
```

### Primary branches

```text
color: [0, 1, 0]
voxel size: 10 mm
maximum parent distance: 0.1 m
minimum length: 0.05 m
DBSCAN radius: 30 mm
minimum samples: 5
parent-obstacle removal: 15 mm
slice thickness: 10 mm
spline samples: 100
```

### Secondary branches

```text
color: [1, 0, 0]
voxel size: 0.000005 m
maximum parent distance: 0.06 m
minimum length: 0.02 m
DBSCAN radius: 30 mm
minimum samples: 4
parent-obstacle removal: 10 mm
slice thickness: 5 mm
spline samples: 50
```

The secondary voxel size is:

```text
0.000005 m = 0.005 mm
```

which is implausibly small relative to Azure Kinect depth precision and is likely stale or erroneous.

---

# `tree_component.py`

## Purpose

Represents an individual trunk or branch as a structured component containing:

```text
raw point cloud
fitted centerline
parent
children
junction point
length
direction
diameter measurements
representative diameter
```

This creates a hierarchy such as:

```text
trunk
├── primary branch
│   ├── secondary branch
│   └── secondary branch
└── primary branch
```

---

## Centerline Fitting

The main process is:

```text
raw component cloud
→ PCA dominant direction
→ slices perpendicular to that direction
→ centroid per slice
→ optional parent junction insertion
→ B-spline fitting
→ fixed number of fitted points
```

This creates a skeleton centerline for each component.

---

## Length and Diameter Findings

Several implementation details appear experimental or inconsistent:

* fitted length is calculated as endpoint-to-endpoint distance rather than spline arc length;
* comments refer to a 6 cm branch measurement region, but the code uses 15 cm;
* comments refer to a 6 cm trunk region, but the code uses 10 cm;
* comments mention a 30th or 40th percentile, but the implementation uses a weighted average;
* diameter is calculated after discarding the local `y` coordinate rather than using a true local 2D basis perpendicular to the branch;
* trunk selection appears to minimize absolute mean `x`, despite a comment mentioning both X and Y;
* secondary branch assignment depends on primary-branch iteration order;
* empty parent point clouds are not safely handled before KD-tree construction.

These issues are relevant if reproducing Dawood’s horticultural measurements, but they do not invalidate the basic RGB-D reconstruction and ICP approach.

---

# `pc_utils.py`

Provides shared helpers for:

```text
DBSCAN clustering
voxel downsampling
cluster visualization
Open3D → ROS PointCloud2 conversion
```

The Open3D-to-ROS conversion is duplicated in `pointcloud_processing.py` and should be consolidated.

---

# `test.py`

Only publishes an example RViz `LINE_STRIP`.

It does not test:

* motion planning;
* point-cloud reconstruction;
* semantic segmentation;
* skeletonization;
* or Gazebo conversion.

It can be ignored for the current objective.

---

# `tfs.rviz`

The saved RViz configuration confirms the intended outputs:

| Topic                   | Intended meaning                          |
| ----------------------- | ----------------------------------------- |
| `/inference`            | segmentation visualization                |
| `/original_point_cloud` | intended raw point cloud                  |
| `/output_point_cloud`   | accumulated semantic reconstruction       |
| `/final_pointcloud`     | fitted skeleton points                    |
| `/skeleton_visualizer`  | centerlines, diameter markers, and labels |

The fixed frame is:

```text
base_link
```

The current reconstruction node creates `/original_point_cloud` but does not publish to it, indicating an incomplete or outdated code path.

---

# Relevance of Each Pipeline to the Gazebo Goal

## Reuse directly

```text
UR5e motion framework
camera topic synchronization
camera intrinsics
camera-to-base TF
RGB-D back-projection
multiscale ICP
frame acceptance metrics
voxel downsampling
outlier filtering
rosbag recording
offline replay
```

## Remove or bypass initially

```text
YOLO class coloring
BiSeNet branch classes
black-point semantic deletion
class-specific voxel downsampling
skeletonization
DBSCAN branch extraction
diameter measurement
paper figures
paper timing outputs
```

## Potentially reuse later

The fitted skeleton could eventually generate a lightweight procedural model:

```text
centerlines
→ cylinders or tapered tubes
→ simplified trunk and branch collision geometry
→ procedural canopy
```

This would be useful for a lightweight synthetic Gazebo tree but would not be a scan-faithful leafy surface.

---

# Missing Dependencies and Information

The following items are still needed to reproduce the system fully:

```text
ur5e.yml
collision_table.yml
YOLO model weights
physical robot calibration
verified camera-mount calibration
complete live TF tree
complete offline TF tree
package and environment versions
possibly BiSeNet lib/ files
representative rosbag
```

The highest-priority unresolved issue is the live/offline camera-frame discrepancy:

```text
live:    tool0 → camera_base with nonidentity rotation
offline: tool0 → azure with identity rotation
```

The correct `base_link → rgb_camera_link` chain must be verified before trusting metric reconstruction.

---

# Recommended Adaptation Plan

## Phase 1 — Single Stationary Frame

1. Receive RGB, registered depth, and camera intrinsics.
2. Generate an unsegmented RGB point cloud.
3. Transform it into `base_link`.
4. Save it as PLY.
5. Inspect it in Open3D or CloudCompare.

Do not use:

```text
segmentation
ICP
skeletonization
robot motion
```

during this phase.

---

## Phase 2 — Two-View Fusion

1. Move the camera once.
2. Stop and allow the robot to settle.
3. Capture a second frame.
4. Compare:

   * TF-only alignment;
   * TF plus ICP alignment.
5. Save both results.
6. Inspect fitness, RMSE, and visible double surfaces.

This validates:

* camera extrinsics;
* timing;
* depth registration;
* neighboring-view overlap;
* and ICP settings.

---

## Phase 3 — Small Controlled Scan

Use approximately five deliberate viewpoints rather than the complete hardcoded scan.

At each viewpoint:

```text
move
→ settle
→ request one capture
→ receive capture-complete acknowledgment
→ move to next viewpoint
```

Save:

```text
RGB
depth
CameraInfo
TF
ICP correction
individual frame cloud
accumulated cloud
per-frame registration metrics
```

---

## Phase 4 — Leafy Tree Reconstruction

Begin without semantic branch segmentation.

Then test:

```text
workspace crop
ground or table removal
tree-versus-background segmentation
statistical filtering
radius filtering
multiple scan sides
```

A single frontal raster may not reconstruct the back of a dense canopy. Complete tree geometry will require viewpoints from multiple sides or a rotated specimen.

Outdoor scanning also introduces wind-driven nonrigid motion, which can reduce ICP fitness.

---

## Phase 5 — Mesh and Gazebo Preparation

Once a reliable point cloud is produced:

1. remove background, table, and ground;
2. estimate and orient normals;
3. test Poisson or ball-pivoting reconstruction;
4. remove unsupported or low-density surfaces;
5. repair or intentionally preserve holes;
6. simplify the visual mesh;
7. create a much simpler collision representation;
8. preserve metric scale;
9. export visual and collision assets;
10. create the Gazebo model metadata.

Likely output roles:

```text
PLY: master colored point cloud
OBJ / DAE / glTF: visual mesh
STL or primitive decomposition: simplified collision geometry
```

The collision geometry should generally be much simpler than the visual mesh for Warthog simulation performance.

---

# Current Blockers

## RGB-D transport

1. Compressed image transport is missing locally.
2. Jetson-local and received frame rates have not been compared.
3. Best Effort plus queue depth 1 has not yet been measured.
4. The precise bottleneck among camera driver, Jetson, DDS, and Wi-Fi remains unknown.
5. Multiple high-bandwidth streams may be competing for bandwidth.

## Reconstruction

1. The current motion file runs pruning mode instead of scanning mode.
2. The old scan path has not been validated for the current physical setup.
3. Capture coordination is incomplete.
4. The camera TF chain has not been verified.
5. Live and offline camera transforms differ.
6. The physical UR5e calibration and cuRobo configuration files are still needed.
7. The code does not save the final fused point cloud by default.
8. The code does not generate a mesh or Gazebo model.
9. Dormant-tree segmentation will likely remove leafy canopy geometry.
10. The live ROS 2 system and Dawood’s ROS 1 code still require integration or porting.

---

# Decisions

* Continue using Ubuntu 22.04 / ROS 2 Humble for Warthog and Jetson visualization.
* Treat ROS discovery as resolved.
* Prefer Best Effort QoS and queue depth 1 for live image viewing.
* Prefer compressed image transport over Wi-Fi.
* Avoid visualizing multiple high-bandwidth streams simultaneously.
* Prefer Ethernet or direct Jetson recording for full-resolution RGB-D capture.
* Preserve raw RGB-D and TF data in rosbags rather than relying only on a live fused cloud.
* Separate Dawood’s acquisition/reconstruction core from his semantic skeletonization layer.
* Begin the new reconstruction with original RGB or geometry-only clouds.
* Do not run the full hardcoded scanning path on the UR5e until its workspace and collision assumptions are validated.
* Use a stop–settle–capture handshake for physical scanning.
* Save PLY/PCD outputs before attempting mesh generation.
* Treat STL as a possible collision format rather than necessarily the best visual asset format.

---

# Next Actions

## RGB-D transport

1. Install image transport plugins:

```bash
sudo apt update
sudo apt install -y ros-humble-image-transport-plugins
```

2. Verify available transports:

```bash
ros2 run image_transport list_transports
```

3. Restart RViz and use:

```text
Reliability: Best Effort
History: Keep Last
Depth: 1
Durability: Volatile
```

4. Test one camera topic at a time.

5. Measure received frequency:

```bash
ros2 topic hz /sensors/camera_0/color/image
```

```bash
ros2 topic hz /sensors/camera_jetson/color/image_raw
```

6. Inspect publisher QoS:

```bash
ros2 topic info -v /sensors/camera_jetson/color/image_raw
```

7. Measure bandwidth:

```bash
ros2 topic bw /sensors/camera_jetson/color/image_raw
```

8. Compare the same frequency locally on the Jetson.

9. Validate aligned depth and camera-info topics:

```text
/sensors/camera_jetson/aligned_depth_to_color/image_raw
/sensors/camera_jetson/color/camera_info
/sensors/camera_jetson/aligned_depth_to_color/camera_info
```

10. Record a short RGB-D rosbag.

---

## Dawood pipeline adaptation

1. Obtain:

   * `ur5e.yml`;
   * `collision_table.yml`;
   * the physical UR5e calibration;
   * YOLO weights if needed;
   * and a representative rosbag.

2. Inspect the live and offline TF trees.

3. Confirm the exact transform from:

```text
tool0 → rgb_camera_link
```

4. Confirm cuRobo quaternion ordering and relative-pose semantics.

5. Replace manual joint-index swapping with name-based ordering.

6. Add action-server waiting and robust execution failure handling.

7. Create a reduced, parameterized scan routine.

8. Implement:

```text
move
→ settle
→ capture request
→ capture complete
```

9. Build a geometry-only reconstruction mode.

10. Parameterize:

    * topics;
    * camera/base frames;
    * depth cutoff;
    * image resolution;
    * voxel sizes;
    * outlier settings;
    * ICP thresholds;
    * live/offline timeout;
    * output directory.

11. Save:

    * RGB;
    * depth;
    * intrinsics;
    * TF;
    * ICP corrections;
    * individual frame clouds;
    * final fused PLY/PCD;
    * accepted/rejected-frame metrics.

12. Validate one stationary frame, then two frames, then a five-view scan before attempting the original scan path.

13. Only after reconstruction quality is verified, begin mesh reconstruction and Gazebo integration.

---

# End-of-Day Status

The laptop successfully displayed remote Warthog and Jetson RGB images in RViz2, proving end-to-end ROS 2 discovery and image decoding. The stream was still far below real-time performance at approximately `0.2 Hz`, and compressed image transport was unavailable locally.

Dawood’s available repository files were reviewed at the architectural, launch, configuration, and function level. The central reusable method was identified as:

```text
registered RGB-D
→ camera intrinsics
→ camera-to-base TF
→ Open3D point cloud
→ coarse-to-fine point-to-plane ICP
→ filtering
→ global fusion
```

The semantic segmentation and skeletonization layers were identified as research-specific dormant-tree processing that should be disabled for the initial spring/fall reconstruction.

The immediate engineering priority is not yet STL generation. It is to establish a reliable, synchronized, geometry-only RGB-D scan that can be saved as a metric PLY/PCD and reproduced offline from a rosbag. Once that core is verified, the resulting cloud can be cleaned, meshed, simplified, and packaged for Gazebo.
::: 

