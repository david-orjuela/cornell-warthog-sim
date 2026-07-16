# July 15 — Inherited Workspace Discovery and Recovery

## Major Development

Access was obtained to the lab Linux desktop previously used by Nidhish. Two substantial ROS 2 workspaces and associated documentation, maps, models, evaluation results, TF diagrams, sensor CAD files, and rosbag recordings were located.

This removes the immediate dependency on waiting for the repository handoff meeting and allows the existing system to be reconstructed before meeting with Nidhish.

## Workspaces Identified

### `cornell_orchard_ws`

Preliminary contents suggest that this workspace contains orchard simulation or mapping work, including:

* source packages;
* a colored orchard point cloud;
* TF diagrams;
* previous build and execution logs.

### `orchard_navigation_rl_ws`

This workspace contains a learned multi-modal navigation architecture for the Warthog, including:

* camera and LiDAR data collection;
* behavior-cloning training;
* learned-policy deployment;
* orchard-world assets;
* command-velocity safety multiplexing;
* odometry diagnostics;
* camera-only, LiDAR-only, and multimodal evaluations;
* model checkpoints and rosbag recordings.

The README specifies ROS 2 Humble and a Warthog/Husky platform.

## Preliminary Architecture

The learned controller uses:

* a Stable Diffusion VAE for visual feature extraction;
* a PointNet encoder for 3D LiDAR features;
* early feature fusion;
* a GRU over 13-frame temporal sequences;
* prediction of linear and angular velocity commands.

## Preliminary Sensor Evidence

The files and TF diagrams provide evidence of:

* an RGB camera;
* 3D LiDAR, likely an Ouster unit;
* an Xsens IMU;
* wheel odometry.

The exact correspondence with the current physical Cornell Warthog remains to be verified.

## Preliminary Evaluation Findings

* LiDAR-only produces the strongest linear-velocity regression performance.
* Multimodal fusion produces the strongest angular-velocity regression performance and turn-classification macro F1.
* Camera-only results are close to a majority-class turn baseline.
* Camera-only and multimodal models fail to detect stop events in the current evaluation.
* All models show weaknesses on minority turn classes.
* Several models produce a narrower velocity range than the ground truth.

These results suggest that closed-loop navigation performance cannot be inferred from the existing offline metrics alone.

## Updated Research Opportunity

A potentially valuable contribution is to connect the existing learned navigation models to the Gazebo orchard environment and build a reproducible closed-loop evaluation framework.

Possible comparisons include:

* camera-only;
* LiDAR-only;
* multimodal fusion;
* conventional Nav2 baseline, if appropriate.

Possible metrics include completion rate, collisions, cross-track error, obstacle clearance, stop success, turning success, path length, runtime, and robustness to controlled environmental variation.

## Immediate Technical Plan

1. Preserve the original directories without modification.
2. Create clean working copies under my account.
3. Record Ubuntu, ROS, Gazebo, CUDA, and dependency versions.
4. inspect Git state and preserve uncommitted files.
5. reconstruct the intended launch sequence from documentation and launch files.
6. rebuild the ROS workspaces from clean source.
7. launch the orchard environment and Warthog.
8. audit topics, nodes, transforms, sensors, and robot footprint.
9. identify where `map → odom → base_link` is generated.
10. document all blockers and missing assumptions.

## Current Questions

* Which workspace and branch represent Nidhish’s final intended system?
* Which files were never committed?
* Which Gazebo version was used with ROS 2 Humble?
* Was the simulator used for training, testing, or only visualization?
* Which physical sensor configuration should be reproduced?
* Are the current evaluation results based on a held-out route, held-out time sequence, or randomly sampled frames?
* How were stop and turn labels defined?
* Is the project’s primary goal imitation learning, Nav2 evaluation, mapping, or a combined system?
* What constitutes the desired publication contribution?
