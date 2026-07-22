# 2026-07-16 — Repository Reconstruction and Configuration Audit

## Objective

Create a clean repository for reconstructing and extending the inherited orchard simulation work, organize the recovered material into a maintainable structure, and begin identifying which configurations and packages represent the latest working system.

The official handoff with Nidhish had not yet occurred and was scheduled for Friday, so all conclusions remained provisional.

## Work Completed

### Created the project repository

Created a new GitHub repository named:

`orchard-autonomy-sim`

The repository is intended to become a reproducible ROS 2 and Gazebo platform for Warthog simulation, mapping, perception, navigation, and evaluation in orchard environments.

Rather than copying Nidhish’s complete workspaces directly, I created a clean scaffold designed to separate:

* ROS packages;
* simulation worlds and models;
* robot and sensor configuration;
* launch files;
* evaluation tools;
* experiment results;
* daily research notes;
* formal documentation;
* external dependencies;
* inherited-file provenance.

The repository was created as a reconstruction and integration workspace, not yet as an authoritative replacement for Nidhish’s original project.

## Initial Repository Structure

```text
orchard-autonomy-sim/
├── README.md
├── LICENSE
├── CITATION.cff
├── CONTRIBUTING.md
├── SECURITY.md
├── .gitignore
├── .gitattributes
├── orchard_autonomy_sim.repos
├── docs/
│   ├── project_charter.md
│   ├── architecture.md
│   ├── setup.md
│   ├── quickstart.md
│   ├── troubleshooting.md
│   ├── known_issues.md
│   ├── inherited_work.md
│   ├── provenance.md
│   ├── handoff_questions.md
│   ├── workflows/
│   ├── evaluation/
│   ├── logbook/
│   └── figures/
├── src/
│   ├── orchard_description/
│   ├── orchard_bringup/
│   ├── orchard_perception/
│   ├── orchard_evaluation/
│   └── orchard_nav_integration/
├── config/
│   ├── robot/
│   ├── sensors/
│   ├── slam/
│   ├── nav2/
│   └── evaluation/
├── worlds/
├── models/
├── launch/
├── scripts/
│   ├── setup/
│   ├── run/
│   ├── evaluation/
│   └── migration/
├── tests/
├── experiments/
├── tools/
└── external/
```

This structure is intentionally broader than the current implementation. Only the directories needed for the first reproducible simulation will be populated initially.

## Migration Strategy

Decided not to copy the complete inherited workspaces directly because they contain:

* generated `build`, `install`, and `log` directories;
* absolute paths tied to Nidhish’s home directory;
* multiple generations of worlds and scripts;
* large maps, datasets, bags, and model checkpoints;
* third-party repositories;
* hardware-specific configuration;
* unclear file provenance and licensing.

The first migration should instead include only:

1. the relevant orchard-description package;
2. the current orchard world and required assets;
3. the latest Warthog simulation launch workflow;
4. a sanitized or clearly labeled simulation robot configuration;
5. teleoperation support;
6. the required mapping configuration;
7. setup and troubleshooting documentation.

Third-party packages such as LIO-SAM should be referenced using a `.repos` file or pinned external dependency rather than copied as original project code.

## Robot Configuration Audit

Inspected several `robot.yaml` configurations.

The files are not simply robot models. They define a particular Clearpath robot deployment, including:

* robot namespace;
* sensor types;
* mounting transforms;
* sensor launch behavior;
* hardware drivers;
* networking;
* generated URDF and launch configuration.

Different researchers appear to have maintained separate versions because they were using different experimental configurations.

### Current `/etc/clearpath/robot.yaml`

The current system configuration defines a simulated Warthog with:

* an Ouster OS1 3D LiDAR;
* a Microstrain IMU;
* three Intel RealSense cameras;
* a front camera;
* left- and right-facing cameras;
* simulation namespace `w200_0000`.

This configuration appears closest to Nidhish’s later three-camera orchard-mapping workflow and is likely the correct initial simulation configuration.

### Other robot configurations

One inspected configuration appeared closer to a physical Ouster deployment because it included:

* sensor IP addresses;
* UDP destination addresses;
* LiDAR and IMU ports;
* Clearpath discovery-server settings.

Another configuration contained:

* custom structural boxes;
* a tower;
* a primary RealSense camera;
* an overhead camera mounted four meters above the robot.

That version appears to be an experimental simulation or data-collection fixture rather than a realistic physical Warthog configuration.

## Configuration Handling Decision

For the private reconstruction repository, sensor types and mounting transforms can be retained.

For any eventual public release:

* real sensor IP addresses;
* hostnames;
* network topology;
* device-specific deployment values

should be removed or converted to placeholders.

The repository should distinguish among:

```text
config/clearpath/
├── robot.sim.yaml
├── robot.hardware.example.yaml
└── README.md
```

The exact physical deployment file should remain outside the public repository or in a private lab configuration repository.

## Orchard Description Inspection

Inspected Nidhish’s `orchard_description` package.

The package contains:

* multiple orchard worlds;
* aggregate orchard meshes;
* individual tree meshes;
* collision meshes;
* dormant, growing, flat, high-resolution, and uneven orchard variants;
* teleoperation and world-generation scripts;
* temporal camera–LiDAR projection code;
* Python and C++ perception implementations.

The asset directory contains several overlapping naming conventions and likely represents multiple development iterations. The final repository should include only assets referenced by the selected current world.

## Issues Identified in `orchard_description`

### Stale Gazebo Classic launch path

`launch/orchard_launch.py` launches:

```python
gazebo --verbose ... -s libgazebo_ros_factory.so
```

This is a Gazebo Classic workflow.

The recovered machine instead uses:

* Ubuntu 22.04;
* ROS 2 Humble;
* Ignition Gazebo Fortress;
* `clearpath_gz`.

The latest documented workflow launches the environment through:

```bash
ros2 launch clearpath_gz simulation.launch.py
```

Therefore, `orchard_launch.py` is likely an older or abandoned launch path and should not be used as the first reproduction target.

### Stale GPU comment

The launch file contains a comment referring to an RTX 4060 even though the current machine uses RTX 5000 Ada GPUs.

The underlying environment variables are generic NVIDIA rendering variables, so the comment is stale but not itself a functional problem.

### Packaging issue for `.world` files

`setup.py` installs:

```python
glob('worlds/*.sdf')
```

but does not install `.world` files.

Because the old launch file attempts to load `orchard.world`, that file may be missing from the installed package share directory after a clean build.

### Likely broken Python entry point

The package declares:

```python
custom_teleop = orchard_description.custom_teleop:main
```

but the script appears under:

```text
scripts/custom_teleop.py
```

rather than inside the importable `orchard_description` Python module.

This explains why the inherited workflow runs the script directly with Python rather than through `ros2 run`.

### Asset duplication and provenance

The package contains many duplicated or historical tree assets, including several naming styles and both aggregate and individual collision meshes.

The exact assets used by the latest world must be identified before migration. The package’s Apache-2.0 declaration does not by itself establish redistribution rights for all imported meshes and textures.

## ROS Dependency Audit

Ran:

```bash
rosdep check --from-paths src --ignore-src -r
```

After updating the rosdep cache, the remaining unresolved dependency was:

```text
ros-humble-gazebo-ros
```

This does not mean ROS 2 Humble or Gazebo are missing.

The machine already contains:

* ROS 2 Humble;
* Ignition Gazebo Fortress;
* Clearpath Gazebo packages;
* ROS–Gazebo bridge packages.

Instead, one package in the inherited workspace appears to declare a dependency on `gazebo_ros`, which belongs to the Gazebo Classic integration stack.

This suggests the workspace contains a mixture of:

* older Gazebo Classic code;
* newer Ignition Gazebo / Clearpath code.

The package declaring `gazebo_ros` must be identified before installing additional simulator packages. The likely candidate is the older `orchard_worlds` simulation package.

## Current Technical Interpretation

The latest likely simulation workflow is:

```text
/etc/clearpath/robot.yaml
        ↓
Clearpath robot generators
        ↓
Generated Warthog description and sensors
        ↓
clearpath_gz simulation.launch.py
        ↓
Ignition Gazebo Fortress orchard world
        ↓
Warthog, LiDAR, IMU, and camera ROS topics
```

The older `orchard_launch.py` and any `gazebo_ros` dependencies likely belong to an earlier development path.

## Decisions

* Keep the new repository private during reconstruction.
* Treat the recovered files as an unofficial handoff until the Friday meeting.
* Do not modify Nidhish’s original directories.
* Do not install Gazebo Classic solely to satisfy the current rosdep warning.
* Use `/etc/clearpath/robot.yaml` as the provisional simulation configuration.
* Build `orchard_description` separately before attempting the entire inherited workspace.
* Launch through `clearpath_gz`, not the stale Gazebo Classic launch file.
* Keep hardware-specific configuration separate from simulation examples.
* Track imported files and authorship in `docs/provenance.md`.

## Questions for Nidhish

1. Which workspace represents the final intended project?
2. Are `cornell_orchard_ws` and `orchard_navigation_rl_ws` intended to run together?
3. Which orchard world is the latest working version?
4. Is `/etc/clearpath/robot.yaml` the intended simulation configuration?
5. Which robot YAML corresponds to the physical Warthog?
6. Is `orchard_launch.py` obsolete?
7. Is the `gazebo_ros` dependency still required?
8. Was Gazebo Classic used at any point in the final workflow?
9. Which tree meshes and textures may be redistributed?
10. Which files were changed locally but never committed?
11. What exact launch sequence produced the latest successful simulation?
12. Which components should be migrated into the new repository?
13. Was the learned policy ever evaluated closed-loop in Gazebo?
14. What specific deliverable would be most useful for publication?

## Next Actions

1. Locate the package declaring the `gazebo_ros` dependency.
2. Build only `orchard_description` from a clean copy.
3. Source ROS 2 Humble and the rebuilt workspace.
4. Launch the orchard through `clearpath_gz`.
5. Verify Warthog spawning and teleoperation.
6. Record all ROS nodes, topics, transforms, and sensor streams.
7. Compare the observed system with the inherited documentation.
8. Bring the reconstructed architecture, blocker list, and targeted questions to the Friday handoff meeting.

## End-of-Day Status

The project now has a clean repository and a migration plan.

The inherited system appears more developed than initially expected, but it contains multiple generations of simulation code and configuration. The primary immediate task is no longer creating a Gazebo environment from scratch. It is identifying, reproducing, and consolidating the latest valid Clearpath Warthog orchard-simulation workflow without carrying forward obsolete code or undocumented machine-specific assumptions.

