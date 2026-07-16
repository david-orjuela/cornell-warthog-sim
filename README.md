# Orchard Autonomy Sim

A reproducible ROS 2 Humble and Gazebo Fortress platform for
simulating a Clearpath Warthog in orchard environments and evaluating
mapping, perception, and autonomous-navigation workflows.

## Project status

Early reconstruction of an inherited Cornell AgRobotics Lab workspace.

Currently verified:

- [ ] Clean workspace build
- [ ] Orchard world launches
- [ ] Warthog spawns
- [ ] Teleoperation works
- [ ] LiDAR publishes
- [ ] Camera publishes
- [ ] IMU publishes
- [ ] TF tree is valid
- [ ] Mapping works
- [ ] Navigation works
- [ ] Automated evaluation works

## Supported environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- Clearpath simulator packages

## Quick start

See `docs/quickstart.md`.

## Repository scope

This repository contains original integration, evaluation, configuration,
and documentation. External dependencies are imported using `vcstool`.

## Data and models

Large datasets, rosbags, maps, and model checkpoints are stored separately.

## Attribution

See `docs/provenance.md`.
