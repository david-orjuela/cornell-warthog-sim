# MEMO RESCINDED -- REPOSITORY MADE AVAILABLE EARLY JULY 15.

# Provisional Technical Decision Memo — July 14

## Context

The planned meeting with Divyanth did not occur, and the previous intern’s repository and environment specifications remain unavailable. Divyanth has been contacted, but he was not yet able to confirm the software versions, sensor configuration, or repository details.

## Interim Decision

Until the inherited repository is received, I will use the current Clearpath Simulator Jazzy branch and its included orchard world as a reference implementation.

This work is intended to:

* establish familiarity with the Clearpath simulation workflow;
* confirm that a Warthog can be spawned in an orchard environment;
* understand the roles of `robot.yaml`, ROS-Gazebo bridges, launch files, sensors, topics, and transforms;
* identify the information needed to compare the reference stack with Cornell’s physical Warthog;
* prepare reusable topic-audit and reproducibility procedures for the inherited project.

## Environment Decision

The reference simulator will be installed on the lab Linux desktop rather than Windows because the Clearpath instructions and ROS/Gazebo dependency workflow target Linux. Windows support will not be pursued unless later required by the inherited repository.

## Current Assumptions

* The current Clearpath Jazzy/Harmonic simulator may differ from the prior intern’s stack.
* The included orchard world is a reference environment, not necessarily the final Cornell environment.
* The default sensor suite may not correspond to the physical Cornell Warthog.
* No architectural changes will be committed until the inherited repository and robot configuration are reviewed.

## Immediate Outputs

* Linux environment inventory.
* Reproducible installation log.
* Build and launch result.
* Orchard-world screenshot.
* ROS topic and TF inventory.
* List of default sensors.
* CPU/GPU/RAM notes.
* Errors and blockers.
* Questions requiring confirmation from Divyanth or the former intern.

## Pending Decisions

* Inherited repository versus current Clearpath stack.
* ROS and Gazebo versions.
* Mission-critical sensors.
* Physical Warthog correspondence.
* Mapping and navigation stack.
* Final definition of done.
* Research and publication evaluation target.
