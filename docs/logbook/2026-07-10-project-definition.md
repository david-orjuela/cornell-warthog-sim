\# Cornell Warthog Orchard Simulation Platform



\## Project Objective



Extend the Cornell AgRobotics Lab’s existing simulation work into a reproducible Gazebo-based apple-orchard environment in which a simulated Clearpath Warthog can be launched, operated, mapped, and used for initial testing of autonomous-navigation algorithms before field deployment.



The project will build on the previous intern’s work rather than begin from scratch.



\## Motivation



The lab currently needs to transport the physical Warthog to an orchard to test navigation algorithms and software changes. Field testing is time-consuming, and even a small software failure may require returning the robot to the lab, correcting the problem, and repeating the deployment.



A reusable simulation platform would provide an initial validation stage for detecting integration problems and evaluating navigation behavior before using the physical robot. Divyanth identified this as potentially useful across multiple lab projects and as a contribution that could support future publication work.



\## Narrow Scope for the Cornell Visit



The initial scope is not to create a perfectly photorealistic digital twin of an orchard or solve every aspect of autonomous navigation.



The goal is to produce one functioning, reproducible end-to-end simulation workflow:



1\. Launch the selected Gazebo orchard world.

2\. Spawn the appropriate Warthog model and sensor configuration.

3\. control or navigate the Warthog through orchard rows.

4\. Produce or load a map using the lab’s selected mapping approach.

5\. Run at least one representative navigation or planning workflow.

6\. Record whether the workflow succeeds and where it fails.

7\. Document installation, launch, configuration, and known limitations.



\## Definition of Done



The initial project is complete when a new lab member can follow the repository documentation on the supported Linux environment and reproduce the following workflow without undocumented manual intervention:



\* Build or install all required dependencies.

\* Launch Gazebo with an apple-orchard world containing navigable rows.

\* Spawn the Warthog with the agreed-upon sensor configuration.

\* Confirm that required ROS topics, transforms, and simulated sensor outputs are available.

\* Drive the robot manually through at least one orchard row.

\* Generate or use a map of the environment.

\* launch one agreed-upon navigation stack or planning algorithm.

\* Send the Warthog to one or more goals without colliding with static orchard geometry.

\* Save logs, maps, configuration files, and representative results.

\* Reproduce the workflow using a documented launch command or small set of commands.



Completion does not require the simulated system to match real-world orchard performance perfectly. Simulation-to-reality validation is a later research stage.



\## Expected Deliverables



\* Reproducible ROS/Gazebo workspace.

\* Version-controlled orchard world and configuration files.

\* Warthog model integration.

\* Documented simulated sensor configuration.

\* Launch files for the end-to-end workflow.

\* Mapping and navigation configuration.

\* Basic validation procedure and results.

\* README containing installation and execution instructions.

\* Known-issues and future-work document.

\* Architecture or system diagram.

\* Short demonstration video or screen recording, if feasible.



\## Current Blockers



\### Immediate blockers



\* The previous intern’s repository has not yet been transferred.

\* The project’s existing Ubuntu, ROS, and Gazebo versions are unknown.

\* The current completion state of the previous intern’s mapping work is unknown.

\* Access to an appropriate Linux workstation or laptop has not yet been confirmed.



\### Technical unknowns



\* Exact Warthog model and package used by the previous intern.

\* Physical Warthog sensor configuration that should be reproduced.

\* Whether the lab uses ROS 1 or ROS 2.

\* Whether the repository uses Gazebo Classic or modern Gazebo.

\* Existing orchard-world source and licensing.

\* Mapping package previously attempted.

\* Navigation stack and planner that the lab wants to evaluate.

\* Existing errors in launch files, transforms, sensor topics, or configuration.

\* Required computing and GPU resources.

\* Preferred destination and ownership of the maintained repository.



\## Assumptions to Validate



\* A usable open-source Warthog simulation model exists.

\* The previous intern created a recoverable Git repository.

\* Some orchard environment or reusable orchard assets already exist.

\* The unfinished work is sufficiently complete to provide a starting point.

\* The lab’s primary short-term need is navigation and mapping rather than photorealistic perception.

\* The physical Warthog’s relevant sensors can be represented in Gazebo.

\* A supported Linux machine can be provided or configured.

\* The project can continue after July 30 if full evaluation is not completed during the visit.



\## Explicit Non-Goals for the Initial Three Weeks



Unless Divyanth changes the scope, this phase will not attempt to:



\* Construct an exact digital twin of a specific Cornell orchard.

\* Develop a new SLAM or path-planning algorithm from first principles.

\* Model all vegetation physics or seasonal variation.

\* Achieve validated simulation-to-real performance equivalence.

\* Build an entire perception system unrelated to navigation.

\* Replace the lab’s existing robotics software architecture.

\* Support several ROS and Gazebo versions simultaneously.



\## Immediate Actions



1\. Obtain the previous intern’s repository and technical handoff.

2\. Request the exact Ubuntu, ROS, Gazebo, and package versions.

3\. Secure a suitable Linux machine.

4\. Obtain the physical Warthog and sensor specifications.

5\. Determine the preferred repository destination and maintainers.

6\. Run the existing project unchanged before modifying it.

7\. Convert all discovered failures into tracked GitHub issues.

8\. Agree with Divyanth on one minimum end-to-end demonstration.

9\. Establish one measurable navigation evaluation beyond merely showing that the simulator launches.



\## Questions for the Handoff Meeting



\* What operating system, ROS distribution, and Gazebo version does the repository require?

\* Is this Gazebo Classic, Ignition Gazebo, or modern Gazebo?

\* Which branch or commit represents the latest working state?

\* What currently launches successfully?

\* What was the last end-to-end workflow that worked?

\* What part of mapping remains unfinished?

\* Which errors or limitations were already known?

\* Which Warthog model and sensor plugins are used?

\* Does the simulated sensor suite match the physical Cornell Warthog?

\* What orchard assets were used, and where did they come from?

\* Which SLAM and navigation packages were attempted?

\* Are there recorded commands, maps, bags, videos, or experiment notes?

\* What would the former intern have implemented next?

\* What hardware was used to run the simulation?

\* Are there uncommitted files or local configuration changes?

\* What licenses apply to the model, world, and imported assets?



\## Success Beyond the Minimum



A stronger research contribution would add a repeatable evaluation layer rather than only producing a working simulation. This could include predefined routes, automated trial execution, collision and completion metrics, localization error where ground truth is available, path length, traversal time, intervention count, and controlled variations in orchard geometry or sensor noise.



That evaluation layer would make the platform more useful for comparing future algorithms and more defensible as a publication contribution.



