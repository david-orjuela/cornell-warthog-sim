# 2026-07-24 — UR5e Raster Motion and Capture Integration

## Day Objective

The primary goal for Friday was to move beyond camera-only reconstruction and begin integrating the real UR5e motion stack with the RGB-D capture pipeline.

The work focused on:

1. understanding Dawood's raster-scan implementation and its relationship to the reconstruction package;
2. preparing the ROS 2 MoveIt and Universal Robots dependencies;
3. launching the UR5e driver and trajectory controllers;
4. generating and replaying a recorded scan trajectory;
5. validating the first trajectory segments cautiously on hardware;
6. diagnosing an unsafe trajectory-controller rejection;
7. connecting each settled robot pose to the reconstruction node's capture request/result interface.

---

## Workspace and Package Organization

Two separate workspaces were in use and needed to remain conceptually distinct:

- `~/dev/cornell-warthog-sim`
  - contains the migrated `reconstruct` package;
  - contains `pointcloud_processing.py`, the RGB-D reconstruction node;
  - contains the reconstruction configuration in `src/reconstruct/config/params.yaml`.

- `~/dev/dawood_tree_scanning_ur5e`
  - contains Dawood's `arm_scan_motion` package;
  - contains the MoveIt configuration and UR5e bringup launch files;
  - contains the raster trajectory generator and the `scanning_trajectory` replay executable;
  - contains `recorded_scan_trajectory.json`.

This distinction became important after an initial search looked for the trajectory path under `cornell-warthog-sim`, even though `arm_scan_motion` and its JSON lived in Dawood's workspace.

---

## Motion Package and Raster-Scan Design

Dawood's motion package generates a structured scan around a manually selected starting pose.

The intended motion is a raster, or snake, pattern:

- move across one row;
- step vertically to the next row;
- reverse horizontal direction;
- continue alternating until all rows are covered.

The scan parameters considered included:

- four rows;
- 0.15 m row spacing;
- 0.15 m lateral offset;
- 15-degree camera look angle;
- velocity scaling of 0.05;
- acceleration scaling of 0.05.

The planned workflow separated trajectory validation from recording:

- use `record_trajectory:=false` for a cautious motion-only validation;
- use `record_trajectory:=true` only after the scan geometry was confirmed;
- save the resulting trajectory to an explicit JSON path;
- increase `max_steps` only as more of the sequence was validated.

A 42-step trajectory was considered for the complete four-row scan. The additional steps were intended to increase view coverage rather than change the underlying raster geometry.

The scan axes are defined in the end-effector frame. Because the RealSense mount orientation can rotate the camera relative to the tool, `scan_axis_u_ee` and `scan_axis_v_ee` must match the actual mount rather than being assumed from the nominal tool axes.

---

## ROS 2 and MoveIt Dependency Setup

The Dawood workspace initially failed to build or launch because required ROS 2 packages were missing.

Observed issues included:

- missing `moveit_ros_planning_interface` during `colcon build`;
- missing `realsense2_description`;
- missing `ur_robot_driver`.

The environment was sourced in this order:

1. `/opt/ros/humble/setup.bash`;
2. `~/dev/cornell-warthog-sim/install/setup.bash`;
3. `~/dev/dawood_tree_scanning_ur5e/install/setup.bash`, as appropriate for the command.

`realsense2_description` was later found under `/opt/ros/humble`. The UR driver and MoveIt packages also had to be available before the hardware bringup could succeed.

Old ROS processes were treated as a possible source of duplicate nodes, stale controllers, or launch conflicts. Only one UR driver and one `/controller_manager` should be active.

---

## UR5e Bringup and Controller State

The real robot driver must remain running while a separate terminal executes the trajectory.

The intended arrangement became:

1. launch `arm_scan_motion bringup.launch.py` with `use_sim_time:=false`;
2. leave that terminal running;
3. verify the desired trajectory controller is active;
4. run the trajectory executable from another terminal;
5. run the reconstruction node from a third terminal when capture is enabled.

The controller manager exposed several possible trajectory controllers:

- `scaled_joint_trajectory_controller`;
- `joint_trajectory_controller`;
- `passthrough_trajectory_controller`.

At one diagnostic point, all three were loaded but inactive, while state and status broadcasters were active. The existence of a `follow_joint_trajectory` action name did not by itself mean the corresponding controller could execute motion.

The replay executable targeted `joint_trajectory_controller`. Therefore, that controller needed to be active before testing.

Both of the checked time parameters were correctly set to wall-clock time:

- `/controller_manager use_sim_time = False`;
- `/joint_trajectory_controller use_sim_time = False`.

For hardware operation, simulation time must remain disabled.

---

## Recorded Trajectory File

The motion generator produced:

`~/dev/dawood_tree_scanning_ur5e/recorded_scan_trajectory.json`

The file contained:

- six UR5e joint names;
- a sequence of named trajectory segments;
- per-point positions;
- per-point velocities and accelerations;
- `time_from_start_sec` values;
- per-segment pause information.

The replay node supports the `trajectory_input_path` parameter. An absolute path was preferred so the executable could not silently load a different relative JSON file.

---

## Initial Hardware Replay and Safety Fault

The first single-step test corresponded to `Home Base`. It was not a meaningful motion test because the segment contained only one point at `t=0`, already approximately equal to the robot's current home pose.

The first real movement occurred in step 2, `Initial Look Down`.

During that test, the UR controller reported a jump from approximately:

- initial joint value: `-1.25007` rad;
- commanded value: `-1.20133` rad;
- elapsed controller time: approximately 0.002 s.

The controller rejected the command rather than executing the apparent instantaneous jump. Testing beyond step 1 was stopped while the trajectory was inspected offline.

This was treated as a safety-critical execution problem, not as evidence that the intended raster geometry was wrong.

---

## Offline JSON Validation

The boundary between steps 1 and 2 was inspected with `jq`.

Step 1 ended at:

- one point;
- `time_from_start_sec = 0`;
- joint positions approximately `[-0.0599549, -1.25007, 2.29995, 1.57011, -1.56997, -1.56994]`.

Step 2 contained ten points:

- first point at `t=0`;
- second point at approximately `t=0.0959` s;
- final point at approximately `t=0.8774` s.

The largest position difference between the end of step 1 and beginning of step 2 was only about `3e-5` rad. The segment boundary was therefore effectively continuous.

Within step 2:

- samples were generally spaced by approximately 0.10–0.11 s;
- the relevant joint moved gradually from `-1.25008` to `-1.20133` rad;
- the final point duplicated the previous point after another approximately 40 ms;
- total intended duration was approximately 0.877 s.

This cleared the JSON and Dawood's planned spatial path. The file did not command the final pose after 2 ms.

---

## Replay-Code Diagnosis

An early hypothesis was that the replay code might overwrite trajectory points or send each point as a separate action. Inspection of `scanning_trajectory.cpp` disproved that theory.

The code:

- appended all recorded points to one `FollowJointTrajectory` goal;
- preserved positions;
- preserved velocities and accelerations;
- preserved each recorded `time_from_start`.

The more plausible issue was that every segment began with its first point at exactly `time_from_start = 0`, while the goal header also implied an immediate start. By the time the controller processed the goal on its next update, the first sample was already at or behind the start boundary.

The replay path also differed from executing the original MoveIt plan directly. A trajectory that was valid in the MoveIt planning context was being serialized and then sent later as an independent controller goal.

---

## Positive Trajectory Lead-In

A 0.25-second positive lead-in was added to every replayed point:

`pt.time_from_start = rclcpp::Duration::from_seconds(trajectory_lead_in_sec + rpt.time_from_start_sec);`

The same lead-in was added to the computed segment duration used for waiting on the result:

`seg_duration = trajectory_lead_in_sec + seg.points.back().time_from_start_sec;`

The constant was declared once before the segment loop:

`constexpr double trajectory_lead_in_sec = 0.25;`

This changes step 2 from a trajectory beginning at `t=0` and ending at approximately `t=0.877` s to one beginning at `t=0.25` s and ending at approximately `t=1.127` s.

The purpose is to give the controller a valid future first point rather than an already-expired zero-time point.

Increasing the inter-segment `pause_ms` would not solve this issue because pausing before sending a goal does not change the goal's internal trajectory timestamps.

---

## Motion-Only Validation

The replay node enables reconstruction captures by default.

An initial `max_steps:=1` run loaded the trajectory successfully but exited after 15 seconds because no capture pipeline was connected:

`Capture pipeline was not connected within 15.0 s. Start the reconstructor first or set capture_enabled:=false.`

For motion-only validation, capture therefore needed to be disabled explicitly with:

`-p capture_enabled:=false`

This failure was unrelated to the UR controller or motion. It confirmed that the replay executable checks for the reconstruction request/result interface before beginning a capture-enabled run.

---

## Reconstruction Integration

The reconstruction node was launched from `cornell-warthog-sim` with wall-clock time:

`ros2 launch reconstruct scan_tree.launch.py params_file:="$HOME/dev/cornell-warthog-sim/src/reconstruct/config/params.yaml" use_sim_time:=false`

It reported:

- readiness on `/tree_scan/capture_request`;
- RGB-D topics from `/sensors/camera_jetson`;
- output to `~/dev/tree_scans/indoor_model_tree_1/global_point_cloud.ply`.

The trajectory node connected successfully to:

- `/tree_scan/capture_request`;
- `/tree_scan/capture_result`.

The intended integrated cycle was:

`move → wait for robot result → settle → request capture → wait for capture result → continue`

---

## Capture Timeout

A five-step replay was started with the reconstruction node connected.

Step 1 reached the home pose and requested capture 1 after a 1000 ms settle time. The trajectory node then timed out after 15 seconds and aborted.

The reconstructor log showed:

- capture request queued at approximately `18:54:37.241`;
- trajectory timeout at approximately `18:54:52.240`;
- scan-complete message received immediately after the abort;
- reconstructor began waiting for five new synchronized RGB-D frames at approximately `18:55:03.260`;
- capture accepted at approximately `18:55:03.725`.

The capture therefore succeeded, but about 11 seconds after the trajectory node had already timed out.

Once synchronized frames became available, burst collection and point-cloud processing took less than half a second. Most of the delay occurred before processing while waiting for new synchronized RGB, aligned depth, and camera-info messages.

The accepted first capture produced:

- first-frame/TF-only acceptance;
- approximately 81.6% valid median-depth pixels;
- 5,378 global points;
- approximately 229 ms reported processing time.

---

## Interpretation of the Timeout

The request/result communication worked correctly. The immediate problem was delayed RGB-D synchronization or intermittent camera delivery, not point-cloud computation.

The replay timeout of 15 seconds was shorter than the approximately 26-second delay between the capture request and the reconstructor beginning its burst.

The following camera streams were identified for rate and timestamp checks:

- `/sensors/camera_jetson/color/image_raw`;
- `/sensors/camera_jetson/aligned_depth_to_color/image_raw`;
- `/sensors/camera_jetson/aligned_depth_to_color/camera_info`.

Increasing the capture timeout to at least 35 seconds could permit testing to continue, but would only mask the underlying camera or synchronization delay.

Another lifecycle issue was exposed: after the trajectory node aborted, it published scan completion while the reconstructor still had a pending request. The reconstructor announced the final PLY and then later processed the queued capture.

The completion behavior should eventually be made consistent by either:

- cancelling pending captures when a scan is aborted or completed; or
- delaying finalization until the pending capture queue is empty.

Otherwise, the file described as final can still be modified by a late capture.

---

## Operational Safety Notes

The following conditions were established before future motion tests:

- use only one UR driver and one controller manager;
- keep `use_sim_time:=false` for the hardware driver, controller manager, trajectory controller, and reconstruction node;
- verify the controller targeted by the replay code is active;
- keep the pendant's External Control program running and the robot enabled;
- begin with `max_steps:=1`;
- enable additional steps only after the previous step behaves correctly;
- keep the pendant emergency stop immediately accessible;
- do not assume action-server visibility means the controller is active;
- do not use a longer pause as a substitute for valid internal trajectory timing;
- use absolute paths for the trajectory JSON and parameter files.

---

## End-of-Day Status

By the end of Friday, July 24:

- Dawood's raster-scan structure and configurable geometry were understood;
- the two workspaces and their responsibilities were clarified;
- missing MoveIt, RealSense-description, and UR-driver dependencies were identified;
- the UR5e bringup and controller requirements were established;
- a recorded six-joint scan trajectory was available as JSON;
- the replay node could load the JSON and connect to the controller and capture pipeline;
- the JSON was verified to contain continuous, reasonably timed points;
- the unsafe step-2 rejection was isolated to the controller replay path rather than the recorded raster geometry;
- a 0.25-second lead-in was added to each replayed segment;
- motion-only runs were separated from capture-enabled runs;
- the reconstruction request/result interface connected successfully;
- one integrated capture was eventually accepted and written to the global PLY;
- the capture completed too late for the replay node's 15-second timeout;
- delayed synchronized RGB-D delivery became the next integration blocker;
- scan finalization with pending captures was identified as a code-level lifecycle issue;
- the full multi-step robot scan was not yet considered validated.

---

## Next Steps

1. rebuild `arm_scan_motion` with the 0.25-second trajectory lead-in;
2. verify `joint_trajectory_controller` is active before every replay;
3. rerun step 1 with `capture_enabled:=false`;
4. cautiously validate step 2 only after the time and controller checks pass;
5. measure the publication rates of RGB, aligned depth, and camera-info topics;
6. compare their header timestamps and identify the synchronization delay;
7. inspect the reconstructor's five-frame burst and message-filter behavior;
8. temporarily raise the capture-result timeout only if needed for controlled testing;
9. prevent scan completion from finalizing while captures remain pending;
10. validate the camera-to-tool transform and physical mount orientation;
11. confirm the raster axes and look angle match the installed camera;
12. increase `max_steps` incrementally rather than immediately running the full scan;
13. record a new four-row, 15-degree trajectory after motion safety is confirmed;
14. run the complete move–settle–capture sequence;
15. inspect the fused PLY for coverage, registration quality, and tree detail.
