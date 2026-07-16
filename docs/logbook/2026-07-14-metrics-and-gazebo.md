7-14-26_day_3_metrics_and_gazebo.md

# Priors
## Read *Benchmark of visual and 3D lidar SLAM systems in simulation environment for vineyards*
- Read the vineyard benchmark and scenario-execution paper. Decide on likely metrics: launch success, map creation success, plan success, completion rate, collisions, time-to-goal, path length, CPU/GPU notes, and qualitative failure categories.

- RTAB achieves superior performance with the lowest RMSE across all scenarios compared to other systems

## Initial Evaluation
How do I evaluate "okay this simulation works well enough"? I need metrics to compare and test whether the environment is simulating correctly. I need to test the robot's navigation (is it bumpy and is this being received; are the rows wide enough, or too wide; is it too long or too short; are trees and plants too hard or too easy, etc), its equipment (if LiDAR, is it working, or RGBD cameras, etc, which SLAM algorithm, etc). 

The paper creates 4 scenarios:
– Scenario 0 (S0): Move in a straight line. That is mainly to evaluate the drift of the generated trajectory. Trajectory length is 25 m.
– Scenario 1 (S1): Send the robot to inspect a row and get back from the same row. Trajectory length is 48.6 m.
– Scenario 2 (S2): Send the robot to inspect a row and get back from the adjacent row. Trajectory length is 54.6 m.
– Scenario 3 (S3): Inspect multiple rows. That is to simulate a real-life inspection scenario with multiple loop closures. Trajectory length is 101.2 m.

## Metrics from Paper
The output trajectory of a SLAM system can be evaluated by finding the absolute distance between the estimated trajectory and the ground truth. The Absolute Trajectory Error (ATE) is defined as the average deviation from the ground truth trajectory.

# Evaluation Goal
The simulator should not be considered complete merely because Gazebo launches and the Warthog can move. It should be evaluated at four levels:

1. Simulation and ROS integration.
2. Sensor and orchard-world correctness.
3. Mapping and localization performance.
4. Autonomous-navigation task performance.

The initial metrics will establish whether the platform is reproducible and useful for pre-field testing. More detailed simulation-to-real validation can be added later.

## Level 1: Simulation and integration health

> "Can the complete system start and communicate correctly?"

| Metric                      | Measurement                                                  |
| --------------------------- | ------------------------------------------------------------ |
| Launch success              | Percentage of launches completing without fatal errors       |
| Startup time                | Seconds from launch command to ready state                   |
| Robot spawn success         | Did the Warthog spawn at the expected pose?                  |
| Required topic availability | Percentage of expected ROS topics present                    |
| TF validity                 | Missing transforms, disconnected TF trees, timestamp errors  |
| Sensor publication rate     | Actual Hz versus configured Hz                               |
| Simulation real-time factor | Simulated time divided by wall-clock time                    |
| Repeatability               | Does the same configuration behave consistently across runs? |
| Resource usage              | Average/peak CPU, GPU, and RAM                               |
| Crash frequency             | Crashes or unrecoverable failures per trial                  |


## Level 2: Sensor and world correctness

> "Are the world and sensors producing plausible inputs?"

**LiDAR**
- Expected scan or point-cloud topic exists.
- Correct frame ID.
- Expected update rate.
- Maximum and minimum range behave correctly.
- Trees appear as obstacles at plausible distances.
- Ground returns are appropriate for the LiDAR orientation.
- No unexplained missing sectors or self-collisions.
- Point density is consistent with the configured sensor.

**RGB or RGB-D camera**
- Image topic exists.
- Camera info topic exists.
- Depth aligns with RGB, if RGB-D.
- Resolution and frame rate match configuration.
- Objects at known distances have approximately correct depth.
- Camera pose and field of view match the physical mounting.

**IMU and odometry**
- Stationary robot reports approximately zero velocity.
- Straight motion produces plausible linear odometry.
- Turns produce plausible angular motion.
- IMU orientation and acceleration axes are correct.
- No obvious frame-axis inversions.

**Orchard geometry**
- Row width.
- Tree spacing.
- Row length.
- Trunk diameter or obstacle footprint.
- Headland or turning-space width.
- Ground roughness or slope.
- Collision geometry matches visible geometry.
- Robot clearance through rows.

## Level 3: Mapping and localization performance

| Metric                   | Meaning                                                          |
| ------------------------ | ---------------------------------------------------------------- |
| Map creation success     | Did the SLAM system produce a usable map?                        |
| ATE RMSE                 | Global deviation between estimated and ground-truth trajectories |
| Relative Pose Error      | Local drift over short trajectory intervals                      |
| Final-position drift     | Difference between estimated and true final pose                 |
| Loop-closure success     | Whether revisiting an area corrected accumulated drift           |
| Map completeness         | Percentage of expected orchard structure represented             |
| Map consistency          | Duplicate rows, warped rows, broken geometry, ghost obstacles    |
| Localization loss events | Number of times tracking failed                                  |
| Relocalization time      | Time needed to recover after tracking loss                       |
| Processing rate          | Whether SLAM keeps up with sensor data in real time              |
| CPU/GPU/RAM              | Computational cost of the selected SLAM approach                 |

ATE tells you how far the estimated trajectory differs from simulation ground truth. Relative Pose Error is also worth adding because a trajectory can have moderate global alignment while accumulating problematic local motion error.

## Level 4: Autonomous-navigation performance

> Can the robot accomplish representative orchard tasks reliably?

| Metric                        | Suggested definition                                            |
| ----------------------------- | --------------------------------------------------------------- |
| Plan success rate             | Valid global plan generated / navigation requests               |
| Goal completion rate          | Goals reached within tolerance / attempted goals                |
| Collision rate                | Trials with any collision / total trials                        |
| Collision count               | Number of collision events per trial                            |
| Time to goal                  | Seconds from accepted goal to completion                        |
| Path length                   | Total distance traveled                                         |
| Path efficiency               | Actual path length / shortest feasible or reference path length |
| Planning latency              | Time required to produce or re-produce a plan                   |
| Recovery count                | Number of recovery behaviors triggered                          |
| Intervention count            | Manual resets or teleoperation interventions                    |
| Goal-position error           | Final distance and heading error relative to target             |
| **Minimum obstacle clearance**| Closest distance to trees or row boundaries                     |
| **Cross-track error**         | Lateral deviation from desired row centerline                   |
| Oscillation events            | Repeated forward/backward or heading corrections                |
| Stuck rate                    | Trials terminated because progress stopped                      |
| Costmap validity              | Missing, stale, inflated incorrectly, or false obstacles        |
| Controller command smoothness | Abrupt changes in linear/angular velocity                       |
| Real-time factor              | Whether the experiment ran at usable simulation speed           |


## Four Scenarios
### S0 — Straight row traversal

Purpose:

Odometry and localization drift.
Controller stability.
Cross-track error.
Basic sensor and costmap operation.

Metrics:

Completion.
ATE/RPE.
Cross-track RMSE.
Time.
Path length.
Minimum clearance.
Oscillations.

### S1 — Enter row, travel down, return through same row

Purpose:
- Rotation and re-entry.
- Accumulated SLAM drift.
- Reverse-direction perception.
- Potential loop closure.

Metrics:
- Completion.
- ATE/RPE.
- Final pose error.
- Loop-closure behavior.
- Recovery events.
- Collision count.

### S2 — Enter one row, return through adjacent row

Purpose:
- Headland turning.
- Global planning.
- Correct row separation in the map.
- Avoiding false shortcuts through trees.

Metrics:
- Plan success.
- Completion.
- Path efficiency.
- Headland turning clearance.
- Wrong-row entry count.
- Collision count.

### S3 — Multi-row orchard mission

Purpose:
- Longer-duration stability.
- Repeated loop closures.
- Costmap and localization consistency.
- More realistic mission execution.

Metrics:
- Completion rate.
- ATE/RPE.
- Total time.
- Distance.
- Recovery count.
- Localization failures.
- CPU/GPU/RAM.
- Real-time factor.

To Consider Adding:
### S4 — Perturbed or stress-test scenario

Vary one controlled factor:

- narrower rows,
- uneven terrain,
- sensor noise,
- missing trees,
- different lighting, if camera-based,
- reduced LiDAR range,
- increased speed.