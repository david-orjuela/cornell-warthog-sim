# 2026-07-26 — Full-Scan Validation, Filter Evaluation, and Trajectory Expansion

## Day Objective

The primary goal for Sunday was to move beyond the isolated capture-timeout incident and evaluate the reconstruction system as a complete scanning pipeline.

The work focused on:

1. confirming that the complete 32-step UR5e trajectory could run reliably with one capture at every settled pose;
2. evaluating the resulting tree point cloud quantitatively rather than relying only on visual judgment;
3. separating registration quality, point density, voxel resolution, and outlier filtering as distinct reconstruction variables;
4. identifying why an attempted voxel-size comparison initially produced identical results;
5. determining whether missing leaves or branches were caused by filtering or by acquisition coverage;
6. expanding the trajectory to include an additional high row and stronger oblique views;
7. preparing the revised trajectory generator for a cautious hardware test.

These notes begin after the earlier one-off timeout, once the full 32-step run was reported to work correctly.

---

## Full 32-Step Integrated Scan

The complete recorded trajectory was replayed successfully with the reconstruction node connected.

For every trajectory step, the system followed the intended sequence:

`move → settle for 1000 ms → request capture → wait for result → continue`

All 32 capture requests were accepted. The later portion of the scan included:

- Row 1 completion through captures 20–22;
- movement to Row 2 center at capture 23;
- left, right, and neutral-gaze observations across Row 2;
- a direct 0.30 m lateral move between the left and right positions;
- final return to Row 2 center at capture 32;
- automatic return of the arm to the home pose.

The replay ended with:

`Scanning trajectory replay completed successfully.`

This demonstrated that the earlier delayed capture was likely a one-off camera or synchronization event rather than a persistent failure in the trajectory–reconstruction interface.

---

## Full-Run Quantitative Results

The completed scan produced the following headline results:

| Metric | Result |
|---|---:|
| Successful captures | 32/32 |
| Capture success rate | 100% |
| Final stored points from the online run | 57,726 |
| Mean ICP fitness | approximately 0.920 |
| Median ICP fitness | 0.942 |
| Mean ICP RMSE | approximately 3.76 mm |
| Median ICP RMSE | 3.8 mm |
| Mean valid burst pixels | 71.5% |
| Mean capture processing time | 521 ms |
| Capture processing-time range | 341–747 ms |

The reliable 32/32 capture result was an important system-level improvement over the earlier manual capture workflow and the isolated timeout.

The scan also confirmed that explicit capture triggering after motion and settling produced better RGB-D consistency than continuously sampling while the arm moved.

---

## Visual Assessment of the Reconstruction

The resulting RGB-colored reconstruction looked substantially better than the earlier scans.

Positive observations included:

- the overall tree structure was recognizable;
- the capture-trigger workflow reduced motion-related inconsistency;
- the camera frames themselves looked usable;
- the fused cloud appeared coherent enough for continued evaluation;
- the scan covered the intended lower and middle rows successfully.

Remaining visual limitations included:

- leaves often appeared as blobs rather than clearly separated thin surfaces;
- some leaves visible in the RGB frames were not distinct in the final cloud;
- blue or light-colored background points appeared around reflective objects;
- the upper portion of a high-hanging fruit was missed;
- frontal left-to-right coverage did not provide enough true side-view parallax;
- the cloud still looked fuzzy when examined closely.

Some of the background artifacts were already present as reflections or uncertain depth edges in the saved RGB-D frames. Therefore, they were not necessarily introduced only by ICP or the final outlier filter.

---

## Why More Points Do Not Automatically Mean Better Geometry

The visual discussion established a distinction between sampling density and sensing accuracy.

A smaller voxel size can preserve more points and allow closer visual inspection. However, it cannot recover geometric information that the camera never measured accurately.

Below the effective accuracy limit of the RealSense and the registration pipeline, smaller voxels may preserve:

- depth noise;
- doubled leaf edges;
- small ICP alignment errors;
- RGB/depth boundary mismatch;
- uncertain measurements around thin leaves;
- reflective or low-confidence background points.

A point cloud is also not expected to be watertight. Watertightness becomes relevant if a later stage attempts to construct a closed surface mesh. At the point-cloud stage, quality is better characterized by:

- distinguishable leaves, stems, fruit, and trunk;
- limited surface doubling or excessive thickness;
- adequate viewpoint coverage;
- limited isolated noise;
- repeatable geometry across scans;
- sufficient resolution for the eventual downstream task.

---

## Metrics Do Not Fully Determine the Best-Looking Cloud

ICP metrics and point counts are useful, but none directly measures whether the reconstructed leaves look botanically correct.

- ICP fitness measures the fraction of points that find correspondences within the allowed distance.
- ICP RMSE measures correspondence error, not whether each correspondence belongs to the correct physical leaf.
- Point count measures density, not correctness or completeness.
- A dense fuzzy surface can obtain good correspondence metrics.
- A filter can remove a thin leaf tip or branch while improving conventional noise statistics.
- A visually cleaner cloud can have slightly worse ICP statistics than a cloud with more redundant or noisy points.

Therefore, the best reconstruction can be a non-statistical-filter result even if the statistical profile has a marginally lower ICP RMSE.

The conventional metrics should be combined with:

- cropped visual comparisons around the trunk, a branch, leaves, and fruit;
- isolated-point measurements;
- nearest-neighbor spacing;
- occupied voxel count at a common evaluation scale;
- background-noise assessment;
- explicit checks for missing thin structures.

---

## Offline Filter Comparison

The saved scan directory was:

`~/dev/tree_scans/indoor_model_tree_1`

The filter comparison needed to begin from the same saved captures and fused reconstruction inputs. This avoids changing the physical robot trajectory, camera observations, and tree pose between filter trials.

The four tested profiles were:

1. no outlier removal;
2. the current radius filter;
3. Dawood's radius filter;
4. statistical outlier removal.

The original comparison at the saved 6 mm local and 5 mm global voxel settings produced:

| Profile | Captures accepted | Final points | Mean ICP fitness | Mean ICP RMSE | Processing time |
|---|---:|---:|---:|---:|---:|
| None | 32/32 | 59,065 | 0.91967 | 3.755 mm | 6.97 s |
| Current radius | 32/32 | 58,401 | 0.91934 | 3.754 mm | 6.26 s |
| Dawood radius | 32/32 | 50,903 | 0.91784 | 3.905 mm | 5.26 s |
| Statistical | 32/32 | 59,367 | 0.92061 | 3.731 mm | 6.22 s |

Interpretation:

- every profile successfully reconstructed all 32 captures;
- the statistical profile had the best numerical fitness and RMSE, although the differences were small;
- Dawood's radius settings were substantially more aggressive;
- Dawood's profile retained much less geometry and was visually ruled out;
- the unfiltered profile was also visually ruled out because of its noise;
- the current-radius and statistical profiles were the strongest candidates.

The current-radius result appeared to preserve slightly more useful leaf quality, even though the statistical result had marginally better ICP statistics.

---

## Meaning of the `none` Profile

The `none` profile is useful for checking whether final outlier removal deleted a leaf or branch.

It still performs:

- target cropping;
- local voxel downsampling;
- ICP registration when `use_icp=True`;
- global voxel downsampling.

It skips only the configured radius or statistical outlier-removal stage.

The diagnostic interpretation is:

- present in `none` but absent in a filtered result: the outlier filter probably removed the structure;
- absent in `none` as well: the structure was likely lost during sensing, target cropping, voxelization, registration, or viewpoint coverage.

The `--no-icp` option is a separate experiment. It compares robot-TF-only fusion against robot TF plus ICP correction; it should not be used to represent “no filtering.”

---

## Initial Voxel Sweep Was Invalid

The reconstruction configuration contained:

- `local_voxel_size_m: 0.006`;
- `global_voxel_size_m: 0.005`;
- radius outlier removal with a 0.020 m radius and six minimum neighbors;
- statistical alternatives of 20 neighbors and a 2.0 standard-deviation ratio.

An initial attempt changed `params.yaml` to 4 mm local and 4 mm global and reran `compare_filters.py`.

The output had the exact same:

- final point counts;
- ICP fitness values;
- ICP RMSE values;
- visual appearance.

Only processing time varied slightly.

This showed that `compare_filters.py` was not reading the live `params.yaml`.

---

## `compare_filters.py` Parameter Source

The script loads the acquisition-time configuration from:

`<scan_directory>/run_config.json`

Its effective logic was:

`run_config = load_json(run_config_path)`

`parameters = run_config["parameters"]`

It then obtained voxel sizes from the saved dictionary:

`parameters.get("local_voxel_size_m", 0.006)`

`parameters.get("global_voxel_size_m", 0.005)`

The literal values `0.006` and `0.005` are only fallbacks. Changing those defaults does nothing when `run_config.json` already contains both keys.

The original `run_config.json` should not be overwritten because it records the actual parameters used during acquisition.

Instead, explicit command-line overrides were added for:

- `--local-voxel-size-m`;
- `--global-voxel-size-m`.

The script copies the saved parameter dictionary and applies the experiment overrides to the copy.

This keeps the source scan configuration intact while making each offline reconstruction experiment reproducible.

---

## Output-Directory Naming Bug

The comparison output directory was modified to include the effective voxel settings.

The first attempted f-string was invalid because it used double quotes for both the outer f-string and the dictionary keys:

`f"...{parameters["local_voxel_size_m"]}..."`

The inner quotes prematurely ended the f-string.

There was also a logic error: the directory name was generated before the command-line overrides were applied, so the label would have shown the original saved voxel values rather than the effective experiment values.

The corrected order was:

1. load `run_config.json`;
2. copy `run_config["parameters"]`;
3. apply local and global voxel overrides;
4. convert both effective sizes to millimeters;
5. construct the descriptive output directory;
6. run the reconstruction profiles.

A valid directory name became:

`20260726_170000_voxel_4mm_local_4mm_global`

The earlier timestamped run that was thought to represent 4 mm/4 mm was recognized as another 6 mm local/5 mm global reconstruction and should not be reported as a valid voxel experiment.

---

## Valid 4 mm and 3 mm Results

After the parameter override issue was corrected, genuine voxel comparisons were run from the same 32 saved captures.

For the current-radius profile:

| Metric | 4 mm local/global | 3 mm local/global | Change |
|---|---:|---:|---:|
| Final points | 119,575 | 177,237 | +48.2% |
| Mean ICP fitness | 0.9276 | 0.9323 | modest improvement |
| Mean ICP RMSE | 3.336 mm | 3.033 mm | 9.1% lower |
| Processing time | 8.31 s | 13.33 s | 60.4% longer |

At 3 mm:

- the statistical result achieved an ICP RMSE of approximately 2.996 mm;
- the current-radius result achieved approximately 3.033 mm;
- the numerical difference was very small;
- the current-radius version appeared to retain slightly better fine plant structure;
- Dawood's radius and the unfiltered result were visually rejected;
- current-radius and statistical filtering remained the two useful candidates.

The working reconstruction choice became:

- 3 mm local voxel size;
- 3 mm global voxel size;
- current-radius filtering;
- 0.020 m radius;
- six minimum neighbors.

---

## Recommended Limit on Further Voxel Reduction

The 3 mm result represented a meaningful improvement over 4 mm, but further reduction was not automatically justified.

A limited 3 mm local/2 mm global experiment could test whether a denser final export improves visualization while keeping ICP at the proven 3 mm local resolution.

Reducing the local ICP voxel below 3 mm was deferred because:

- it would increase processing cost;
- it might mostly preserve RealSense noise;
- current missing geometry was more clearly associated with viewpoint and height coverage;
- the high fruit was physically outside the existing scan coverage.

The next major improvement was therefore expected from new viewpoints rather than only from higher point density.

---

## Fixed Analysis Resolution

A common analysis resolution was recommended for fair quantitative comparisons between scans reconstructed at different native voxel sizes.

For example, every final cloud can be copied and temporarily downsampled to:

`analysis_cloud = cloud.voxel_down_sample(voxel_size=0.005)`

This 5 mm evaluation grid was not a MeshLab viewer setting and had not previously been selected in the interface.

Its purpose is to ensure that:

- a 3 mm reconstruction does not win a metric only because it contains more samples;
- the original and expanded trajectories can be compared at the same spatial sampling scale;
- occupied-voxel and nearest-neighbor comparisons reflect geometry rather than only export density.

The native 3 mm files remain appropriate for visual inspection. The temporary 5 mm copies are only for standardized evaluation.

---

## Suggested Evaluation Metrics

For each reconstruction profile or trajectory, the following metrics were identified:

- input and final point counts;
- point-retention percentage where the filtering stages are directly comparable;
- processing time;
- mean and median ICP fitness;
- mean and median ICP RMSE;
- median nearest-neighbor distance;
- 95th-percentile nearest-neighbor distance;
- isolated-point percentage;
- occupied voxel count at a fixed evaluation resolution;
- cloud bounding-box dimensions;
- capture acceptance rate;
- valid burst-pixel percentage.

Metrics should ideally be computed both:

- over the complete cloud;
- inside a manually defined tree-only crop.

Otherwise, a profile could appear to preserve more geometry merely because it retained more wall, floor, or background points.

---

## PLY Color in MeshLab

The PLY files appeared colored in RViz2 but initially appeared uncolored in MeshLab.

RViz can apply a synthetic display color even when the underlying cloud does not store camera RGB values. Relevant RViz color transformers include:

- `AxisColor`;
- `FlatColor`;
- `Intensity`;
- `RGB8`.

Only `RGB8` indicates that the published point cloud includes actual RGB data.

The PLY header can be checked for:

- `property uchar red`;
- `property uchar green`;
- `property uchar blue`.

If those properties exist, MeshLab should be configured to display per-vertex colors.

If they do not exist, the reconstruction/export path is saving geometry only. Real color requires:

- extracting the aligned RGB pixel for every valid aligned-depth pixel;
- converting BGR input to RGB when required;
- attaching normalized colors to the Open3D point cloud;
- preserving the point–color correspondence through all downsampling and filtering stages;
- exporting the colored Open3D cloud.

Open3D index-selection operations should be preferred over rebuilding a new cloud from only XYZ coordinates, because rebuilding only the point array discards the associated colors.

Voxel size and outlier-filter choice do not themselves determine whether RGB is stored in the PLY.

---

## Coverage Became the Main Reconstruction Limitation

The top fruit was only partially observed because the existing raster did not extend high enough.

The scan also remained predominantly frontal:

- lateral positions moved left and right;
- camera orientation changed modestly;
- the available perspective was not equivalent to a true side pass around the canopy.

The highest-value trajectory additions were identified as:

1. one additional row above the existing top row;
2. stronger left- and right-oblique views;
3. selected views into the canopy interior;
4. lower/upward-looking views if the undersides remain missing;
5. a center pass only when it adds a new angle rather than duplicating the same frontal observation.

The existing ±0.15 m lateral positions already create a 0.30 m baseline. Increasing the lateral reach to ±0.20 m could later add translation-based parallax, but it would also change arm reach and canopy-clearance risk.

---

## Recorded Trajectory Versus Trajectory Generator

The existing `scanning_trajectory.cpp` file is a replay engine.

It does not encode the Cartesian row positions. It loads the previously recorded:

`recorded_scan_trajectory.json`

Therefore, editing the replay engine cannot safely raise the camera by one row.

The source that generates the MoveIt poses is:

`scanning_trajectory_node.cpp`

The generator is the appropriate place to:

- define an additional row;
- change row spacing;
- change lateral offset;
- change the camera look angle;
- ask MoveIt to plan and validate each motion;
- record the resulting time-parameterized joint trajectories.

Manually inserting a single endpoint from `/joint_states` into the JSON would be unsafe and incomplete because each segment contains a time-parameterized series of positions, velocities, accelerations, and timestamps rather than only one destination joint pose.

---

## Four-Row Trajectory Revision

The trajectory generator was revised to expose:

- `row_count`;
- `row_spacing_m`;
- `lateral_offset_m`;
- `look_angle_deg`.

The planned values were:

- four total rows;
- 0.15 m vertical spacing;
- ±0.15 m lateral offset;
- 15-degree look angle;
- 0.05 velocity scaling;
- 0.05 acceleration scaling.

The fourth row adds a Row 3 sequence at:

`home_z + 0.45 m`

This is one additional 0.15 m increment above the prior Row 2.

The revised default sequence contains 42 waypoints:

- the original 32;
- 10 additional Row 3 waypoints.

The 15-degree look angle gives somewhat stronger oblique aiming than the earlier 10-degree value. It does not replace translation-based side views, but it adds angular diversity without immediately increasing lateral reach.

The lateral offset was intentionally kept at 0.15 m until the new high-row reach and branch clearance can be verified on hardware.

---

## Planned Hardware Validation

The new row should first be tested only through step 33, which reaches the new Row 3 center.

The cautious test parameters are:

- `record_trajectory:=false`;
- `row_count:=4`;
- `row_spacing_m:=0.15`;
- `lateral_offset_m:=0.15`;
- `look_angle_deg:=15.0`;
- `velocity_scaling:=0.05`;
- `acceleration_scaling:=0.05`;
- `max_steps:=33`.

After checking reach and physical clearance, the complete 42-step sequence can be recorded to:

`~/dev/dawood_tree_scanning_ur5e/recorded_scan_trajectory_4rows_15deg.json`

The replay run must use `max_steps:=42`; leaving it at 32 would silently truncate the new top row.

---

## Build Result and `No executable found`

The workspace rebuilt successfully with:

`colcon build --symlink-install`

All three packages finished:

- `arm_scan_motion`;
- `corn37_description`;
- `arm_scan_motion_moveit_config`.

The build printed stale `AMENT_PREFIX_PATH` and `CMAKE_PREFIX_PATH` warnings for old install directories. These warnings did not prevent the packages from compiling.

After sourcing `install/setup.bash`, the attempted command:

`ros2 run arm_scan_motion scanning_trajectory_node`

returned:

`No executable found`

The likely cause is that `scanning_trajectory_node.cpp` exists under `src/` but is not registered as a built and installed executable in `arm_scan_motion/CMakeLists.txt`.

Placing a C++ source file under `src/` does not automatically create a ROS 2 executable.

The package needs:

- an `add_executable(scanning_trajectory_node ...)` target;
- the required `ament_target_dependencies`;
- `scanning_trajectory_node` in an `install(TARGETS ...)` block;
- the corresponding `find_package(...)` declarations.

After updating CMake, the package should be rebuilt and verified with:

`ros2 pkg executables arm_scan_motion`

The expected result should include:

`arm_scan_motion scanning_trajectory_node`

Only then can the 33-step high-row clearance test be launched.

---

## ZED 2i Camera Consideration

A ZED 2i was considered as a possible future replacement for the current RealSense.

The camera could provide registered depth and colored point-cloud support under ROS 2 Humble, but changing cameras would introduce several major variables:

- a new physical mount;
- new camera-to-end-effector extrinsic calibration;
- ZED SDK and ROS wrapper installation;
- NVIDIA CUDA hardware requirements;
- topic remapping;
- different depth encodings and working-distance behavior;
- new timestamp, bandwidth, and depth-burst validation;
- complete revalidation of the reconstruction pipeline.

The previously inspected computer exposed Intel HD 530 graphics and could not directly satisfy the ZED SDK's NVIDIA GPU requirement. Processing would need to run on a compatible Jetson or another NVIDIA host.

Because the missing high fruit was clearly outside the current trajectory coverage, a camera replacement was not considered the most direct next step. The RealSense four-row scan should be evaluated first while holding the sensor and calibration constant.

The ZED 2i remains a possible later A/B camera experiment rather than an immediate reconstruction fix.

---

## End-of-Day Status

By the end of Sunday, July 26:

- the complete 32-step UR5e replay succeeded;
- all 32 capture requests were accepted;
- the arm returned home automatically;
- the earlier capture timeout appeared to be an isolated event;
- the integrated move–settle–capture workflow was validated;
- the online reconstruction produced 57,726 stored points;
- mean ICP fitness was approximately 0.920;
- mean ICP RMSE was approximately 3.76 mm;
- objective scan and registration metrics were collected;
- four offline outlier-filter profiles were compared from the same saved captures;
- Dawood's radius filter and the unfiltered profile were visually ruled out;
- statistical and current-radius filtering were the strongest candidates;
- current-radius filtering appeared to preserve fine plant structure slightly better;
- the first supposed 4 mm test was invalidated because `compare_filters.py` used `run_config.json`, not the edited `params.yaml`;
- explicit local/global voxel overrides were designed;
- output-directory naming was corrected to reflect effective voxel settings;
- a genuine 3 mm reconstruction increased current-radius point density by approximately 48% over 4 mm;
- current-radius mean ICP RMSE improved from approximately 3.336 mm to 3.033 mm;
- 3 mm local/global with current-radius filtering became the working choice;
- fixed-resolution evaluation was defined as an offline metric step rather than a viewer setting;
- `none` was established as the diagnostic profile for detecting geometry removed by final outlier filtering;
- missing top fruit and incomplete side coverage were identified as acquisition-trajectory problems;
- the MoveIt trajectory generator was revised for four rows and a 15-degree look angle;
- the proposed sequence increased from 32 to 42 waypoints;
- the workspace compiled, but the new generator was not yet exposed as a ROS 2 executable;
- `CMakeLists.txt` registration became the immediate blocker before hardware validation;
- switching to a ZED 2i was deferred until the improved RealSense trajectory can be tested.

---

## Next Steps

1. add `scanning_trajectory_node` to `arm_scan_motion/CMakeLists.txt`;
2. rebuild `arm_scan_motion` and verify the executable with `ros2 pkg executables`;
3. cautiously test only through step 33 at 5% velocity and acceleration scaling;
4. verify the new Row 3 center is reachable and clear of the tree, mount, robot, and surrounding hardware;
5. record the complete 42-step four-row trajectory only after the clearance test passes;
6. replay the 42-step trajectory with the reconstruction node and a new scan ID;
7. retain 3 mm local/global voxels and current-radius filtering for the first expanded scan;
8. compare the original 32-step and expanded 42-step scans at the same temporary 5 mm analysis resolution;
9. crop and compare the trunk, one branch, representative leaves, and the high fruit;
10. compute occupied-voxel, nearest-neighbor, isolated-point, bounding-box, capture-success, and ICP metrics;
11. confirm whether exported PLY files contain RGB properties and enable per-vertex colors in MeshLab when present;
12. if RGB is absent, preserve aligned camera colors throughout the Open3D reconstruction and export path;
13. consider a 3 mm local/2 mm global offline export test before reducing the local ICP voxel below 3 mm;
14. evaluate larger lateral offsets or dedicated oblique passes only after the four-row trajectory is safely validated;
15. keep the ZED 2i as a later controlled sensor comparison rather than changing cameras during the current coverage experiment.
