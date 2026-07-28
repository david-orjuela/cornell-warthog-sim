# 2026-07-27 — TF-Anchored Registration Evaluation and Orchard Preparation

## Day Objective

Monday was planned around taking the Warthog to the orchard because Tuesday and Wednesday were forecast for sustained rain. The Warthog base is weather-resistant, but the UR5e, RealSense mount, and supporting electronics are not. Light rain delayed the original departure, so the plan became to reassess the radar around 4 PM and aim for a late-day dry window near 5 PM.

While the robot was unavailable for testing, Dawood reported measurable drift across the accumulated point cloud and suggested replacing ICP with TEASER++. The technical objective therefore became:

1. determine whether the observed error was actually a pairwise-registration failure;
2. establish quantitative TF-only and TF+ICP baselines before changing algorithms;
3. test whether the current incremental ICP architecture was order-dependent;
4. evaluate a TF-anchored pose graph as a global-consistency alternative;
5. prepare the capture and trajectory behavior for less predictable orchard scenes.

---

## Registration Problem Definition

The existing reconstruction uses the robot TF pose to place each camera cloud in `base_link`, then refines that placement with coarse-to-fine point-to-plane ICP against the accumulated global cloud.

This creates two distinct potential error layers:

- **Pairwise alignment:** whether one new cloud can be aligned to overlapping geometry.
- **Global consistency:** whether many individually plausible corrections remain mutually consistent across the full scan.

Good ICP fitness and RMSE at each capture do not prove that the final trajectory is drift-free. Small corrections can accumulate in one direction, and registering against an evolving, voxelized global map can make the result depend on which frame was fused first.

Without external ground truth, absolute camera-pose error could not be measured directly. The evaluation therefore used internal consistency tests:

- deviation of ICP-corrected poses from calibrated TF;
- repeated-pose closure error;
- residuals on overlapping frame pairs excluded from graph optimization;
- point-to-plane error on those held-out pairs;
- sensitivity to chronological, reverse, and shuffled replay order;
- identical front, side, and top visualizations.

---

## Why TEASER++ Was Not the First Change

TEASER++ is valuable when a pair of point clouds has a poor initial transform and its proposed correspondences contain a very high proportion of outliers. It estimates a robust rigid transform from those correspondences, but does not generate them; a feature-matching frontend such as FPFH is still required, and ICP is commonly used afterward for local refinement.

That did not match the main evidence in this pipeline:

- every capture already had a physically meaningful TF initialization from the UR5e;
- adjacent raster views generally had substantial overlap;
- the existing ICP usually reported high fitness and millimeter-scale RMSE;
- the suspected drift appeared across the complete fused sequence, not as one clearly unregistrable pair.

Replacing ICP with TEASER++ at every capture would still produce a sequence of pairwise transforms without enforcing global consistency. It would therefore attack the pairwise solver while leaving the accumulated-map architecture and pose-consistency problem intact.

The working decision became:

- retain TF as the primary pose source;
- use robust local ICP to estimate selected relative constraints;
- optimize all camera poses together in a TF-anchored pose graph;
- reserve TEASER++ for a specific overlapping pair or loop closure that repeatedly fails despite verified calibration, sufficient overlap, and a reasonable TF initialization.

---

## Candidate Registration Methods Considered

Candidate methods were compared against the failure indicated by the data rather than only their pairwise alignment performance.

| Candidate | Potential benefit | Limitation for this pipeline | Decision |
|---|---|---|---|
| Point-to-point ICP | Simple local refinement without requiring surface normals | Less suited to locally planar surfaces and still produces only one incremental correction | Not preferred |
| Point-to-plane ICP | Efficient, accurate local refinement when TF already provides a close initial pose | Can converge to a locally plausible but globally inconsistent foliage alignment | Retain for local edge estimation |
| Generalized ICP | Models local surface covariance and may improve difficult pairwise alignments | Does not remove accumulated-map feedback or allow earlier poses to be revised | Possible later pairwise comparison |
| FPFH + RANSAC | Provides global pairwise initialization when relative pose is uncertain | Feature matches can be ambiguous on repetitive, thin foliage; unnecessary for normal adjacent views with good TF | Reserve for uncertain loop closures |
| Fast Global Registration | Faster feature-based global pairwise alignment | Still depends on descriptor quality and does not enforce multi-frame consistency | Not the first change |
| TEASER++ | Robustly estimates a pairwise transform despite many false proposed correspondences | Requires a correspondence frontend and would preserve the same sequential fusion problem if substituted directly for ICP | Reserve for verified pairwise failures |
| Colored ICP | Adds RGB consistency to geometry | Sensitive to lighting, view-dependent appearance, and incomplete color fidelity; remains a local solver | Not prioritized |
| TF-anchored pose graph | Preserves each capture, combines many relative constraints, allows global revision, and delays fusion | Requires careful edge selection and prior weighting | Selected architecture |

The key distinction was between improving an individual pairwise transform and improving the consistency of the complete scan. ICP variants, RANSAC, Fast Global Registration, and TEASER++ could change how one edge is estimated, but none would by itself correct the one-way accumulation of locally accepted poses. A pose graph directly tests that architectural hypothesis while continuing to use the existing TF and ICP information.

The resulting scientific progression was:

1. verify the UR calibration and camera-to-tool transform;
2. establish TF-only as the control condition;
3. quantify scan-order sensitivity in the existing incremental TF+ICP method;
4. retain raw captures and construct robust relative ICP constraints from TF initialization;
5. optimize all poses together with TF priors, then fuse once;
6. compare all methods on identical captures using held-out constraints and fixed views;
7. introduce TEASER++ only if particular overlapping pairs cannot be registered reliably from TF.

---

## Controlled TF-Only Versus TF+ICP Baseline

The same saved 32-capture dataset was reconstructed offline so that the robot trajectory, RGB-D observations, crop, filter, and voxel settings remained fixed.

`compare_filters.py` already supported the required first comparison:

- `tf_only`: fuse each saved cloud exactly at its recorded TF pose;
- `tf_icp`: use TF initialization followed by the current incremental frame-to-accumulated-cloud ICP.

At 3 mm local/global voxels, both statistical and current-radius profiles accepted all 32 captures.

| Profile | Mode | Accepted | Final points | Mean fitness | Mean RMSE |
|---|---|---:|---:|---:|---:|
| Statistical | TF only | 32/32 | 190,029 | — | — |
| Statistical | TF + ICP | 32/32 | 200,077 | 0.932 | 2.996 mm |
| Current radius | TF only | 32/32 | 184,647 | — | — |
| Current radius | TF + ICP | 32/32 | 177,237 | 0.932 | 3.033 mm |

The similar acceptance rates and strong ICP statistics showed that ordinary capture metrics alone could not determine whether ICP was helping or introducing drift. Point count was also not a quality metric because it changed in opposite directions between the two filter profiles.

---

## TF-Anchored Pose-Graph Experiment

A companion `compare_registration.py` experiment was created to evaluate the registration architecture directly.

The experiment:

- rebuilt TF-only and chronological TF+ICP baselines;
- replayed incremental ICP in chronological, reverse, and deterministic shuffled orders;
- estimated robust pairwise ICP constraints from relative TF initialization;
- added adjacent, overlapping, and repeated-pose closure edges;
- held six overlap edges out of optimization for independent evaluation;
- added an absolute TF prior for every camera node and anchored the graph to `base_link`;
- optimized all poses in one batch;
- fused the individually retained clouds once after optimization;
- generated RGB and capture-ID fixed views.

The nominal TF prior represented 10 mm translation and 1° rotation uncertainty. It was an experimental weighting, not a measured calibration covariance.

The graph attempted 81 pairwise edges:

- 62 were accepted for optimization;
- 13 were rejected by the robust registration checks;
- 6 were held out for evaluation.

An initial output-writing issue was corrected, after which the complete nominal-prior experiment produced the comparison artifacts.

---

## Evidence of Incremental ICP Drift

The chronological incremental result looked successful by normal ICP metrics:

- all 32 captures were accepted;
- mean fitness was approximately 0.927;
- mean RMSE was approximately 3.32 mm.

However, its corrected poses increasingly departed from the calibrated TF trajectory:

| Metric | Chronological TF + ICP |
|---|---:|
| Mean translation deviation from TF | 5.30 mm |
| 95th-percentile translation deviation | 11.02 mm |
| Maximum translation deviation | 13.46 mm |
| Mean rotation deviation | 1.01° |
| Maximum rotation deviation | 1.94° |
| Translation-deviation slope | 0.264 mm/frame |
| Correlation with frame index | 0.762 |

The positive slope and strong correlation with frame index were direct evidence of systematic accumulation rather than independent zero-mean refinement.

The scan-order test provided stronger causal evidence. Only the replay order changed:

| Replay order | Accepted | Mean pose difference from chronological | Maximum pose difference | Mean rotation difference |
|---|---:|---:|---:|---:|
| Reverse | 31/32 | 13.44 mm | 18.77 mm | 1.92° |
| Shuffled | 28/32 | 13.33 mm | 19.82 mm | 2.00° |

The reverse and shuffled results also rejected frames that chronological replay accepted. This confirmed that accumulated-map ICP was materially order-dependent: the changing target map affected both the estimated poses and whether later frames passed registration.

---

## Pose-Graph Results

The nominal TF-anchored pose graph kept corrections small and removed the frame-wise drift trend:

| Metric | TF only | Incremental TF + ICP | Nominal pose graph |
|---|---:|---:|---:|
| Mean deviation from TF | approximately 0 | 5.30 mm / 1.01° | 1.99 mm / 0.108° |
| Maximum deviation from TF | approximately 0 | 13.46 mm / 1.94° | 4.05 mm / 0.198° |
| Deviation slope | approximately 0 | 0.264 mm/frame | approximately 0 |
| Mean repeated-pose closure | 0.020 mm / 0.002° | 0.918 mm / 0.096° | 0.633 mm / 0.046° |
| Mean held-out pose residual | 3.05 mm / 0.684° | 4.17 mm / 0.686° | 2.91 mm / 0.607° |
| Held-out point-to-plane RMSE | 4.30 mm | 4.13 mm | 4.20 mm |

Interpretation:

- incremental ICP achieved the lowest held-out geometric RMSE by a small margin;
- the pose graph achieved the best held-out pose consistency;
- the graph improved repeated-pose closure relative to incremental ICP;
- the graph avoided the increasing deviation from TF;
- TF-only remained a strong baseline and should not be discarded.

The pose graph was therefore the most promising architecture for controlling drift, but the first result was not a definitive visual-quality winner. It traded a very small increase in held-out surface error relative to incremental ICP for substantially better pose consistency and much smaller TF departures.

This supported the conclusion that TEASER++ would have targeted the wrong layer as the immediate replacement. The more important change was moving from sequential accumulated-map correction to globally optimized, TF-anchored poses.

---

## Visual Evaluation Strategy

The identical full-cloud front, side, and top views showed only subtle differences among TF-only, incremental ICP, and the nominal graph. Capture-ID coloring helped expose how observations from different poses overlapped, but the complete tree remained too dense for reliable judgments about leaves, branches, and apples.

The next visual evaluation should use identical spatial crops for every method:

- a trunk section;
- one representative branch junction;
- two apples at different locations;
- representative thin leaves or leaf edges.

Each crop must use the same base-frame bounds, camera orientation, point size, and zoom across all variants.

Thin cross-sections provide a more objective view of local alignment. A narrow slab through a trunk, branch, or apple can reveal:

- duplicated surfaces;
- shell thickness;
- ghosting between captures;
- gaps or missing observations;
- whether different capture IDs form one surface or several displaced layers.

Useful local measurements include cross-section thickness, fitted-plane or fitted-cylinder residuals, nearest-neighbor spacing, and capture-to-capture spread. These tests are more sensitive to drift than judging the full canopy silhouette.

---

## Calibration Interpretation

The two supplied UR5e calibration YAML files were identical, including the kinematic values and calibration hash. This supported Dawood's statement that the robot factory calibration file in use was current.

That does not verify the RealSense-to-tool transform. The robot kinematic calibration and the camera-to-tool extrinsic are separate components of the TF chain.

A later calibration check should:

1. verify that the static camera transform matches the physical mount orientation and location;
2. confirm that the mount is rigid and has not shifted;
3. perform eye-in-hand calibration from multiple diverse arm poses using a fixed calibration target;
4. validate the result on held-out poses or repeated views rather than only reporting the solver residual.

The small pairwise corrections and strong TF-only baseline suggested that the current TF chain was broadly reasonable, but they did not prove that the camera extrinsic was optimal.

---

## Acquisition Robustness for a Different Tree

Before field deployment, the indoor test tree was changed. Capture 2 then failed under the original final ICP threshold:

- fitness: 0.478, below the required 0.500;
- RMSE: 5.5 mm, within the 8 mm limit;
- valid burst pixels: approximately 78%.

This was a borderline overlap failure, not a catastrophic geometric mismatch. The matched region aligned well, but the new tree position, sparser structure, background dormant trees, or changed crop reduced the fraction of corresponding points.

The result showed that a single hard rejection was too brittle for orchard use, where tree geometry, sunlight, wind, background, and usable depth will vary more than in the lab.

---

## Capture Retry and Rejection Policy

The reconstructor was revised so that one motion pose can request up to three fresh five-frame RGB-D bursts.

The new policy:

- keeps the arm stationary during retries;
- waits 0.25 s before the next attempt;
- never merges a rejected attempt;
- saves each attempt separately for later fitness/RMSE inspection;
- immediately accepts the pose if any attempt succeeds;
- publishes a final negative result only after all three attempts fail.

The final `SN0` fitness threshold was reduced from 0.50 to 0.45 while retaining:

- the 8 mm RMSE ceiling;
- transform-correction sanity bounds;
- stricter rejection at earlier coarse scales.

This allows a result such as 0.478 fitness with 5.5 mm RMSE to remain usable without accepting arbitrary low-quality registrations.

---

## Retry Validation and Timeout Discovery

The revised system performed substantially better on the changed indoor scene:

- captures 1–9 were accepted;
- capture 2 passed with 0.546 fitness and 5.6 mm RMSE;
- the global cloud reached 95,673 points before capture 10;
- rejected clouds were confirmed to remain unmerged.

Capture 10 exercised the complete retry path. All three attempts failed at the coarse `SN3` stage because RMSE remained just above the 25 mm limit:

| Attempt | Fitness | RMSE | Result |
|---|---:|---:|---|
| 1 | 0.609 | 27.1 mm | rejected |
| 2 | 0.593 | 25.4 mm | rejected |
| 3 | 0.591 | 26.1 mm | rejected |

The reconstructor correctly withheld all three clouds and eventually published `result=-10`. However, the trajectory node stopped at its 20 s capture timeout before that final result arrived. The retry response completed approximately 22.8 s after the request.

This separated two policies that need independent configuration:

- `abort_on_capture_rejection: false` allows an explicit rejected pose to be skipped;
- `capture_timeout_sec` controls how long the motion node waits for any result.

For field use, the planned settings became:

- do not abort after one exhausted registration request;
- continue without merging the rejected cloud;
- abort only after repeated consecutive failures or a fatal TF/data/controller fault;
- increase the capture-result timeout to approximately 60 s so all retries can complete.

Periodic approximately 15 s delays before some burst attempts also remained to be diagnosed.

---

## Original 32-Step and New 42-Step Trajectories

The trajectory files and generator behavior were clarified before testing the new high row.

- `recorded_scan_trajectory.json` contained the original 32-step scan.
- The revised generator can create a 42-step four-row trajectory with 0.15 m row spacing, ±0.15 m lateral positions, and a 15° look angle.
- `max_steps` only truncates the selected trajectory; it does not convert a 32-step file into the new scan or make the first 32 steps of the new file identical to the old scan.
- `record_trajectory:=false` still executes the generated motion; it only disables JSON output.
- `record_trajectory:=true` executes the motion and saves the time-parameterized segments.

The intended sequence remained:

1. use the generator through step 33 at 5% speed and acceleration to reach and inspect the new high-row center;
2. after physical clearance is confirmed, execute and record all 42 steps to a separately named JSON;
3. replay the exact 32-step or 42-step file explicitly rather than relying on a generic filename.

The attempted generator commands initially produced `UnknownROSArgsError` because the multiline shell backslashes were followed by spaces and `-p` on the same line. This was a command-formatting issue, not evidence that the 42-step node was broken.

---

## Simulation-First Trajectory Validation

A safer MoveIt preview workflow was defined for the new row.

The correct simulation backend is `arm_scan_motion_moveit_config demo.launch.py`, which provides:

- `move_group`;
- simulated joint states;
- fake trajectory controllers;
- RViz visualization.

RViz by itself is only a viewer and cannot execute the generator. The simulated session should use an isolated ROS domain with localhost-only discovery and must not run the real UR driver or `moveit_real.launch.py`.

Running the 33-step and later 42-step generator against the fake controllers can verify:

- target reachability;
- Cartesian planning completion;
- modeled self-collision and Warthog-arm collision checks;
- intended high-row direction;
- generation of the recorded JSON.

It cannot verify clearance from the real tree, camera cable, mount, people, unmodeled hardware, or calibration error. The physical test must therefore remain at 5% scaling with an operator ready at the stop controls.

---

## End-of-Day Status

By the end of Monday, July 27:

- the weather-dependent orchard deployment was reorganized around a late-day dry window;
- TEASER++ was evaluated conceptually and deferred as a blanket ICP replacement;
- TF-only and TF+ICP baselines were reconstructed from the same 32 saved captures;
- good ICP fitness/RMSE was shown not to be sufficient evidence of global consistency;
- incremental accumulated-map ICP showed increasing deviation from TF;
- reverse and shuffled replay changed estimated poses by approximately 13 mm on average and up to approximately 20 mm;
- the current ICP architecture was confirmed to be materially scan-order-dependent;
- a TF-anchored pose-graph experiment was implemented and completed;
- the nominal graph reduced mean TF deviation to approximately 2 mm and removed the frame-wise drift trend;
- the graph improved closure and held-out pose consistency relative to incremental ICP;
- full-cloud fixed views were generated, but local crops and thin cross-sections were identified as the more sensitive visual test;
- the robot calibration YAML files were confirmed identical, while camera-to-tool calibration remained a separate verification task;
- a changed-tree test exposed the brittleness of a single 0.50 fitness cutoff;
- three-attempt fresh-burst retry logic was implemented;
- rejected attempts were saved and never merged;
- the final fitness threshold was adjusted to 0.45 with the 8 mm RMSE and transform sanity checks retained;
- the revised pipeline accepted the first nine poses of a new indoor run;
- capture 10 correctly failed all retries without contaminating the global cloud;
- the trajectory node's 20 s result timeout was found to be shorter than the full retry cycle;
- nonfatal skip behavior and a longer field timeout were identified as the remaining policy changes;
- the original 32-step JSON and proposed 42-step trajectory were separated explicitly;
- the step-33 high-row clearance test and isolated MoveIt fake-controller preview were defined.

---

## Next Steps

1. set `abort_on_capture_rejection: false` for nonfatal registration failures;
2. increase `capture_timeout_sec` to approximately 60 s and confirm the trajectory waits through all retries;
3. define an abort rule based on consecutive failed poses rather than one rejected capture;
4. diagnose the periodic approximately 15 s delay before some RGB-D burst attempts;
5. generate fixed base-frame crops around the trunk, one branch junction, two apples, and thin leaf structures;
6. add narrow cross-section measurements for surface thickness, duplication, and capture-to-capture spread;
7. repeat the registration experiment with the current-radius profile to verify that the pose-graph conclusion is not filter-specific;
8. test loose, nominal, and strong TF priors using held-out edges rather than choosing the visually densest cloud;
9. verify the camera-to-tool static transform and plan a formal eye-in-hand calibration check;
10. use TEASER++ only if a verified overlapping loop-closure pair repeatedly fails robust TF-initialized ICP;
11. run the four-row generator in an isolated MoveIt fake-hardware session;
12. physically test only through step 33 at 5% speed with direct clearance supervision;
13. record the complete 42-step trajectory to a distinct JSON only after the high-row test passes;
14. compare the eventual 42-step scan with the 32-step baseline using the same fixed metrics and crops.
