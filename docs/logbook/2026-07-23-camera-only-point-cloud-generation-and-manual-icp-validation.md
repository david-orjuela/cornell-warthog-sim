# 2026-07-23 — Camera-Only Point Cloud Generation and Manual ICP Validation

## Day Objective

The primary goal for Thursday was to move beyond the previous day's RGB-D conversion failure and validate the complete camera-only point-cloud reconstruction pipeline.

The focus was:

1. confirm the Python/Open3D environment;
2. replace the failing Open3D RGB-D image conversion path;
3. generate a colored point cloud from the Jetson-mounted Intel RealSense camera;
4. save the result as a global PLY;
5. test repeated captures and coarse-to-fine ICP using small manual camera movements;
6. inspect reconstruction quality and identify the next tuning priorities.

No UR5e motion was attempted because the end-effector camera mount was broken and being reprinted.

---

## Python Environment Verification

The active Python environment was confirmed as:

- NumPy 1.26.4
- SciPy 1.8.0
- Open3D 0.19.0
- Open3D installed under `~/.local/lib/python3.10/site-packages`

SciPy continued to warn that it expected NumPy below 1.25.0. The environment is therefore internally inconsistent, but the warning did not prevent the node from running.

The earlier Open3D failure occurred even though the incoming depth image had valid properties:

- encoding: `16UC1`
- resolution: 640 × 480
- dtype: `uint16`
- little-endian
- contiguous memory
- depth scale: 1000.0

---

## Direct NumPy RGB-D Back-Projection

The failing Open3D path based on `o3d.geometry.Image` and `RGBDImage.create_from_color_and_depth()` was bypassed.

The replacement directly back-projects each valid depth pixel into 3D using the camera intrinsics:

- `X = (u - cx) Z / fx`
- `Y = (v - cy) Z / fy`
- `Z = depth`

The corresponding RGB pixel is assigned to each generated 3D point.

This preserved the rest of the existing pipeline:

- camera-frame or TF placement;
- local filtering;
- multiscale point-to-plane ICP;
- global fusion;
- ROS `PointCloud2` publication;
- per-frame image/depth/metadata export;
- continuously updated global PLY output.

This change resolved the RGB-D point-cloud creation blocker.

---

## First Successful End-to-End Capture

The reconstruction node was launched with motion disabled and wall-clock time:

`ros2 launch reconstruct scan_tree.launch.py execute_motion:=false use_sim_time:=false`

The first successful capture produced:

- accepted first frame;
- camera-only identity transform;
- approximately 18,479 fused points;
- reconstruction time of approximately 167 ms;
- a valid `global_point_cloud.ply`;
- a `/capture_done` response.

A previous test had produced only 58 points because the camera was unintentionally facing the floor. Once the camera was aimed at the tree, captures produced tens of thousands of points.

This confirmed the complete path:

`RGB + aligned depth + intrinsics → colored point cloud → filtering → fusion → PLY`

---

## Capture and File Behavior

The node maintains one in-memory global point cloud and continuously rewrites one file:

`~/tree_scans/indoor_model_tree/global_point_cloud.ply`

Accepted captures are transformed, optionally corrected by ICP, and added to `global_pc`.

Rejected ICP captures do not modify the global point cloud, although the existing global PLY may still be rewritten unchanged.

For captures that process far enough to reach `_fuse_frame()`, the node saves per-frame files regardless of final ICP acceptance:

- color image;
- depth image;
- metadata JSON.

The metadata records whether the frame was accepted and includes:

- ICP fitness;
- ICP RMSE;
- target and camera frames;
- camera intrinsics;
- TF transform;
- ICP correction.

Frames that fail before this stage, such as missing TF, failed image conversion, or zero generated points, are not saved.

---

## Near-Depth Behavior

Objects placed very close to the camera were missing from both the aligned depth image and resulting PLY, even after setting:

- minimum depth to 0.0 m;
- maximum depth to 5.0 m.

Backing the camera away caused the object to appear correctly.

The camera driver was inspected by SSHing into the Warthog computer and switching to `ROS_DOMAIN_ID=26`.

The RealSense node was:

`/sensors/camera_jetson`

Relevant driver settings included:

- `align_depth.enable: true`
- `depth_module.depth_format: Z16`
- `depth_module.depth_profile: 640,480,30`
- `clip_distance: -2.0`
- emitter enabled
- auto exposure enabled
- depth and color enabled
- camera TF publication enabled

Because there was no positive software clipping distance configured, the close-range gap was judged more likely to come from the physical stereo minimum range, invalid disparity at close distance, or aligned-depth occlusion behavior.

Since moving the camera farther back was an effective workaround, near-depth tuning was deprioritized.

---

## Manual Multi-View Scan

Because the printed camera mount was unavailable, the UR5e scan was approximated manually.

A repeated `/capture_alert` loop was used while the camera was moved by hand in small increments.

The intended movement strategy was:

- approximately 2–5 cm between captures;
- minimal rotation;
- high overlap between consecutive views;
- hold the camera still during each capture;
- stop with `Ctrl+C`.

This was a harder test than the eventual robot scan because camera-only mode supplied no motion prior. Every capture entered in the same camera frame, so ICP had to estimate the complete relative movement.

---

## Manual ICP Results

Seven captures were attempted.

Representative accepted results:

| Capture | Fitness | RMSE | Global points |
|---|---:|---:|---:|
| 1 | TF only | — | 18,479 |
| 2 | 0.831 | 5.8 mm | 27,776 |
| 3 | 0.823 | 5.1 mm | 35,576 |
| 6 | 0.922 | 4.5 mm | 44,734 |
| 7 | 0.824 | 4.4 mm | 55,044 |

Captures 4 and 5 were rejected at `SN3`:

- capture 4: fitness 0.509, RMSE 29.1 mm;
- capture 5: fitness 0.474, RMSE 30.4 mm.

Both exceeded the coarse-scale RMSE threshold of 25 mm.

The later recovery at capture 6 showed that the global map remained valid after rejected frames and that a sufficiently overlapping viewpoint could still register successfully.

The accepted fitness values around 0.82–0.92 and RMSE values around 4–6 mm indicate that coarse-to-fine point-to-plane ICP was functioning correctly under small manual viewpoint changes.

---

## Coarse-to-Fine ICP Confirmation

Every post-initial capture runs the full registration sequence:

`SN3 → SN2 → SN1 → SN0`

There is no separate fine scan that must be triggered later.

The final `SN0` stage uses no additional scale-specific voxel downsampling, but the input cloud has already been filtered using the configured local voxel size.

Therefore, `SN0` is full resolution relative to the filtered frame, not necessarily full raw camera resolution.

The global point cloud is also downsampled after fusion using the configured global voxel size.

---

## Initial Reconstruction Quality

The resulting point cloud was recognizable and substantially better than the earlier single-frame tests. The tree, canopy, and several apples could be identified.

However, the cloud remained visually limited:

- grainy point-based appearance;
- significant room and whiteboard background geometry;
- floating clusters;
- incomplete side and rear coverage;
- poor appearance from oblique viewpoints;
- dull color;
- incomplete thin branches, leaves, and apple surfaces.

The PLY is currently a colored point cloud, not a triangle mesh. It therefore lacks:

- continuous surfaces;
- watertight geometry;
- texture maps;
- simulator-ready collision geometry.

A later processing stage will be required for cropping, denoising, surface reconstruction, mesh cleanup, and simulation import.

---

## Interpretation of the Current Quality

Good ICP metrics do not guarantee a simulator-ready reconstruction.

The current limitations are primarily caused by:

- limited viewpoint coverage;
- manual camera motion;
- the tree being observed mainly from the front;
- background geometry being included;
- RealSense depth noise;
- incomplete sensing of thin and reflective structures;
- global voxel downsampling;
- lack of post-processing or meshing.

The poor side view is expected because frontal captures mainly create a partial front-facing shell. ICP can align observed surfaces but cannot reconstruct unseen sides or the rear of the tree.

A shallow arc around the tree will be more valuable for 3D completeness than repeatedly capturing nearly identical frontal images.

---

## Parameter Tuning Priorities

### Depth cropping

The most important immediate improvement is to reduce the retained depth range around the tree.

A 5 m maximum includes much of the laboratory and allows background geometry to influence both appearance and registration.

A target-relative range such as approximately 0.35–2.0 m was suggested as a starting point, depending on measured camera-to-tree distance.

### Voxel resolution

The current filtered and global clouds use millimeter-scale voxels.

A finer experimental configuration was suggested:

- local voxel size: approximately 4 mm;
- global voxel size: approximately 3 mm.

This may preserve more apple, branch, and leaf detail, at the cost of higher point counts and runtime.

### Outlier rejection

Disabling outlier rejection was identified as a useful comparison test, but it was expected to retain more floating depth noise.

A less aggressive radius filter may better preserve thin structures:

- radius around 15 mm;
- minimum neighbors around 4.

The goal is not simply to maximize point count, but to preserve true thin geometry without keeping isolated noise.

### ICP thresholds

The ICP thresholds were not loosened.

Captures 4 and 5 likely represented poor overlap or excessive manual movement, and accepting them could introduce duplicated or blurred geometry.

The current thresholds appeared to reject bad views while preserving later recovery.

### Final cleanup

Statistical outlier removal was identified as a possible final post-processing step after the complete scan rather than after every capture.

A tight spatial crop around the tree is expected to improve visual quality more than aggressive ICP changes.

---

## Color Quality

The dull point-cloud color was traced primarily to the source RGB frames and viewing conditions.

The frames showed:

- strong backlighting;
- dark foliage;
- low contrast;
- softness and occasional motion blur;
- bright room background.

There was no obvious RGB/BGR channel reversal because green foliage and warm apple colors remained plausible.

Potential later improvements include:

- keeping the camera stationary during capture;
- reducing backlighting;
- adjusting RealSense exposure, gain, white balance, sharpness, or saturation;
- changing point-viewer rendering settings;
- delaying color enhancement until geometry is validated.

---

## End-of-Day Status

By the end of Thursday, July 23:

- the direct NumPy back-projection workaround was implemented successfully;
- camera-only RGB-D point-cloud generation worked end to end;
- valid colored PLY files were generated;
- `/capture_done` worked;
- single captures produced tens of thousands of points;
- repeated captures updated one global PLY;
- per-frame color, depth, and metadata files were saved;
- manual multi-view captures successfully exercised coarse-to-fine point-to-plane ICP;
- accepted registrations achieved fitness around 0.82–0.92 and RMSE around 4–6 mm;
- poor-overlap frames were rejected at the coarse scale;
- the global cloud remained valid after rejected frames;
- the tree and apples were recognizable;
- background clutter, limited coverage, depth noise, and point-cloud sparsity remained;
- no UR5e motion was attempted because the camera mount was being reprinted;
- the result was not yet a simulator-ready mesh.

---

## Next Steps

1. rerun the scan with a tighter tree-centered depth range;
2. compare outlier rejection enabled versus disabled;
3. test finer local and global voxel sizes;
4. add point-count and depth-range diagnostics for controlled comparisons;
5. perform a denser manual or robotic scan over a shallow horizontal and vertical arc;
6. restore `target_frame: base_link` when the robot and camera TF chain are online;
7. verify camera-to-tool extrinsic calibration before relying on robot motion;
8. run the move–settle–capture workflow using the UR5e;
9. crop the fused cloud tightly around the tree;
10. evaluate apples, leaves, branches, and trunk in fixed close-up views and cross-sections;
11. add final statistical denoising;
12. investigate TSDF, Poisson, or ball-pivoting reconstruction for conversion into a simulation-usable mesh.
