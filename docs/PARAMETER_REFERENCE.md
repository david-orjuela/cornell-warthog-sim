# Parameter Reference

This document describes the final summer 2026 configuration supplied with the
repository. It distinguishes:

- the effective values in `src/reconstruct/config/params.yaml`;
- the Python reconstructor's internal defaults;
- parameters consumed by the external `arm_scan_motion` executables;
- command-line options for the two offline analysis scripts.

For a completed dataset, `run_config.json` is the authoritative record of the
parameters actually used. Editing the repository YAML after acquisition does
not change an existing dataset.

## Launch arguments

`src/reconstruct/launch/scan_tree.launch.py` declares:

| Argument | Default | Meaning |
| --- | --- | --- |
| `params_file` | Installed `share/reconstruct/config/params.yaml` | Parameter file passed to the reconstructor |
| `use_sim_time` | `false` | Overrides the node's clock setting |

The launch file starts only `tree_point_cloud_reconstructor`. It does not have
an `execute_motion` argument.

## `tree_point_cloud_reconstructor`

### Clock and input topics

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `use_sim_time` | `false` | Use wall time on physical hardware |
| `color_topic` | `/sensors/camera_jetson/color/image_raw` | UR5e-mounted RealSense color |
| `depth_topic` | `/sensors/camera_jetson/aligned_depth_to_color/image_raw` | Depth aligned to color |
| `camera_info_topic` | `/sensors/camera_jetson/aligned_depth_to_color/camera_info` | Intrinsics matching aligned depth |
| `joint_state_topic` | `/joint_states` | Snapshot source saved with each frame |

`camera_jetson` is the eye-in-hand camera. `camera_0` is the Warthog-front
camera and is not the validated reconstruction input.

### Capture and output topics

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `capture_request_topic` | `/tree_scan/capture_request` | `UInt32` pose/request ID |
| `capture_result_topic` | `/tree_scan/capture_result` | `Int32`; `+N` accepted, `-N` rejected |
| `scan_finished_topic` | `/tree_scan/scan_finished` | `Bool` completion notice |
| `output_cloud_topic` | `/tree_scan/global_cloud` | Accumulated accepted cloud |
| `original_cloud_topic` | `/tree_scan/latest_tf_cloud` | Latest cropped TF-placed cloud before ICP |

### Frames and output paths

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `target_frame` | `base_link` | Fixed frame for moving-arm reconstruction |
| `camera_frame_override` | empty | Use the depth/camera-info message frame |
| `output_root` | `~/dev/tree_scans` | Root directory for generated datasets |
| `scan_id` | `example_scan` | Dataset directory name; change for every run |
| `capture_on_start` | `false` | Wait for a numbered request rather than capturing immediately |

Do not commit a field dataset name such as `outdoor_tree_6` as the reusable
default. A repeated `scan_id` mixes attempts from different runs.

### RGB-D synchronization and timing

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `sync_queue_size` | 30 | Approximate-time synchronizer queue |
| `sync_slop_sec` | 0.08 s | Maximum synchronization slop |
| `tf_timeout_sec` | 0.75 s | Exact-time TF lookup timeout |
| `post_request_guard_sec` | 0.0 s | Additional age guard after the request |
| `sensor_clock_warning_sec` | 0.25 s | Warn when sensor stamp and local clock differ by more than this |

The code always rejects synchronized frames at or before the pose-specific
request time. The clock warning includes transport latency and is diagnostic,
not an automatic rejection.

### Depth burst and retry policy

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `depth_burst_size` | 5 | Fresh synchronized samples per attempt |
| `depth_burst_min_valid_samples` | 2 | Minimum nonzero observations required per pixel |
| `capture_max_attempts` | 3 | Fresh bursts allowed at one pose |
| `capture_retry_delay_sec` | 0.25 s | Delay before collecting the next burst |
| `burst_pose_translation_tolerance_m` | 0.005 m | Maximum translation spread inside a burst |
| `burst_pose_rotation_tolerance_deg` | 0.5° | Maximum rotation spread inside a burst |

The median ignores zero/invalid depth. A rejected attempt is saved but never
merged. A negative result is published only after all attempts fail.

### Depth range and units

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `min_depth_m` | 0.30 m | Conservative D435 near-range guard |
| `max_depth_m` | 3.50 m | Broad final field range |
| `depth_scale_override` | 0.0 | Automatic: 1000 units/m for `16UC1`, 1 for `32FC1` |

The first five orchard datasets used `max_depth_m: 2.50`. `outdoor_tree_6`
used 3.50 m. The range limits are sensor guards; they are not a tree-specific
scene crop.

### Target-relative crop

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `target_crop_enabled` | `false` | Disable the fixed base-frame crop |
| `target_center_mode` | `camera_forward` | `camera_forward` or `fixed_target_frame` |
| `target_distance_m` | 1.0 m | First-pose optical-axis distance used by `camera_forward` |
| `target_center_target_frame_m` | `[0.0, 0.0, 0.0]` | Explicit center for `fixed_target_frame` |
| `target_box_size_m` | `[1.0, 1.0, 1.5]` | Base-frame X/Y/Z box dimensions |

In `camera_forward` mode, the first accepted burst projects
`[0, 0, target_distance_m]` through the recorded camera transform and locks
that base-frame center for the entire scan. Later camera motion does not move
the crop.

The reported field runs left this feature disabled. Measure the target and test
on saved data before enabling it during acquisition.

### Downsampling and outlier filtering

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `local_voxel_size_m` | 0.003 m | Per-capture voxel downsampling |
| `global_voxel_size_m` | 0.003 m | Accumulated-cloud voxel downsampling |
| `outlier_removal_enabled` | `true` | Enable the selected outlier stage |
| `outlier_filter_mode` | `statistical` | `none`, `radius`, or `statistical` |
| `outlier_radius_m` | 0.020 m | Radius-filter neighborhood |
| `outlier_min_neighbors` | 6 | Radius-filter minimum count |
| `statistical_nb_neighbors` | 20 | Statistical-filter neighborhood size |
| `statistical_std_ratio` | 2.0 | Statistical outlier threshold |

Radius parameters remain in the file so saved data can be replayed with the
`current_radius` profile. They are inactive while the live mode is
`statistical`.

### ICP and fusion

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `use_icp` | `true` | Apply multiscale ICP after TF placement |
| `icp_crop_margin_m` | 0.15 m | Margin around the source bounds for the local target |
| `icp_min_target_points` | 200 | Fall back to the full global map below this count |
| `max_icp_correction_translation_m` | 0.10 m | Reject implausibly large ICP translations |
| `max_icp_correction_rotation_deg` | 12.0° | Reject implausibly large ICP rotations |

ICP is a bounded refinement, not a replacement for TF or calibration. The
10 cm/12° guards were retained until the eye-in-hand transform can be formally
validated. Future experiments may evaluate approximately 3 cm/5° after
calibration.

### Persistence and lifecycle

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `save_raw_point_clouds` | `true` | Save raw camera- and target-frame PLYs |
| `publish_original_cloud` | `true` | Publish the latest TF-placed cloud |
| `shutdown_on_scan_finished` | `false` | Save on completion but keep the node alive |

Regardless of `save_raw_point_clouds`, the code saves the raw RGB/depth burst,
median arrays, intrinsics, TF, joint-state snapshots, and metadata. This flag
controls the two raw PLY files.

## Registration scales

Every scale uses point-to-plane ICP. Thresholds are checked after each scale;
failure at an early scale rejects the attempt.

| Scale | Voxel | Correspondence | Iterations | Relative RMSE | Relative fitness | RMSE ceiling | Fitness floor |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `SN3` | 0.050 m | 0.050 m | 50 | `1e-6` | `1e-6` | 0.025 m | 0.35 |
| `SN2` | 0.020 m | 0.030 m | 30 | `1e-6` | `1e-6` | 0.015 m | 0.40 |
| `SN1` | 0.010 m | 0.018 m | 20 | `1e-6` | `1e-6` | 0.010 m | 0.45 |
| `SN0` | no voxel | 0.012 m | 15 | `1e-6` | `1e-6` | 0.008 m | 0.45 |

The final `SN0` fitness floor was reduced from 0.50 to 0.45 for more variable
orchard overlap while retaining the 8 mm RMSE ceiling, stricter coarse stages,
and correction sanity bounds.

Each scale is represented by:

```text
registration.<scale>.voxel_size_m
registration.<scale>.max_correspondence_distance_m
registration.<scale>.max_iterations
registration.<scale>.relative_rmse
registration.<scale>.relative_fitness
registration.<scale>.rmse_threshold_m
registration.<scale>.fitness_threshold
```

## `scanning_trajectory`

This section is consumed by the field-validated JSON replay executable in the
external `arm_scan_motion` package.

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `capture_enabled` | `true` | Use the reconstruction handshake |
| `capture_request_topic` | `/tree_scan/capture_request` | Numbered request |
| `capture_result_topic` | `/tree_scan/capture_result` | Signed result |
| `scan_finished_topic` | `/tree_scan/scan_finished` | Completion output |
| `capture_settle_time_ms` | 250 ms | Wait after successful motion |
| `capture_timeout_sec` | 60.0 s | Includes all reconstructor retry attempts |
| `capture_subscriber_timeout_sec` | 30.0 s | Wait for the reconstructor to connect |
| `abort_on_capture_rejection` | `false` | Preserve a partial scan and continue after `-N` |
| `trajectory_timeout_scale` | 10.0 | Multiplier on nominal segment duration |
| `trajectory_timeout_margin_sec` | 10.0 s | Additional segment timeout |
| `max_steps` | 0 in YAML | Pass 1 for preflight or 32 for the validated full replay |
| `pause_ms` | 300 ms | Inter-segment pause |

Known external runtime overrides:

| Parameter | Validated use |
| --- | --- |
| `trajectory_input_path` | Absolute path to `recorded_scan_trajectory.json` |
| `max_steps` | Explicitly set to `1` during preflight and `32` for the final run |
| `capture_enabled` | Set `false` only for a motion-only test |

The external C++ source was not included in the documentation audit, so this
table claims only the supplied YAML values and the runtime overrides verified
in the project notes. Inspect the pinned C++ source or run `ros2 param list`
for any additional declarations.

## `scanning_trajectory_node`

This section configures the separate MoveIt waypoint/trajectory-recording
executable. It was not the final field replay path.

| Parameter | Supplied value | Meaning |
| --- | ---: | --- |
| `capture_enabled` | `true` | Enable capture handshake |
| `capture_request_topic` | `/tree_scan/capture_request` | Numbered request |
| `capture_result_topic` | `/tree_scan/capture_result` | Signed result |
| `scan_finished_topic` | `/tree_scan/scan_finished` | Completion output |
| `capture_settle_time_ms` | 1000 ms | Settle time during generated motion |
| `capture_timeout_sec` | 60.0 s | Wait through retries |
| `capture_subscriber_timeout_sec` | 30.0 s | Wait for reconstructor |
| `abort_on_capture_rejection` | `false` | Do not abort on one exhausted pose |

The proposed four-row generator could produce a 42-step trajectory with
0.15 m row spacing, ±0.15 m lateral positions, and a 15° look angle. That path
remained future work and must not be represented as field-validated.

## `compare_filters.py`

Run:

```bash
python3 src/reconstruct/scripts/compare_filters.py --help
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `scan_dir` | required | Dataset with `run_config.json` and `raw/capture_*` |
| `--profiles` | all four | `none`, `current_radius`, `dawood_radius`, `statistical` |
| `--output-dir` | timestamped directory under the scan | Explicit output root |
| `--no-icp` | false | Deprecated shortcut for TF-only; cannot combine with `--registration-modes` |
| `--registration-modes` | `tf_only tf_icp` | Registration variants |
| `--local-voxel-size-m` | saved value | Override local voxel size |
| `--global-voxel-size-m` | saved value | Override global voxel size |
| `--closure-pairs` | auto-detect | Comma-separated request pairs such as `3:12,13:22,23:32` |
| `--closure-translation-tolerance-m` | 0.005 m | Auto-detection translation tolerance |
| `--closure-rotation-tolerance-deg` | 1.0° | Auto-detection rotation tolerance |
| `--closure-min-frame-gap` | 8 | Minimum capture-index separation |

Filter profiles:

| Profile | Voxel | Outlier stage |
| --- | --- | --- |
| `none` | Current/overridden local voxel | None |
| `current_radius` | Current/overridden local voxel | Current radius and neighbor settings |
| `dawood_radius` | 0.008 m | 8 neighbors inside 0.014 m |
| `statistical` | Current/overridden local voxel | Current statistical settings |

The `none` profile still applies crop, voxelization, optional ICP, and global
voxelization. It disables only outlier removal.

## `compare_registration.py`

Run:

```bash
python3 src/reconstruct/scripts/compare_registration.py --help
```

### Source, profile, and closure options

| Argument | Default | Meaning |
| --- | --- | --- |
| `scan_dir` | required | Saved dataset |
| `--profiles` | `statistical` | Filter profiles to evaluate |
| `--output-dir` | `<scan>/registration_comparisons/<timestamp>` | Output root |
| `--local-voxel-size-m` | saved value | Override all variants |
| `--global-voxel-size-m` | saved value | Override all variants |
| `--closure-pairs` | auto-detect | Repeated-pose request pairs |
| `--closure-translation-tolerance-m` | 0.005 m | Auto-detection translation tolerance |
| `--closure-rotation-tolerance-deg` | 1.0° | Auto-detection rotation tolerance |
| `--closure-min-frame-gap` | 8 | Minimum separation for closure detection |

### Incremental-order options

| Argument | Default | Meaning |
| --- | --- | --- |
| `--skip-order-test` | false | Skip reverse and shuffled incremental replays |
| `--shuffle-count` | 3 | Deterministic shuffled replays |
| `--random-seed` | 23 | Shuffle and held-out-edge seed |

### TF-anchored pose-graph options

| Argument | Default | Meaning |
| --- | --- | --- |
| `--graph-priors` | `loose nominal strong` | TF-prior sweep |
| `--graph-scales` | `SN2 SN1 SN0` | Pairwise edge ICP scales |
| `--graph-min-aabb-overlap` | 0.05 | Minimum AABB intersection/min-volume ratio |
| `--graph-extra-neighbors` | 3 | Maximum extra overlap candidates per capture |
| `--graph-holdout-fraction` | 0.20 | Extra edges excluded from optimization |
| `--graph-max-correction-translation-m` | 0.030 m | Maximum edge departure from relative TF |
| `--graph-max-correction-rotation-deg` | 5.0° | Maximum edge departure from relative TF |
| `--robust-kernel-k-fraction` | 0.5 | Tukey-k fraction of correspondence distance |
| `--robust-kernel-min-m` | 0.004 m | Minimum Tukey-k |
| `--graph-edge-information-scale` | 1.0 | Geometric information multiplier |
| `--graph-edge-prune-threshold` | 0.25 | Open3D uncertain-edge prune threshold |
| `--graph-loop-closure-preference` | 1.0 | Certain/uncertain edge balance |

TF-prior presets are experimental weights, not measured calibration
covariances:

| Preset | Translation sigma | Rotation sigma |
| --- | ---: | ---: |
| `loose` | 20 mm | 2.0° |
| `nominal` | 10 mm | 1.0° |
| `strong` | 5 mm | 0.5° |

### Evaluation and visualization options

| Argument | Default | Meaning |
| --- | --- | --- |
| `--evaluation-voxel-size-m` | 0.010 m | Held-out geometric evaluation voxel |
| `--evaluation-max-distance-m` | 0.025 m | Nearest-neighbor evaluation limit |
| `--evaluation-trim-fraction` | 0.90 | Lowest residual fraction retained |
| `--skip-visuals` | false | Skip fixed-view comparison PNGs |
| `--visual-max-points` | 150,000 | Maximum plotted points per variant |

Keep `--graph-scales` ordered coarse-to-fine. Do not relax the 30 mm/5° edge
guards merely to make more edges pass; inspect overlap, crop, and calibration
first.
