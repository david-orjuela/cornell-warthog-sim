# TF-Anchored Registration Experiment

## What already exists

`compare_filters.py` already implements the scientific baseline in step 2:

- `tf_only` replays each saved `raw_target_cloud.ply` exactly as TF placed it.
- `tf_icp` replays the same selected captures with the current incremental
  frame-to-accumulated-cloud ICP architecture.
- Both modes use the same saved capture attempts, crop, filter profile, and
  voxel settings.

No edit to `compare_filters.py` is required for the TF-only baseline.

`compare_registration.py` is a companion script that imports those validated
baseline functions and adds steps 3â€“6:

- chronological, reverse, and seeded shuffled incremental-ICP replays;
- relative-TF-initialized robust point-to-plane ICP between selected pairs;
- a fixed `base_link` node and an absolute TF prior for every camera node;
- adjacent, TF-overlap, and repeated-pose/closure graph edges;
- Open3D batch pose-graph optimization;
- one-time fusion from the individually retained capture clouds;
- loose, nominal, and strong TF-prior sweeps;
- held-out edge metrics and repeated-pose closure metrics;
- fixed front/side/top RGB and capture-ID-colored comparison images.

## Required files

Place these scripts beside one another:

```text
compare_filters.py
compare_registration.py
```

The scan directory must contain:

```text
<scan_dir>/
â”œâ”€â”€ run_config.json
â””â”€â”€ raw/
    â”œâ”€â”€ capture_0001/
    â”‚   â”œâ”€â”€ capture.json
    â”‚   â””â”€â”€ raw_target_cloud.ply
    â””â”€â”€ ...
```

The current reconstructor already saves this layout. The companion script does
not need the original RGB/depth arrays or `raw_camera_cloud.ply`: it applies the
same base-frame crop/filter as the baseline and uses the inverse saved TF to
recover each selected cloud in its camera frame.

The runtime needs Python 3, NumPy, Open3D 0.19, and Matplotlib. No ROS nodes need
to be running; this is entirely offline.

## Full experiment

From the directory containing both scripts:

```bash
python3 compare_registration.py \
  ~/dev/tree_scans/indoor_model_tree_2 \
  --profiles statistical \
  --closure-pairs 3:12,13:22,23:32 \
  --local-voxel-size-m 0.004 \
  --global-voxel-size-m 0.004
```

If the request IDs change in a later scan, omit `--closure-pairs` and the script
will detect repeated TF poses automatically.

The default full run performs:

- TF-only baseline;
- chronological incremental TF+ICP baseline;
- reverse incremental ICP;
- three deterministic shuffled incremental-ICP replays;
- robust pairwise edge estimation using `SN2`, `SN1`, and `SN0`;
- graph optimization with all three TF-prior strengths;
- held-out numerical evaluation;
- fixed-view visual comparisons.

## Faster first smoke test

```bash
python3 compare_registration.py \
  ~/dev/tree_scans/indoor_model_tree_2 \
  --profiles statistical \
  --closure-pairs 3:12,13:22,23:32 \
  --graph-priors nominal \
  --shuffle-count 1
```

This still tests TF-only, current ICP, reverse order, one shuffle, and one graph
configuration.

## Optional two-filter replication

Filter choice is not the primary registration question, so one profile is the
cleanest first experiment. To replicate the entire registration comparison for
both filters already studied:

```bash
python3 compare_registration.py \
  ~/dev/tree_scans/indoor_model_tree_2 \
  --profiles current_radius statistical \
  --closure-pairs 3:12,13:22,23:32
```

## Main outputs

The default output root is:

```text
<scan_dir>/registration_comparisons/<timestamp>/
```

For each filter profile, inspect:

| Output | Meaning |
|---|---|
| `registration_comparison.csv` | One-row summary for TF-only, incremental ICP, and every graph prior |
| `order_test/order_summary.csv` | Mean, 95th-percentile, and maximum pose change caused only by replay order |
| `order_test/order_pose_differences.csv` | Per-capture order sensitivity relative to chronological replay |
| `pairwise_edges.csv` | Every graph edge attempted, its TF overlap, fitness/RMSE, TF correction, and acceptance reason |
| `heldout_edge_comparison.csv` | Pose residual and symmetric trimmed point-to-plane error on edges excluded from optimization |
| `baselines/*/global_point_cloud.ply` | TF-only and current incremental-ICP RGB reconstructions |
| `pose_graph/tf_prior_*/global_point_cloud.ply` | Optimized, fuse-once RGB graph reconstructions |
| `*/global_capture_id_cloud.ply` | Geometry colored by capture identity to expose duplicated shells |
| `visuals/registration_rgb_fixed_views.png` | Identical front, side, and top views for all main methods |
| `visuals/registration_capture_id_fixed_views.png` | Same views colored by contributing capture |
| `visuals/incremental_order_*_fixed_views.png` | Visual scan-order test |
| `pose_graph/tf_prior_*/optimized_pose_graph.json` | Optimized Open3D pose graph |

## How to interpret the experiment

### Scan-order dependence

If reverse/shuffled runs produce materially different final poses or visibly
different clouds from chronological replay, the current accumulated-map ICP is
scan-order-dependent. The only changed variable is replay order.

### TF-prior sweep

The prior presets are deliberately an experiment, not claimed calibration
uncertainties:

| Preset | Translation sigma | Rotation sigma |
|---|---:|---:|
| loose | 20 mm | 2.0Â° |
| nominal | 10 mm | 1.0Â° |
| strong | 5 mm | 0.5Â° |

Prefer a graph result only if it improves held-out geometry and repeated-pose
closure, reduces duplicated/thickened structures, preserves apples and thin
branches, and does not require implausibly large departures from calibrated TF.

### When TEASER++ becomes justified

Inspect `pairwise_edges.csv`. TEASER++ becomes a focused next experiment when a
specific, genuinely overlapping pairâ€”especially an uncertain loop closureâ€”
repeatedly fails robust ICP despite:

1. verified UR factory and camera-to-tool calibration;
2. a reasonable relative TF initialization;
3. sufficient overlap and point count; and
4. plausible crop/filter settings.

TEASER++ would then propose that difficult pairwise edge. The pose graph would
still be responsible for deciding whether it is globally consistent.

## Useful tuning

If no held-out overlap edges are selected:

```bash
--graph-min-aabb-overlap 0.02 --graph-extra-neighbors 5
```

If too many unrelated foliage pairs are attempted:

```bash
--graph-min-aabb-overlap 0.10 --graph-extra-neighbors 2
```

Do not relax the default 30 mm / 5Â° graph-edge correction limits merely to make
more edges pass. First inspect the failed request IDs, TF calibration, actual
overlap, and scene crop.
