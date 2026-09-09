# TF-Anchored Registration Experiment

## Purpose

This experiment compares three registration architectures while holding the
selected captures, crop, filter, and voxel settings fixed:

1. TF-only fusion;
2. chronological frame-to-accumulated-cloud TF+ICP;
3. TF-anchored batch pose-graph optimization followed by one-time fusion.

It also replays incremental ICP in reverse and seeded shuffled orders. If only
the replay order changes but the estimated poses, accepted captures, or final
geometry change materially, accumulated-map ICP is order-dependent.

## Scripts

Keep these files beside one another:

```text
src/reconstruct/scripts/compare_filters.py
src/reconstruct/scripts/compare_registration.py
```

`compare_registration.py` imports the validated capture selection, crop,
filter, metric, and baseline-fusion functions from `compare_filters.py`.

## Required dataset layout

```text
<scan_dir>/
├── run_config.json
└── raw/
    ├── capture_0001/
    │   ├── capture.json
    │   └── raw_target_cloud.ply
    └── ...
```

For each request ID, the scripts prefer the highest-numbered accepted attempt.
If every attempt was rejected, they retain the latest attempt so it can still
be tested under another offline configuration.

The experiment needs Python 3, NumPy, Open3D 0.19, and Matplotlib. ROS nodes and
robot hardware do not need to be running.

## Baseline experiment

`compare_filters.py` provides controlled `tf_only` and `tf_icp` replays:

```bash
python3 src/reconstruct/scripts/compare_filters.py \
  "$HOME/dev/tree_scans/<scan_id>" \
  --profiles statistical \
  --registration-modes tf_only tf_icp \
  --local-voxel-size-m 0.003 \
  --global-voxel-size-m 0.003
```

Both modes consume the same selected captures in the same order.

## Fast pose-graph smoke test

```bash
python3 src/reconstruct/scripts/compare_registration.py \
  "$HOME/dev/tree_scans/<scan_id>" \
  --profiles statistical \
  --graph-priors nominal \
  --shuffle-count 1
```

This still runs TF-only, chronological TF+ICP, reverse ICP, one deterministic
shuffle, one pose-graph prior, held-out evaluation, and fixed-view visuals.

## Full indoor experiment

The known repeated-pose pairs for the reported indoor dataset were:

```bash
python3 src/reconstruct/scripts/compare_registration.py \
  "$HOME/dev/tree_scans/indoor_model_tree_2" \
  --profiles statistical \
  --closure-pairs 3:12,13:22,23:32 \
  --local-voxel-size-m 0.004 \
  --global-voxel-size-m 0.004
```

If request IDs change, omit `--closure-pairs`; the script attempts to detect
repeated TF poses automatically.

The default full run performs:

- TF-only fusion;
- chronological incremental TF+ICP;
- reverse incremental ICP;
- three deterministic shuffled incremental-ICP replays;
- robust relative-TF-initialized point-to-plane ICP edges;
- adjacent, TF-overlap, and repeated-pose closure constraints;
- loose, nominal, and strong TF-prior sweeps;
- held-out numerical evaluation;
- one-time fusion from optimized per-capture poses;
- fixed front, side, and top RGB/capture-ID views.

## Optional two-filter replication

Filter choice is not the primary registration variable, so one profile is the
cleanest first experiment. To test whether the conclusion is filter-specific:

```bash
python3 src/reconstruct/scripts/compare_registration.py \
  "$HOME/dev/tree_scans/indoor_model_tree_2" \
  --profiles current_radius statistical \
  --closure-pairs 3:12,13:22,23:32
```

## Output layout

Default output:

```text
<scan_dir>/registration_comparisons/<timestamp>/
```

For each filter profile:

| Output | Meaning |
| --- | --- |
| `registration_comparison.csv` | Summary for TF-only, incremental ICP, and every graph prior |
| `order_test/order_summary.csv` | Mean, 95th-percentile, and maximum pose changes caused only by replay order |
| `order_test/order_pose_differences.csv` | Per-capture order sensitivity |
| `pairwise_edges.csv` | Every attempted graph edge, overlap, metrics, correction, and acceptance reason |
| `heldout_edge_comparison.csv` | Pose and trimmed point-to-plane errors on edges excluded from optimization |
| `baselines/*/global_point_cloud.ply` | TF-only and incremental-ICP reconstructions |
| `pose_graph/tf_prior_*/global_point_cloud.ply` | Fuse-once graph reconstructions |
| `*/global_capture_id_cloud.ply` | Geometry colored by capture identity |
| `visuals/registration_rgb_fixed_views.png` | Identical RGB front/side/top views |
| `visuals/registration_capture_id_fixed_views.png` | Same views colored by capture |
| `visuals/incremental_order_*_fixed_views.png` | Visual replay-order comparison |
| `pose_graph/tf_prior_*/optimized_pose_graph.json` | Optimized Open3D graph |

Every run also writes manifests that preserve the source dataset, selected
capture attempts, effective settings, and transform conventions.

## Transform convention

`T_base_camera` maps camera-frame points into `base_link`.

A geometric edge from camera `i` to camera `j` stores
`T_camera_j_camera_i`.

Open3D graph node poses store `T_base_camera`. Node 0 is a fixed identity
`base_link` node. The absolute TF prior for capture `i` is therefore an edge
from node 0 to node `i` whose measurement is
`inverse(T_base_camera_i)`.

## Prior sweep

The presets are experimental weights, not measured calibration covariances:

| Preset | Translation sigma | Rotation sigma |
| --- | ---: | ---: |
| `loose` | 20 mm | 2.0° |
| `nominal` | 10 mm | 1.0° |
| `strong` | 5 mm | 0.5° |

Prefer a graph result only if it improves held-out consistency and repeated-pose
closure, reduces duplicate/thick surfaces, preserves thin structures, and does
not require implausibly large departures from calibrated TF.

## Reported indoor result

The chronological incremental baseline accepted all 32 captures and appeared
successful under conventional local metrics:

- mean fitness: approximately 0.927;
- mean RMSE: approximately 3.32 mm.

It nevertheless accumulated increasing departure from TF:

| Metric | Incremental TF+ICP |
| --- | ---: |
| Mean translation deviation | 5.30 mm |
| 95th-percentile translation deviation | 11.02 mm |
| Maximum translation deviation | 13.46 mm |
| Mean rotation deviation | 1.01° |
| Maximum rotation deviation | 1.94° |
| Translation-deviation slope | 0.264 mm/frame |
| Correlation with frame index | 0.762 |

Changing only replay order produced:

| Replay order | Accepted | Mean pose difference | Maximum pose difference | Mean rotation difference |
| --- | ---: | ---: | ---: | ---: |
| Reverse | 31/32 | 13.44 mm | 18.77 mm | 1.92° |
| Shuffled | 28/32 | 13.33 mm | 19.82 mm | 2.00° |

The nominal TF-anchored graph reduced mean deviation to approximately 1.99 mm
and 0.108°, reduced maximum deviation to 4.05 mm and 0.198°, and removed the
frame-wise translation trend.

It improved held-out pose residual and repeated-pose closure relative to
incremental ICP, while incremental ICP retained a slightly lower held-out
point-to-plane RMSE. The first pose-graph result was therefore a strong
drift-control result, not a definitive visual-quality winner.

## Visual evaluation

Full-canopy views can hide doubled leaves and branch surfaces. Use identical
base-frame crops for:

- a trunk section;
- one branch junction;
- two apples at different locations;
- representative thin leaves or leaf edges.

Use the same crop bounds, view, point size, and zoom for every method.

Thin slabs through a trunk, branch, or apple can reveal:

- duplicate shells;
- surface thickness;
- gaps;
- capture-to-capture displacement;
- whether different capture IDs form one surface or several layers.

Useful local metrics include cross-section thickness, fitted-plane/cylinder
residuals, nearest-neighbor spacing, and capture-to-capture spread.

## When TEASER++ is justified

TEASER++ addresses difficult pairwise alignment; it does not by itself solve
one-way accumulation in a sequential fusion architecture.

Inspect `pairwise_edges.csv`. A focused TEASER++ experiment becomes justified
when a specific, genuinely overlapping pair—especially a loop closure—fails
robust TF-initialized ICP despite:

1. verified UR and camera-to-tool calibration;
2. a reasonable relative-TF initialization;
3. sufficient overlap and point count;
4. plausible crop and filter settings.

TEASER++ could then propose that edge. The pose graph would still decide
whether the edge is globally consistent.

## Useful tuning

If no held-out overlap edges are selected:

```bash
--graph-min-aabb-overlap 0.02 --graph-extra-neighbors 5
```

If too many unrelated foliage pairs are attempted:

```bash
--graph-min-aabb-overlap 0.10 --graph-extra-neighbors 2
```

Do not relax the default 30 mm/5° graph-edge correction guards merely to make
more edges pass. Inspect failed request IDs, TF calibration, actual overlap,
and scene crop first.
