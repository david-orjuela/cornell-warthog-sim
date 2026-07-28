# 2026-07-27 — Orchard Field Scan Addendum

## Field Deployment Outcome

The late-day dry window was sufficient to take the Warthog, UR5e, and RealSense system into the orchard. Six reconstruction datasets were produced across five physical trees, providing the first real-tree field validation of the complete motion, capture, retry, registration, saving, and summary pipeline.

All runs used the same core reconstruction settings:

- five-frame depth bursts with at least two valid samples per pixel;
- up to three capture attempts per requested pose;
- 0.3 m minimum depth;
- 3 mm local and global voxel sizes;
- statistical outlier removal;
- TF initialization followed by coarse-to-fine point-to-plane ICP;
- raw capture preservation and final PLY export.

The first five runs used a 2.5 m maximum depth. The final repeat, `outdoor_tree_6`, increased this limit to 3.5 m.

## Reconstruction Summary

| Dataset | Completed pose requests | Accepted requests | Rejected attempts | Mean ICP fitness | Mean ICP RMSE | Final points | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| `outdoor_tree_1` | 32 | 32 | 0 | 0.861 | 3.64 mm | 599,999 | Complete; usable away-from-sun scan |
| `outdoor_tree_2` | 32 | 32 | 0 | 0.915 | 3.39 mm | 664,476 | Complete, but visually unusable because the camera faced the sun |
| `outdoor_tree_3` | 32 | 32 | 0 | 0.836 | 3.78 mm | 1,031,782 | Complete; usable away-from-sun scan |
| `outdoor_tree_4` | 14 | 12 | 6 | 0.802 | 4.24 mm | 660,495 | Partial; two requests exhausted all three attempts |
| `outdoor_tree_5` | 15 | 15 | 0 | 0.810 | 4.13 mm | 737,169 | Partial first attempt on the final tree |
| `outdoor_tree_6` | 32 | 32 | 0 | 0.862 | 3.78 mm | 1,505,222 | Successful 32-view repeat of the final tree |

Across the six datasets, the reconstructor accepted 155 of 157 pose requests. It processed 161 total attempts, of which six were rejected. Four datasets completed all 32 planned views; two preserved useful partial reconstructions.

The `accepted_percent` value in the summary is attempt-based. For example, `outdoor_tree_4` reports 66.7% because 12 of 18 attempts were accepted, although its pose-request success rate was 12 of 14, or 85.7%.

## Direct-Sun Failure

`outdoor_tree_2` was the only run performed while the camera faced the sun. The RealSense depth observations of the tree were largely lost, and the resulting reconstruction was qualitatively unacceptable. All later tests were oriented away from the sun and produced substantially cleaner tree geometry.

This run exposed a critical limitation in the current metrics. Despite being the worst reconstruction visually, `outdoor_tree_2` had:

- the highest mean ICP fitness, 0.915;
- the lowest mean ICP RMSE, 3.39 mm;
- 32/32 accepted capture requests;
- more than 664,000 final points.

ICP fitness and RMSE therefore describe how well the surviving points align; they do not establish that the target tree was captured completely or even that most of the retained points belong to the tree. Final point count has the same limitation because background or otherwise irrelevant geometry can remain dense.

Future acquisition summaries should include target-quality measurements before registration:

- valid-depth percentage within a tree region of interest;
- raw and filtered target-point counts per pose;
- image/depth saturation or invalid-pixel rate;
- per-capture depth coverage and distance distribution;
- the number of captures contributing to each local tree region.

The field evidence supports treating direct sunlight as an acquisition-condition failure that must be detected before ICP acceptance, rather than as a registration problem to solve afterward.

## Partial Runs and Retry Behavior

The two incomplete datasets represent different failure types.

For `outdoor_tree_4`, 12 pose requests were accepted and two requests exhausted all three retries, producing six rejected attempts. The rejected attempts did not contaminate the accumulated cloud, and the accepted portion was still saved. This validated the field retry and partial-result preservation behavior under a more difficult real-tree scene.

For `outdoor_tree_5`, every one of the 15 requests received by the reconstructor was accepted. The summary therefore cannot classify the stop at 15/32 as an ICP or capture rejection; the interruption occurred elsewhere in the motion/run sequence or after the last recorded request. Repeating the same final tree as `outdoor_tree_6` completed all 32 requests successfully, showing that the trajectory and reconstruction pipeline could recover on a clean rerun.

## Interpretation

The orchard deployment established a strong operational baseline:

- the system can execute a complete 32-pose raster and reconstruct real orchard trees in the field;
- the depth-burst, retry, rejection, raw-data, PLY, configuration, and summary mechanisms operated outside the lab;
- away-from-sun orientation was a necessary condition for usable RealSense depth;
- three complete away-from-sun runs produced promising starting reconstructions;
- rejected attempts were isolated rather than fused;
- partial scans remained recoverable for diagnosis;
- a failed partial run on the final tree was followed by a successful 32/32 repeat.

The run summaries are not sufficient to rank geometric reconstruction quality. The apparently excellent ICP statistics of the sun-facing failure reinforce the earlier indoor conclusion that local fitness and RMSE cannot serve as standalone quality metrics. Each dataset must still be evaluated with TF-only, incremental TF+ICP, and TF-anchored pose-graph replay, together with fixed local crops and thin cross-sections through trunks, branches, leaves, and apples.

Point count and reconstruction time also increased substantially across some runs, reaching approximately 1.5 million points and 6.0 seconds mean reconstruction time in `outdoor_tree_6`. These values are influenced by the number of accepted views, scene content, the larger 3.5 m depth range in the final run, and the growing accumulated cloud, so they should be treated as workload indicators rather than direct measures of quality.

## Resulting Experimental Direction

The field data should be preserved as an independent validation set rather than used immediately for method-specific tuning. The next analysis should:

1. inspect each final PLY and label the sun-facing and partial-run failure modes explicitly;
2. calculate per-capture valid-depth and target-point coverage, especially for `outdoor_tree_2`;
3. replay the away-from-sun datasets using TF-only, chronological incremental ICP, and the TF-anchored pose graph;
4. test incremental ICP order sensitivity on at least one complete outdoor scan;
5. compare identical close-up regions around a trunk, branch, visible apple, partially occluded apple, and representative leaves;
6. measure local surface thickness and duplicate shells with fixed thin cross-sections;
7. compare `outdoor_tree_5` and `outdoor_tree_6` only with the configuration and completion differences stated explicitly;
8. reserve TEASER++ or another global pairwise solver for specific overlapping frame pairs that fail robust TF-initialized ICP, rather than using it to address missing sun-corrupted depth.

The primary field conclusion is that the full system is operational and can collect promising real-tree reconstructions, but acquisition completeness must become a first-class metric. Direct sunlight can make a reconstruction unusable while every existing registration statistic still appears successful.
