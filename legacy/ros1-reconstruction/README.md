# Superseded Reconstruction Experiments

This directory is for historical code that is intentionally excluded from the
active ROS 2 package installation.

Expected archived files:

- `curobo_motion_planner.py` — an experimental ROS 2/cuRobo planner migrated
  before the final field workflow was established. It uses the obsolete
  `/capture_alert` and `/capture_done` interfaces and was not used for the
  reported orchard datasets.
- `old_pointcloud_processing.py` — an earlier reconstruction implementation
  superseded by
  `src/reconstruct/scripts/pointcloud_processing.py`.

These files are retained for development history and attribution. Do not run
them as handoff quickstart commands or use them to reproduce the reported
results.
