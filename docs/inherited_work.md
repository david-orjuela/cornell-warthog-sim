# Inherited Work Register

## Purpose

This is the operational register for inherited code and assets. It answers a different question from `provenance.md`:

> What should be done with each inherited component under the current reconstruction-focused scope?

For authorship, licensing, and chain-of-custody details, use `provenance.md`.

---

## Disposition Categories

- **Active — adapt:** required for the current deliverable and being changed now.
- **Active dependency:** required, but maintained primarily upstream or by the lab.
- **Preserve — downstream:** not part of the immediate milestone, but likely needed after reconstruction succeeds.
- **Preserve — decision required:** conflicting or ambiguous inherited work that must not be overwritten.
- **Archive:** useful evidence, historical work, or generated output; not active source.
- **Exclude from current scope:** do not spend time on it unless the project is redirected.

---

## Register

| Component | Immediate source | Current interpretation | Disposition | Priority / next action |
| --- | --- | --- | --- | --- |
| Dawood `motion_planner.py` | Dawood reconstruction repository | ROS 1 UR5e + cuRobo control with hardcoded scan/pruning behavior and unsafe assumptions | **Active — adapt** | Validate ROS 2 action namespace, six-joint state source, cuRobo configs, quaternion convention, raster axes, and dry-run behavior |
| Dawood `pointcloud_processing.py` | Dawood reconstruction repository | Core synchronized RGB-D, TF, Open3D, ICP, filtering, fusion, metrics, and publishing logic | **Active — adapt** | Produce one stationary ROS 2 PLY, then two-view and small-raster validation |
| Dawood `pc_utils.py` | Dawood reconstruction repository | Shared point-cloud conversion and clustering helpers; ROS 1 conversion dependency | **Active — adapt** | Retain useful geometry helpers; consolidate duplicate conversion code; validate ROS 2 `PointCloud2` output |
| Dawood ROS 1 package/launch files | Dawood reconstruction repository | Catkin, `rospy`, ROS 1 actions/services/TF, old topic paths | **Archive after migration** | Keep for traceability; do not run as the active Warthog implementation |
| YOLO segmentation integration | Dawood reconstruction repository | Dormant-tree trunk/branch instance-mask coloring | **Preserve — downstream / optional** | Bypass for initial leafy geometry; revisit only for vegetation/background separation or structural analysis |
| BiSeNet integration | Dawood reconstruction repository | Alternative dormant-tree semantic segmentation | **Archive / optional** | Do not migrate until there is a demonstrated need and model/license provenance is known |
| YOLO/BiSeNet model weights | External files used by Dawood's pipeline | Required only for semantic modes | **Preserve — decision required** | Verify ownership, training data, version, and license before copying into a new repository |
| `skeletonization.py` | Dawood reconstruction repository | DBSCAN and spline-based trunk/branch hierarchy extraction | **Archive / optional** | Exclude from geometry-first scan; reconsider for procedural collision geometry or horticultural measurements |
| `tree_component.py` | Dawood reconstruction repository | Centerline, parent/child, length, and diameter calculations with several experimental inconsistencies | **Archive / optional** | Preserve notes; do not treat current measurements as validated |
| `skeletonization_params.yaml` | Dawood reconstruction repository | Dormant-tree class and clustering parameters; includes implausible secondary voxel value | **Archive / optional** | Keep with original structural pipeline; do not apply to leafy cloud by default |
| `test.py` and example marker code | Dawood reconstruction repository | RViz line-strip example rather than a reconstruction test | **Exclude from current scope** | Retain only if useful as a tiny visualization example |
| `tfs.rviz` | Dawood reconstruction repository | Documents intended ROS 1 output topics and fixed frame | **Archive / reference** | Create a new RViz2 config for `/tree_scan/latest_tf_cloud` and `/tree_scan/global_cloud` |
| `ur5e.yml` | Dawood/lab cuRobo configuration | Robot model/config required for motion planning | **Active dependency** | Obtain authoritative file; verify joint names, kinematics, limits, and compatibility with current cuRobo version |
| `collision_table.yml` | Dawood/lab cuRobo configuration | Generic collision environment, not yet adequate for the Warthog-mounted arm | **Active dependency** | Replace or extend with the actual mount, camera, floor, equipment, and conservative specimen exclusion volume |
| Physical UR calibration | Lab/Universal Robots calibration process | Required for accurate live and replayed kinematics | **Active dependency** | Identify authoritative file and use consistently online/offline |
| Camera mount / hand–eye calibration | Lab hardware configuration | Required for reliable base-frame placement | **Active dependency** | Verify the complete fixed-frame-to-optical-frame chain; do not rely on ICP to compensate |
| Jetson camera driver and topics | Warthog Jetson | Publishes live ROS 2 RGB, aligned depth, and CameraInfo | **Active dependency** | Measure local/remote rates, encodings, QoS, timestamps, and frame IDs; record a bag |
| ROS 2 migrated reconstruction package | July 21 adaptation | New `ament_cmake`/Python package with ROS 2 motion, reconstruction, launch, parameters, safety, and output flow | **Active — adapt** | Clean-build and run reconstruction-only; then perform controller and supervised motion validation |
| `Spatio-Temporal-Mapping` | Nidhish GitHub repository | Clean baseline for the inherited Warthog orchard simulator | **Preserve — downstream** | Keep as simulator authority; avoid mixing reconstruction changes into it prematurely |
| Recovered `cornell_orchard_ws` | Lab desktop | Buildable ROS 2 simulator plus possible local uncommitted changes and artifacts | **Preserve — decision required** | Keep untouched recovery copy; retain selective patches separately; ask Nidhish before choosing variants |
| `.gitmodules` repair branch | David's repair based on recovered metadata | Restores the LIO-SAM source URL and makes recursive clones reproducible | **Preserve — decision required** | Ask Nidhish whether to merge by PR, retain unmerged, or remove |
| LIO-SAM upstream submodule | TixiaoShan/LIO-SAM at expected commit | Third-party mapping package used by the simulator | **Preserve — downstream** | Keep exact commit; use upstream attribution/license; no active reconstruction work needed |
| Project-specific LIO-SAM config/launch | Clean and recovered simulator copies | Differing local experiment settings | **Preserve — decision required** | Diff and document; defer selection until mapping work resumes |
| `orchard_description` worlds | Simulator repository and recovered workspace | Existing dormant/high-resolution orchard environments | **Preserve — downstream** | Replace absolute paths only when needed; later insert cleaned leafy assets |
| `generate_orchard_new_trees.py` variants | Clean and recovered `orchard_description` | Potentially different generations of orchard layout tooling | **Preserve — decision required** | Identify which version reflects the latest intended workflow before modifying |
| Orchard meshes/textures | Repository Git LFS and recovered workspace | Visual assets with incomplete per-file provenance | **Preserve — decision required** | Trace references and source/license before redistribution or derivative release |
| `orchard_final_high_res.sdf` variants | Clean and recovered `orchard_description` | Structural and path differences | **Preserve — downstream** | Defer portability cleanup until a reconstructed asset is ready for insertion |
| `orchard_perception` launch files | Clean and recovered workspaces | Simulator orchestration and mapping/perception integration | **Preserve — downstream** | Do not spend current time reconciling unless needed to demonstrate an imported tree |
| `temporal_projector.cpp` variants | Clean and recovered `orchard_perception` | Substantially different experimental implementations | **Preserve — decision required** | Keep both; obtain Nidhish's explanation before selecting or merging |
| Orchard RViz configs | Simulator repository and recovered workspace | Useful but non-authoritative visualization state | **Preserve — downstream** | Retain useful displays; low priority |
| `orchard_navigation_rl_ws` | Prior learned-navigation work | Behavior cloning, DAgger, policies, checkpoints, metrics, and evaluation separate from current assignment | **Exclude from current scope** | Archive and revisit only by explicit redirection |
| `final_colored_orchard.pcd` | Recovered desktop | Generated mapping/reconstruction result with incomplete run metadata | **Archive** | Store externally with any recoverable date, command, config, and data reference |
| `frames_*.gv` / `frames_*.pdf` | Recovered desktop | Generated TF diagrams | **Archive** | Keep under experiment artifacts; label with source workspace and date if known |
| `build-reproduction.log` | July 17 clean reproduction | Evidence that the inherited simulator built successfully | **Archive** | Attach to notes/issues or keep outside active source files |
| New scan bags, images, depth maps, PLY/PCD, JSON, CSV, and logs | Active experiments | Raw data and generated outputs from the migrated pipeline | **Archive as research data/artifacts** | Use a documented data hierarchy, checksums, metadata, and lab-approved storage—not normal Git history |

---

## Current Ownership Boundaries

### Active code owned by this adaptation effort

The current effort may claim authorship of the ROS 2 interface and engineering changes it actually introduces, including:

- ROS 2 package/build structure;
- `rclpy` node interfaces;
- ROS 2 action, TF2, QoS, synchronization, and `PointCloud2` integration;
- configurable camera/controller/frame parameters;
- the absolute raster generator;
- the move–settle–capture acknowledgement flow;
- motion-disable safety defaults;
- bounded ICP corrections;
- PLY and metadata output; and
- updated validation and operating documentation.

It should not claim original authorship of Dawood's reconstruction pipeline, Nidhish's simulator, upstream LIO-SAM, cuRobo, Open3D, Clearpath packages, or Universal Robots software.

---

## Rules for Using Inherited Work

1. Keep clean baselines and recovered snapshots separate.
2. Do not overwrite ambiguous competing versions.
3. Adapt only the components required for the current milestone.
4. Preserve attribution in file headers, commit messages, and documentation.
5. Treat unknown license or model-weight provenance as a release blocker, not as permission.
6. Keep generated data and logs outside normal source history unless an approved LFS/data policy exists.
7. Do not replace absolute paths globally just to make files look clean; fix them when the relevant workflow is understood and tested.
8. Do not execute inherited robot motion without validating the current hardware, controller, calibration, collision world, and start pose.
9. Prefer rosbags and offline replay so algorithm changes do not require repeated physical scans.
10. Record decisions from Dawood, Nidhish, and Divyanth in this register or linked issues.

---

## Required Maintainer Decisions

- Which repository should own the migrated reconstruction package?
- What branch/commit is the authoritative Dawood baseline?
- Should the `.gitmodules` repair be merged?
- Which `temporal_projector.cpp` version corresponds to which experiment?
- Which orchard generator and high-resolution world version should be retained?
- May the orchard meshes, textures, robot calibration, model weights, and collected tree data be redistributed?
- What is the expected minimum number and quality of reconstructed orchard trees?
- Who will supervise physical UR5e tests and orchard collection?
