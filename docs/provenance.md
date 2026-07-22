# Provenance and Chain of Custody

## Purpose

This document records where inherited code, configuration, assets, and generated artifacts came from; what is currently known about authorship; what David changed; and which licensing or ownership questions remain unresolved.

An unknown license or author does not prove that an item is unusable, but it does mean the item should not be redistributed publicly or represented as original work until its status is verified.

---

## Source Authority Hierarchy

For the current project, use the following hierarchy unless a lab maintainer provides a newer authoritative source:

1. **Current active reconstruction package** — David's ROS 2 adaptation of the required portions of Dawood's pipeline.
2. **Dawood's reconstruction repository and handoff material** — source for the inherited RGB-D acquisition, registration, semantic segmentation, and skeletonization logic.
3. **`Spatio-Temporal-Mapping` GitHub repository** — clean baseline for Nidhish's Warthog orchard simulator.
4. **Recovered desktop `cornell_orchard_ws`** — recovery source for possible uncommitted simulator changes and generated outputs; not automatically authoritative over the clean repository.
5. **Upstream third-party repositories** — authoritative for their own code, licenses, and history, including the LIO-SAM submodule.
6. **`orchard_navigation_rl_ws`** — older, separate learned-navigation work; not an authority for the active reconstruction task.

---

## Chain of Custody

### Recovered lab-desktop workspaces

After receiving administrator access to the lab desktop, David located prior ROS workspaces while investigating the installed ROS and Gazebo environment. He copied the workspaces into his own directory for inspection and left the original directories untouched.

The recovered copy should therefore be treated as:

- a snapshot of files present on that machine;
- a possible source of uncommitted changes;
- a source of generated result artifacts; and
- evidence for comparison with the clean repository.

It should not be treated as proof of authorship, licensing, or final intended behavior without confirmation from Nidhish or another maintainer.

### Clean simulator repository

Nidhish identified `Spatio-Temporal-Mapping` as the clean repository baseline for the simulator. David initialized its dependencies, downloaded Git LFS content, and compared it with the recovered desktop workspace.

### Dawood reconstruction code

Dawood's code is the inherited basis for the tree-scanning and reconstruction workflow. The reviewed files include motion planning, point-cloud processing, semantic segmentation, skeletonization, tree-component analysis, utility code, launch files, and parameter files.

The active project adapts only the portions required for geometry-first RGB-D reconstruction. It does not imply authorship of Dawood's original algorithms.

---

## Provenance Register

| Item | Immediate source | Known or likely author | David's changes | Current role | License / release status |
| --- | --- | --- | --- | --- | --- |
| `Spatio-Temporal-Mapping` repository | Nidhish's GitHub repository | Nidhish and prior Cornell contributors; exact per-file authors in Git history | Comparison, setup repair branch, reproduction notes | Clean simulator baseline | Verify repository license and per-asset exceptions before public release |
| Recovered `cornell_orchard_ws` | Lab desktop user directories | Nidhish / other prior lab contributors; exact authors vary | Copied for inspection; originals left untouched; no blanket merge | Recovery source for local changes and artifacts | Internal status and per-file licensing require verification |
| `src/LIO-SAM` | Git submodule pointing to `https://github.com/TixiaoShan/LIO-SAM.git` | Upstream LIO-SAM authors | Restored missing `.gitmodules` metadata; preserved expected commit `08af3f32f01725372d4269838dc44c19c6d9e76b` | Third-party mapping dependency in simulator | Use upstream repository/license; do not describe upstream code as Cornell-authored |
| LIO-SAM project configuration | Clean and recovered simulator workspaces | Likely Nidhish/project contributors, based on upstream package | No authoritative merge selected; differences preserved | Deferred simulator-specific configuration | Attribution follows Git history; verify before publishing configuration as a lab release |
| `orchard_description` worlds and generators | Clean and recovered simulator workspaces | Nidhish / Cornell contributors; some assets may have other origins | Reproduction and comparison only so far | Downstream environment and asset integration | Per-mesh, texture, and imported-model provenance remains incomplete |
| Orchard meshes and textures tracked through Git LFS | `orchard_description` / repository LFS | Unknown at present | Downloaded LFS objects for comparison; no claim of authorship | Simulator visual assets | Do not redistribute externally until source and license are documented |
| `orchard_perception` code | Clean and recovered simulator workspaces | Nidhish / Cornell contributors | Compared major files; no implementation selected | Deferred simulator perception/mapping | Check Git history and repository license |
| Competing `temporal_projector.cpp` versions | Clean repository and recovered workspace | Nidhish / prior lab experimentation | Preserved both; no merge | Deferred experimental variants | Ownership likely Cornell-internal; confirm purpose and release status with Nidhish |
| `orchard_navigation_rl_ws` | Recovered or existing learned-navigation repository/workspace | Nidhish, Divyanth, and/or prior contributors | None beyond review and scope classification | Archived separate work | Check its own repository, dependencies, checkpoints, and dataset licenses |
| Dawood `motion_planner.py` | Dawood reconstruction repository | Dawood plus third-party cuRobo/ROS concepts | Rewritten to ROS 2 API; name-based joints; configurable action; absolute raster; safety interlock; capture handshake | Active inherited logic under adaptation | Preserve attribution to Dawood; verify original repository license and third-party notices |
| Dawood `pointcloud_processing.py` | Dawood reconstruction repository | Dawood plus Open3D/ROS concepts | Rewritten to ROS 2; geometry-first mode; TF2; topic/encoding/depth handling; bounded ICP; outputs and metadata | Active inherited logic under adaptation | Preserve attribution to Dawood; verify original repository license |
| Dawood `pc_utils.py` | Dawood reconstruction repository | Dawood | Replaced ROS 1 `ros_numpy` conversion with direct ROS 2 `PointCloud2` helper; consolidation still possible | Active utility layer | Preserve original attribution for retained logic |
| Dawood YOLO/BiSeNet integration and model weights | Dawood repository and external model files | Dawood integration; model/framework authors external | Bypassed for initial leafy reconstruction | Optional/deferred | Model-weight and dataset licenses must be checked separately from code |
| Dawood skeletonization and `tree_component.py` | Dawood repository | Dawood | Reviewed; not migrated into the initial geometry-first path | Archived / optional later | Preserve attribution; verify repository license before reuse or publication |
| ROS 2 migration package | Created from July 21 adaptation work | David for migration/refactor; inherited algorithms remain attributed to Dawood and external projects | New package/build/launch/parameter structure and ROS 2 interface layer | Active current code | Add a clear NOTICE/attribution file before release; choose license with lab approval |
| Clearpath Warthog simulation stack | Installed ROS 2 packages and generated Clearpath configuration | Clearpath Robotics and package contributors | Reproduction, setup-path diagnosis, and configuration use | Simulator dependency | Follow upstream package licenses and model terms |
| Universal Robots ROS 2 driver/calibration | Installed or lab-provided UR software and calibration | Universal Robots ROS driver contributors; lab calibration generated for the physical arm | Integration pending | Physical motion dependency | Do not publish robot-specific calibration without lab approval |
| cuRobo | External motion-planning framework and config files | Upstream cuRobo authors; local configs may be Dawood/lab-authored | Interface adaptation and safety checks | Motion-planning dependency | Follow upstream license; determine provenance of `ur5e.yml` and `collision_table.yml` |
| Open3D | External library | Open3D authors | Used for RGB-D back-projection, registration, filtering, and PLY output | Reconstruction dependency | Follow upstream license |
| `final_colored_orchard.pcd` | Recovered desktop workspace | Generated by an earlier experiment; operator/config unknown | None | Archived result artifact | Not source code; document generating command/data before reuse in a publication |
| `frames_*.gv` / `frames_*.pdf` | Recovered desktop workspace | Generated by TF visualization tooling | None | Archived diagnostic artifacts | Tool-generated; accompanying robot/config provenance remains necessary |
| `build-reproduction.log` | David's clean-build reproduction | David as operator; content from build tools and packages | Generated during reproduction | Diagnostic evidence | Keep out of source unless attached to an issue or handoff record |
| New PLY/PCD, images, depth frames, bags, metrics, and summaries | Active reconstruction runs | David/lab operators as data collectors; subjects and hardware lab-owned | Generated by the adapted pipeline | Research data and experiment artifacts | Store under lab-approved data policy; record date, operator, hardware, parameters, and consent/access restrictions |

---

## Third-Party and External Dependency Notes

The active repository should distinguish source code written or modified by the project from external dependencies installed through ROS, Python, CUDA, or system package managers.

At minimum, preserve attribution and verify licensing for:

- LIO-SAM;
- Clearpath ROS 2 / Gazebo packages and models;
- Universal Robots ROS 2 driver and calibration tools;
- cuRobo;
- Open3D;
- ROS 2, TF2, message_filters, and related packages;
- YOLO/Ultralytics if retained;
- BiSeNet and any copied `lib/` code if retained;
- model weights and their training datasets;
- imported Gazebo meshes, textures, and SDF/URDF assets; and
- any code generated or substantially assisted by automated tools.

Dependency names in package manifests do not require copying their source into this repository. Vendored or modified third-party code requires clearer notices and, where applicable, preservation of license files.

---

## Authorship and Modification Rules

When documenting or publishing the project:

- attribute the original reconstruction workflow and project-specific ROS 1 code to Dawood unless Git history establishes more specific authorship;
- attribute the ROS 2 migration, interface refactor, safety interlocks, and new output workflow to David while making clear that the underlying reconstruction concepts were inherited;
- attribute simulator components according to their Git history and Nidhish's handoff;
- attribute LIO-SAM and other external libraries to their upstream authors;
- do not call recovered local files “David's code” merely because David copied or repaired them;
- do not call generated point clouds, TF diagrams, logs, or meshes source code; and
- do not infer a license from a neighboring package or parent repository when an asset may have been imported separately.

---

## Data and Artifact Metadata

Every new scan dataset or reconstruction should record, at minimum:

```text
date and time
operator
location and tree identifier
indoor/outdoor condition
robot and camera identifiers
ROS distribution and package commit
camera topics and frame IDs
fixed reconstruction frame
camera intrinsics
hand–eye / mount calibration reference
UR calibration reference
controller and joint-state interfaces
parameter snapshot
raw rosbag path and checksum
accepted/rejected frame counts
PLY/PCD path and checksum
notes on motion, lighting, wind, occlusion, and failures
```

Large data should not be committed directly to normal Git history unless the repository intentionally uses Git LFS and the lab approves the storage policy.

---

## Open Provenance Questions

1. What is the exact repository URL, branch, and commit for Dawood's latest reconstruction code?
2. What license, if any, applies to Dawood's project-specific files?
3. Who created each orchard mesh, texture, and tree model, and were any imported from external sources?
4. What license governs the clean `Spatio-Temporal-Mapping` repository?
5. Which recovered simulator files are uncommitted work by Nidhish, and which are generated or copied artifacts?
6. What experiments produced the two `temporal_projector.cpp` variants?
7. What command, dataset, and configuration produced `final_colored_orchard.pcd`?
8. What are the authoritative `ur5e.yml`, `collision_table.yml`, physical UR calibration, and camera-mount calibration sources?
9. Who owns and may redistribute the YOLO/BiSeNet weights and training data?
10. What lab policy governs release of robot calibration, orchard data, imagery, and reconstructed tree assets?
11. Which repository should ultimately own the migrated ROS 2 reconstruction package?
12. What acknowledgements or contributor credits do Divyanth, Dawood, Nidhish, and the lab expect in a future release or paper?

---

## Release Checklist

Before a public repository, paper supplement, or external asset release:

- verify the license at the repository root;
- add NOTICE/ATTRIBUTION documentation;
- preserve upstream license files for vendored code;
- confirm per-asset provenance for meshes, textures, weights, and data;
- remove credentials, private paths, hostnames, and personal identifiers;
- review robot-specific calibration and internal network details with the lab;
- confirm that Cornell permits release of collected orchard imagery and reconstructions;
- separate source from large generated data and experiment outputs; and
- tag the exact commits and parameter snapshots used for reported results.
