#!/usr/bin/env python3
"""Rebuild one saved scan with several point-cloud filter profiles.

The live reconstructor saves an unfiltered, TF-placed PLY for every requested
pose. This script replays those exact inputs, so filter and ICP comparisons do
not require moving the robot or reacquiring sensor data.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
import open3d as o3d

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


PROFILE_NAMES = ("none", "current_radius", "dawood_radius", "statistical")
SCALE_NAMES = ("SN3", "SN2", "SN1", "SN0")
REGISTRATION_MODES = ("tf_only", "tf_icp")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rebuild a saved RGB-D scan with multiple filter profiles using "
            "the exact same pose clouds."
        )
    )
    parser.add_argument(
        "scan_dir",
        type=Path,
        help="Scan directory containing run_config.json and raw/capture_*.",
    )
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=PROFILE_NAMES,
        default=list(PROFILE_NAMES),
        help="Filter profiles to evaluate (default: all).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Output directory. Default: "
            "<scan_dir>/filter_comparisons/<timestamp>."
        ),
    )
    parser.add_argument(
        "--no-icp",
        action="store_true",
        help=(
            "Deprecated compatibility option: run TF-only fusion. "
            "Cannot be combined with --registration-modes."
        ),
    )
    parser.add_argument(
        "--registration-modes",
        nargs="+",
        choices=REGISTRATION_MODES,
        default=None,
        help=(
            "Registration variants to replay. Default: tf_only tf_icp. "
            "Both variants consume the identical selected captures."
        ),
    )
    parser.add_argument(
        "--local-voxel-size-m",
        type=float,
        default=None,
        help="Override the local voxel size saved in run_config.json.",
    )
    parser.add_argument(
        "--global-voxel-size-m",
        type=float,
        default=None,
        help="Override the global voxel size saved in run_config.json.",
    )
    parser.add_argument(
        "--closure-pairs",
        default=None,
        help=(
            "Optional comma-separated request-id pairs for row-center closure, "
            "for example '3:12,13:22,23:32'. By default, repeated TF poses "
            "are detected automatically."
        ),
    )
    parser.add_argument(
        "--closure-translation-tolerance-m",
        type=float,
        default=0.005,
        help="Maximum TF translation difference for automatic closure detection.",
    )
    parser.add_argument(
        "--closure-rotation-tolerance-deg",
        type=float,
        default=1.0,
        help="Maximum TF rotation difference for automatic closure detection.",
    )
    parser.add_argument(
        "--closure-min-frame-gap",
        type=int,
        default=8,
        help=(
            "Minimum frame-index gap for automatic row-center closure "
            "detection (default: 8)."
        ),
    )
    return parser.parse_args()


def load_json(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def discover_captures(scan_dir: Path) -> List[Dict[str, Any]]:
    """Select one saved attempt for each request id.

    Prefer the highest-numbered accepted attempt. If none was accepted, retain
    the latest attempt so a different filter can still be tested on it.
    """

    candidates: Dict[int, List[Dict[str, Any]]] = {}
    for metadata_path in sorted((scan_dir / "raw").glob("capture_*/capture.json")):
        metadata = load_json(metadata_path)
        cloud_path = metadata_path.parent / "raw_target_cloud.ply"
        if not cloud_path.is_file():
            continue
        metadata["_metadata_path"] = str(metadata_path)
        metadata["_cloud_path"] = str(cloud_path)
        candidates.setdefault(int(metadata["request_id"]), []).append(metadata)

    selected = []
    for request_id, attempts in candidates.items():
        attempts.sort(key=lambda item: int(item.get("attempt", 1)))
        accepted_attempts = [
            item for item in attempts if item.get("accepted") is True
        ]
        selected.append((accepted_attempts or attempts)[-1])
    selected.sort(key=lambda item: int(item["request_id"]))
    return selected


def profile_parameters(
    profile_name: str, parameters: Dict[str, Any]
) -> Dict[str, Any]:
    local_voxel = float(parameters.get("local_voxel_size_m", 0.004))
    if profile_name == "none":
        return {
            "voxel_size_m": local_voxel,
            "mode": "none",
        }
    if profile_name == "current_radius":
        return {
            "voxel_size_m": local_voxel,
            "mode": "radius",
            "radius_m": float(parameters.get("outlier_radius_m", 0.020)),
            "min_neighbors": int(
                parameters.get("outlier_min_neighbors", 6)
            ),
        }
    if profile_name == "dawood_radius":
        return {
            "voxel_size_m": 0.008,
            "mode": "radius",
            "radius_m": 0.014,
            "min_neighbors": 8,
        }
    if profile_name == "statistical":
        return {
            "voxel_size_m": local_voxel,
            "mode": "statistical",
            "nb_neighbors": int(
                parameters.get("statistical_nb_neighbors", 20)
            ),
            "std_ratio": float(
                parameters.get("statistical_std_ratio", 2.0)
            ),
        }
    raise ValueError(f"Unknown profile: {profile_name}")


def crop_saved_cloud(
    cloud: o3d.geometry.PointCloud, metadata: Dict[str, Any]
) -> o3d.geometry.PointCloud:
    bounds = metadata.get("target_crop_bounds_m")
    if not bounds:
        return cloud
    minimum = np.asarray(bounds["min"], dtype=np.float64)
    maximum = np.asarray(bounds["max"], dtype=np.float64)
    return cloud.crop(o3d.geometry.AxisAlignedBoundingBox(minimum, maximum))


def filter_cloud(
    cloud: o3d.geometry.PointCloud, profile: Dict[str, Any]
) -> o3d.geometry.PointCloud:
    voxel = float(profile["voxel_size_m"])
    filtered = (
        cloud.voxel_down_sample(voxel)
        if voxel > 0.0
        else copy.deepcopy(cloud)
    )
    if len(filtered.points) == 0 or profile["mode"] == "none":
        return filtered

    if profile["mode"] == "radius":
        _, indices = filtered.remove_radius_outlier(
            nb_points=int(profile["min_neighbors"]),
            radius=float(profile["radius_m"]),
        )
    elif profile["mode"] == "statistical":
        _, indices = filtered.remove_statistical_outlier(
            nb_neighbors=int(profile["nb_neighbors"]),
            std_ratio=float(profile["std_ratio"]),
        )
    else:
        raise ValueError(f"Unsupported filter mode: {profile['mode']}")
    return filtered.select_by_index(indices)


def local_icp_target(
    source: o3d.geometry.PointCloud,
    global_cloud: o3d.geometry.PointCloud,
    margin_m: float,
    minimum_points: int,
) -> o3d.geometry.PointCloud:
    source_box = source.get_axis_aligned_bounding_box()
    minimum = source_box.get_min_bound() - margin_m
    maximum = source_box.get_max_bound() + margin_m
    target = global_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(minimum, maximum)
    )
    return target if len(target.points) >= minimum_points else global_cloud


def multiscale_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    scales: Dict[str, Dict[str, float]],
) -> Tuple[bool, np.ndarray, Optional[float], Optional[float], str]:
    transform = np.eye(4, dtype=np.float64)
    final_fitness: Optional[float] = None
    final_rmse: Optional[float] = None

    for name in SCALE_NAMES:
        params = scales[name]
        voxel = float(params["voxel"])
        correspondence = float(params["correspondence"])
        if voxel > 0.0:
            source_scale = source.voxel_down_sample(voxel)
            target_scale = target.voxel_down_sample(voxel)
            normal_radius = max(voxel * 2.5, correspondence * 1.5)
        else:
            source_scale = copy.deepcopy(source)
            target_scale = copy.deepcopy(target)
            normal_radius = max(0.03, correspondence * 2.0)

        if len(source_scale.points) < 30 or len(target_scale.points) < 30:
            return (
                False,
                transform,
                final_fitness,
                final_rmse,
                f"{name}: fewer than 30 source or target points",
            )

        normal_search = o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius, max_nn=40
        )
        source_scale.estimate_normals(search_param=normal_search)
        target_scale.estimate_normals(search_param=normal_search)
        registration = o3d.pipelines.registration.registration_icp(
            source=source_scale,
            target=target_scale,
            max_correspondence_distance=correspondence,
            init=transform,
            estimation_method=(
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            ),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_rmse=float(params["relative_rmse"]),
                relative_fitness=float(params["relative_fitness"]),
                max_iteration=int(params["iterations"]),
            ),
        )
        transform = registration.transformation
        final_fitness = float(registration.fitness)
        final_rmse = float(registration.inlier_rmse)
        if (
            final_fitness < float(params["fitness_threshold"])
            or final_rmse > float(params["rmse_threshold"])
        ):
            return (
                False,
                transform,
                final_fitness,
                final_rmse,
                f"{name}: ICP threshold failure",
            )

    return True, transform, final_fitness, final_rmse, "ICP accepted"


def correction_is_sane(
    transform: np.ndarray,
    maximum_translation_m: float,
    maximum_rotation_deg: float,
) -> Tuple[bool, float, float]:
    translation = float(np.linalg.norm(transform[:3, 3]))
    trace_term = float((np.trace(transform[:3, :3]) - 1.0) / 2.0)
    rotation_deg = math.degrees(
        math.acos(float(np.clip(trace_term, -1.0, 1.0)))
    )
    return (
        translation <= maximum_translation_m
        and rotation_deg <= maximum_rotation_deg,
        translation,
        rotation_deg,
    )


def validate_transform(value: Any, description: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{description} is not a finite 4x4 transform")
    return transform


def selected_tf_transform(metadata: Dict[str, Any]) -> Optional[np.ndarray]:
    """Return target<-camera TF saved for the selected burst sample."""

    direct = metadata.get("tf_target_from_camera")
    if direct is not None:
        return validate_transform(
            direct,
            f"capture {metadata.get('request_id')} tf_target_from_camera",
        )

    samples = metadata.get("samples")
    if not isinstance(samples, list) or not samples:
        return None
    selected_index = int(
        metadata.get("selected_sample_index", len(samples) // 2)
    )
    if selected_index < 0 or selected_index >= len(samples):
        raise ValueError(
            f"Capture {metadata.get('request_id')} has invalid "
            f"selected_sample_index={selected_index}"
        )
    value = samples[selected_index].get("tf_target_from_camera")
    if value is None:
        return None
    return validate_transform(
        value,
        f"capture {metadata.get('request_id')} selected-sample TF",
    )


def rotation_angle_deg(rotation: np.ndarray) -> float:
    trace_term = float((np.trace(rotation) - 1.0) / 2.0)
    return math.degrees(math.acos(float(np.clip(trace_term, -1.0, 1.0))))


def rotation_matrix_to_rpy_deg(rotation: np.ndarray) -> Tuple[float, float, float]:
    """Return fixed-axis XYZ roll, pitch, yaw in degrees."""

    sy = math.hypot(float(rotation[0, 0]), float(rotation[1, 0]))
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        pitch = math.atan2(-float(rotation[2, 0]), sy)
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        pitch = math.atan2(-float(rotation[2, 0]), sy)
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def relative_pose_error(
    first: np.ndarray, second: np.ndarray
) -> Tuple[float, float]:
    relative = np.linalg.inv(first) @ second
    return (
        float(np.linalg.norm(relative[:3, 3])),
        rotation_angle_deg(relative[:3, :3]),
    )


def normalized_scales(
    run_config: Dict[str, Any]
) -> Dict[str, Dict[str, float]]:
    scales = run_config.get("registration_scales")
    if not scales:
        raise ValueError("run_config.json has no registration_scales")
    return scales


def fuse_profile(
    *,
    profile_name: str,
    captures: Iterable[Dict[str, Any]],
    run_config: Dict[str, Any],
    registration_mode: str,
    output_dir: Path,
) -> Dict[str, Any]:
    if registration_mode not in REGISTRATION_MODES:
        raise ValueError(f"Unsupported registration mode: {registration_mode}")
    use_icp = registration_mode == "tf_icp"
    parameters = run_config["parameters"]
    profile = profile_parameters(profile_name, parameters)
    scales = normalized_scales(run_config)
    global_cloud = o3d.geometry.PointCloud()
    capture_results: List[Dict[str, Any]] = []
    accepted = 0
    rejected = 0
    fitnesses = []
    rmses = []
    start = time.perf_counter()

    for frame_index, metadata in enumerate(captures, start=1):
        request_id = int(metadata["request_id"])
        tf_transform = selected_tf_transform(metadata)
        cloud = o3d.io.read_point_cloud(metadata["_cloud_path"])
        raw_points = len(cloud.points)
        cloud = crop_saved_cloud(cloud, metadata)
        cropped_points = len(cloud.points)
        filtered = filter_cloud(cloud, profile)
        filtered_points = len(filtered.points)
        transform = np.eye(4, dtype=np.float64)
        fitness: Optional[float] = None
        rmse: Optional[float] = None
        reason = "accepted"
        is_accepted = filtered_points > 0

        if not is_accepted:
            reason = "empty after filter"
        elif len(global_cloud.points) > 0 and use_icp:
            target = local_icp_target(
                filtered,
                global_cloud,
                float(parameters.get("icp_crop_margin_m", 0.15)),
                int(parameters.get("icp_min_target_points", 200)),
            )
            is_accepted, transform, fitness, rmse, reason = multiscale_icp(
                filtered, target, scales
            )
            if is_accepted:
                sane, correction_m, correction_deg = correction_is_sane(
                    transform,
                    float(
                        parameters.get(
                            "max_icp_correction_translation_m", 0.10
                        )
                    ),
                    float(
                        parameters.get(
                            "max_icp_correction_rotation_deg", 12.0
                        )
                    ),
                )
                if not sane:
                    is_accepted = False
                    reason = (
                        "ICP correction limit failure "
                        f"({correction_m:.4f} m, {correction_deg:.2f} deg)"
                    )

        if is_accepted:
            corrected = copy.deepcopy(filtered)
            corrected.transform(transform)
            if len(global_cloud.points) == 0:
                global_cloud = corrected
            else:
                global_cloud += corrected
            global_voxel = float(
                parameters.get("global_voxel_size_m", 0.004)
            )
            if global_voxel > 0.0:
                global_cloud = global_cloud.voxel_down_sample(global_voxel)
            accepted += 1
            if fitness is not None:
                fitnesses.append(fitness)
            if rmse is not None:
                rmses.append(rmse)
        else:
            rejected += 1

        correction_translation = transform[:3, 3]
        correction_rpy_deg = rotation_matrix_to_rpy_deg(transform[:3, :3])
        correction_translation_m = float(
            np.linalg.norm(correction_translation)
        )
        correction_rotation_deg = rotation_angle_deg(transform[:3, :3])
        final_pose = (
            transform @ tf_transform
            if tf_transform is not None
            else None
        )
        capture_results.append(
            {
                "frame_index": frame_index,
                "request_id": request_id,
                "accepted": is_accepted,
                "reason": reason,
                "raw_points": raw_points,
                "cropped_points": cropped_points,
                "filtered_points": filtered_points,
                "fitness": fitness,
                "inlier_rmse_m": rmse,
                "icp_correction": transform.tolist(),
                "icp_correction_translation_m": (
                    correction_translation.tolist()
                ),
                "icp_correction_translation_norm_m": (
                    correction_translation_m
                ),
                "icp_correction_rpy_deg": list(correction_rpy_deg),
                "icp_correction_rotation_deg": correction_rotation_deg,
                "tf_target_from_camera": (
                    tf_transform.tolist()
                    if tf_transform is not None
                    else None
                ),
                "final_target_from_camera": (
                    final_pose.tolist() if final_pose is not None else None
                ),
            }
        )

    output_dir.mkdir(parents=True, exist_ok=False)
    cloud_path = output_dir / "global_point_cloud.ply"
    if len(global_cloud.points) > 0:
        if not o3d.io.write_point_cloud(
            str(cloud_path),
            global_cloud,
            write_ascii=False,
            compressed=False,
        ):
            raise OSError(f"Failed to write {cloud_path}")

    elapsed = time.perf_counter() - start
    metrics = {
        "profile": profile_name,
        "profile_parameters": profile,
        "global_voxel_size_m": float(
            parameters.get("global_voxel_size_m", 0.005)
        ),
        "registration_mode": registration_mode,
        "use_icp": use_icp,
        "capture_count": accepted + rejected,
        "accepted_captures": accepted,
        "rejected_captures": rejected,
        "accepted_percent": (
            100.0 * accepted / (accepted + rejected)
            if accepted + rejected
            else 0.0
        ),
        "final_point_count": len(global_cloud.points),
        "mean_icp_fitness": (
            float(np.mean(fitnesses)) if fitnesses else None
        ),
        "mean_icp_rmse_mm": (
            float(np.mean(rmses) * 1000.0) if rmses else None
        ),
        "processing_time_sec": elapsed,
        "cloud_path": str(cloud_path) if len(global_cloud.points) else None,
        "captures": capture_results,
    }
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)
    return metrics


def write_frame_metrics_csv(
    path: Path, captures: List[Dict[str, Any]]
) -> None:
    fieldnames = [
        "frame_index",
        "request_id",
        "accepted",
        "reason",
        "correction_x_mm",
        "correction_y_mm",
        "correction_z_mm",
        "correction_translation_mm",
        "correction_roll_deg",
        "correction_pitch_deg",
        "correction_yaw_deg",
        "correction_rotation_deg",
        "fitness",
        "inlier_rmse_mm",
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for capture in captures:
            translation = capture["icp_correction_translation_m"]
            roll, pitch, yaw = capture["icp_correction_rpy_deg"]
            rmse = capture.get("inlier_rmse_m")
            writer.writerow(
                {
                    "frame_index": capture["frame_index"],
                    "request_id": capture["request_id"],
                    "accepted": capture["accepted"],
                    "reason": capture["reason"],
                    "correction_x_mm": 1000.0 * translation[0],
                    "correction_y_mm": 1000.0 * translation[1],
                    "correction_z_mm": 1000.0 * translation[2],
                    "correction_translation_mm": (
                        1000.0
                        * capture["icp_correction_translation_norm_m"]
                    ),
                    "correction_roll_deg": roll,
                    "correction_pitch_deg": pitch,
                    "correction_yaw_deg": yaw,
                    "correction_rotation_deg": (
                        capture["icp_correction_rotation_deg"]
                    ),
                    "fitness": capture.get("fitness"),
                    "inlier_rmse_mm": (
                        None if rmse is None else 1000.0 * rmse
                    ),
                }
            )


def save_diagnostic_plots(
    output_dir: Path,
    profile_name: str,
    captures: List[Dict[str, Any]],
) -> None:
    if plt is None:
        print(
            "WARNING: matplotlib is not installed; diagnostic CSV files "
            "were written, but PNG plots were skipped."
        )
        return

    frames = np.asarray(
        [capture["frame_index"] for capture in captures], dtype=np.int64
    )
    translations = 1000.0 * np.asarray(
        [
            capture["icp_correction_translation_m"]
            for capture in captures
        ],
        dtype=np.float64,
    )
    translation_norms = 1000.0 * np.asarray(
        [
            capture["icp_correction_translation_norm_m"]
            for capture in captures
        ],
        dtype=np.float64,
    )
    rotations = np.asarray(
        [capture["icp_correction_rpy_deg"] for capture in captures],
        dtype=np.float64,
    )
    rotation_norms = np.asarray(
        [capture["icp_correction_rotation_deg"] for capture in captures],
        dtype=np.float64,
    )
    fitness = np.asarray(
        [
            np.nan if capture.get("fitness") is None else capture["fitness"]
            for capture in captures
        ],
        dtype=np.float64,
    )
    rmse_mm = 1000.0 * np.asarray(
        [
            (
                np.nan
                if capture.get("inlier_rmse_m") is None
                else capture["inlier_rmse_m"]
            )
            for capture in captures
        ],
        dtype=np.float64,
    )

    figure, axis = plt.subplots(figsize=(10, 5.5))
    axis.plot(frames, translations[:, 0], label="X", marker=".", linewidth=1)
    axis.plot(frames, translations[:, 1], label="Y", marker=".", linewidth=1)
    axis.plot(frames, translations[:, 2], label="Z", marker=".", linewidth=1)
    axis.plot(
        frames,
        translation_norms,
        label="Magnitude",
        color="black",
        linewidth=2,
    )
    axis.axhline(0.0, color="0.65", linewidth=0.8)
    axis.set(
        xlabel="Frame index",
        ylabel="ICP translation correction (mm)",
        title=f"{profile_name}: translation correction versus frame",
    )
    axis.grid(alpha=0.25)
    axis.legend(ncol=4)
    figure.tight_layout()
    figure.savefig(output_dir / "translation_correction_vs_frame.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 5.5))
    axis.plot(frames, rotations[:, 0], label="Roll", marker=".", linewidth=1)
    axis.plot(frames, rotations[:, 1], label="Pitch", marker=".", linewidth=1)
    axis.plot(frames, rotations[:, 2], label="Yaw", marker=".", linewidth=1)
    axis.plot(
        frames,
        rotation_norms,
        label="Angle magnitude",
        color="black",
        linewidth=2,
    )
    axis.axhline(0.0, color="0.65", linewidth=0.8)
    axis.set(
        xlabel="Frame index",
        ylabel="ICP rotation correction (deg)",
        title=f"{profile_name}: rotation correction versus frame",
    )
    axis.grid(alpha=0.25)
    axis.legend(ncol=4)
    figure.tight_layout()
    figure.savefig(output_dir / "rotation_correction_vs_frame.png", dpi=180)
    plt.close(figure)

    figure, fitness_axis = plt.subplots(figsize=(10, 5.5))
    rmse_axis = fitness_axis.twinx()
    fitness_line = fitness_axis.plot(
        frames,
        fitness,
        color="tab:blue",
        marker="o",
        markersize=3,
        label="Fitness",
    )
    rmse_line = rmse_axis.plot(
        frames,
        rmse_mm,
        color="tab:red",
        marker="s",
        markersize=3,
        label="Inlier RMSE",
    )
    fitness_axis.set(
        xlabel="Frame index",
        ylabel="ICP fitness",
        title=f"{profile_name}: ICP quality versus frame",
    )
    rmse_axis.set_ylabel("ICP inlier RMSE (mm)")
    fitness_axis.grid(alpha=0.25)
    lines = fitness_line + rmse_line
    fitness_axis.legend(lines, [line.get_label() for line in lines])
    figure.tight_layout()
    figure.savefig(output_dir / "fitness_rmse_vs_frame.png", dpi=180)
    plt.close(figure)


def parse_closure_pairs(value: Optional[str]) -> Optional[List[Tuple[int, int]]]:
    if value is None:
        return None
    pairs: List[Tuple[int, int]] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        parts = token.split(":")
        if len(parts) != 2:
            raise ValueError(
                f"Invalid closure pair {token!r}; expected START:END"
            )
        first, second = (int(part) for part in parts)
        if first == second:
            raise ValueError(f"Closure pair {token!r} repeats one request id")
        pairs.append((first, second))
    if not pairs:
        raise ValueError("--closure-pairs did not contain any pairs")
    return pairs


def detect_closure_pairs(
    captures: List[Dict[str, Any]],
    translation_tolerance_m: float,
    rotation_tolerance_deg: float,
    minimum_frame_gap: int,
) -> List[Tuple[int, int]]:
    """Find later revisits of a TF pose, preferring the latest valid start."""

    detected: List[Tuple[int, int]] = []
    for end_index, end in enumerate(captures):
        if not end.get("accepted") or end.get("tf_target_from_camera") is None:
            continue
        end_pose = validate_transform(
            end["tf_target_from_camera"],
            f"capture {end['request_id']} TF",
        )
        matching_starts: List[Dict[str, Any]] = []
        for start in captures[:end_index]:
            if (
                not start.get("accepted")
                or start.get("tf_target_from_camera") is None
                or (
                    int(end["frame_index"]) - int(start["frame_index"])
                    < minimum_frame_gap
                )
            ):
                continue
            start_pose = validate_transform(
                start["tf_target_from_camera"],
                f"capture {start['request_id']} TF",
            )
            translation_m, rotation_deg = relative_pose_error(
                start_pose, end_pose
            )
            if (
                translation_m <= translation_tolerance_m
                and rotation_deg <= rotation_tolerance_deg
            ):
                matching_starts.append(start)
        if matching_starts:
            # For Row 0 this chooses frame 3 rather than setup frame 1.
            start = max(
                matching_starts, key=lambda item: int(item["frame_index"])
            )
            detected.append(
                (int(start["request_id"]), int(end["request_id"]))
            )
    return detected


def calculate_closure_errors(
    captures: List[Dict[str, Any]],
    pairs: List[Tuple[int, int]],
) -> List[Dict[str, Any]]:
    by_request = {int(item["request_id"]): item for item in captures}
    closures: List[Dict[str, Any]] = []
    for row_index, (start_id, end_id) in enumerate(pairs):
        if start_id not in by_request or end_id not in by_request:
            raise ValueError(
                f"Closure pair {start_id}:{end_id} is not present in the "
                "selected captures"
            )
        start = by_request[start_id]
        end = by_request[end_id]
        if (
            start.get("tf_target_from_camera") is None
            or end.get("tf_target_from_camera") is None
        ):
            raise ValueError(
                f"Closure pair {start_id}:{end_id} has no saved TF pose"
            )
        if (
            start.get("final_target_from_camera") is None
            or end.get("final_target_from_camera") is None
        ):
            raise ValueError(
                f"Closure pair {start_id}:{end_id} has no final camera pose"
            )
        tf_translation_m, tf_rotation_deg = relative_pose_error(
            validate_transform(
                start["tf_target_from_camera"], f"capture {start_id} TF"
            ),
            validate_transform(
                end["tf_target_from_camera"], f"capture {end_id} TF"
            ),
        )
        final_translation_m, final_rotation_deg = relative_pose_error(
            validate_transform(
                start["final_target_from_camera"],
                f"capture {start_id} final pose",
            ),
            validate_transform(
                end["final_target_from_camera"],
                f"capture {end_id} final pose",
            ),
        )
        closures.append(
            {
                "row_index": row_index,
                "start_request_id": start_id,
                "end_request_id": end_id,
                "frame_gap": (
                    int(end["frame_index"]) - int(start["frame_index"])
                ),
                "tf_translation_error_mm": 1000.0 * tf_translation_m,
                "tf_rotation_error_deg": tf_rotation_deg,
                "tf_icp_translation_error_mm": (
                    1000.0 * final_translation_m
                ),
                "tf_icp_rotation_error_deg": final_rotation_deg,
                "translation_error_change_mm": (
                    1000.0 * (final_translation_m - tf_translation_m)
                ),
                "rotation_error_change_deg": (
                    final_rotation_deg - tf_rotation_deg
                ),
            }
        )
    return closures


def write_closure_csv(path: Path, closures: List[Dict[str, Any]]) -> None:
    fieldnames = [
        "row_index",
        "start_request_id",
        "end_request_id",
        "frame_gap",
        "tf_translation_error_mm",
        "tf_rotation_error_deg",
        "tf_icp_translation_error_mm",
        "tf_icp_rotation_error_deg",
        "translation_error_change_mm",
        "rotation_error_change_deg",
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(closures)


def save_closure_plot(
    path: Path, profile_name: str, closures: List[Dict[str, Any]]
) -> None:
    if plt is None or not closures:
        return
    labels = [
        f"{item['start_request_id']}->{item['end_request_id']}"
        for item in closures
    ]
    positions = np.arange(len(closures), dtype=np.float64)
    width = 0.36
    tf_translation = [
        item["tf_translation_error_mm"] for item in closures
    ]
    corrected_translation = [
        item["tf_icp_translation_error_mm"] for item in closures
    ]
    tf_rotation = [item["tf_rotation_error_deg"] for item in closures]
    corrected_rotation = [
        item["tf_icp_rotation_error_deg"] for item in closures
    ]

    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].bar(
        positions - width / 2,
        tf_translation,
        width,
        label="TF-only pose",
    )
    axes[0].bar(
        positions + width / 2,
        corrected_translation,
        width,
        label="TF+ICP pose",
    )
    axes[0].set_ylabel("Translation closure (mm)")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()

    axes[1].bar(
        positions - width / 2,
        tf_rotation,
        width,
        label="TF-only pose",
    )
    axes[1].bar(
        positions + width / 2,
        corrected_rotation,
        width,
        label="TF+ICP pose",
    )
    axes[1].set_ylabel("Rotation closure (deg)")
    axes[1].set_xlabel("Repeated row-center request IDs")
    axes[1].set_xticks(positions, labels)
    axes[1].grid(axis="y", alpha=0.25)
    figure.suptitle(f"{profile_name}: row-center closure errors")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def write_comparison_csv(path: Path, results: List[Dict[str, Any]]) -> None:
    fieldnames = [
        "profile",
        "registration_mode",
        "use_icp",
        "capture_count",
        "accepted_captures",
        "rejected_captures",
        "accepted_percent",
        "final_point_count",
        "mean_icp_fitness",
        "mean_icp_rmse_mm",
        "processing_time_sec",
        "cloud_path",
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow({key: result.get(key) for key in fieldnames})


def main() -> None:
    args = parse_args()
    if args.no_icp and args.registration_modes is not None:
        raise ValueError(
            "--no-icp cannot be combined with --registration-modes"
        )
    registration_modes = (
        ["tf_only"]
        if args.no_icp
        else (
            args.registration_modes
            if args.registration_modes is not None
            else ["tf_only", "tf_icp"]
        )
    )
    manual_closure_pairs = parse_closure_pairs(args.closure_pairs)

    scan_dir = args.scan_dir.expanduser().resolve()
    run_config_path = scan_dir / "run_config.json"
    if not run_config_path.is_file():
        raise FileNotFoundError(f"Missing {run_config_path}")

    captures = discover_captures(scan_dir)
    if not captures:
        raise RuntimeError(
            f"No raw/capture_*/raw_target_cloud.ply inputs found in {scan_dir}"
        )
    run_config = load_json(run_config_path)
    parameters = run_config["parameters"].copy()
    run_config["parameters"] = parameters

    if args.local_voxel_size_m is not None:
        parameters["local_voxel_size_m"] = args.local_voxel_size_m

    if args.global_voxel_size_m is not None:
        parameters["global_voxel_size_m"] = args.global_voxel_size_m

    local_voxel_m = float(parameters.get("local_voxel_size_m", 0.004))
    global_voxel_m = float(parameters.get("global_voxel_size_m", 0.004))
    if local_voxel_m < 0.0 or global_voxel_m < 0.0:
        raise ValueError("Voxel sizes must be non-negative")
    local_voxel_mm = local_voxel_m * 1e3
    global_voxel_mm = global_voxel_m * 1e3

    timestamp = time.strftime("%Y%m%d_%H%M%S")

    output_root = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else (
            scan_dir
            / "filter_comparisons"
            / (
                f"{timestamp}_"
                f"voxel_{local_voxel_mm:g}mm_local_"
                f"{global_voxel_mm:g}mm_global"
            )
        )
    )

    output_root.mkdir(parents=True, exist_ok=False)

    manifest = {
        "source_scan_dir": str(scan_dir),
        "request_ids": [int(item["request_id"]) for item in captures],
        "capture_metadata_paths": [
            item["_metadata_path"] for item in captures
        ],
        "profiles": list(args.profiles),
        "registration_modes": registration_modes,
        "local_voxel_size_m": local_voxel_m,
        "global_voxel_size_m": global_voxel_m,
        "identical_capture_policy": (
            "Every profile and registration mode replays this exact ordered "
            "selected-capture list."
        ),
    }
    with open(
        output_root / "replay_manifest.json", "w", encoding="utf-8"
    ) as handle:
        json.dump(manifest, handle, indent=2)

    results: List[Dict[str, Any]] = []
    for profile_name in args.profiles:
        profile_results: Dict[str, Dict[str, Any]] = {}
        for registration_mode in registration_modes:
            print(
                f"[{profile_name}/{registration_mode}] rebuilding "
                f"{len(captures)} captures..."
            )
            result = fuse_profile(
                profile_name=profile_name,
                captures=captures,
                run_config=run_config,
                registration_mode=registration_mode,
                output_dir=(
                    output_root / profile_name / registration_mode
                ),
            )
            results.append(result)
            profile_results[registration_mode] = result
            write_frame_metrics_csv(
                output_root
                / profile_name
                / registration_mode
                / "frame_metrics.csv",
                result["captures"],
            )

        icp_result = profile_results.get("tf_icp")
        if icp_result is not None:
            icp_output_dir = output_root / profile_name / "tf_icp"
            save_diagnostic_plots(
                icp_output_dir,
                profile_name,
                icp_result["captures"],
            )
            try:
                closure_pairs = (
                    manual_closure_pairs
                    if manual_closure_pairs is not None
                    else detect_closure_pairs(
                        icp_result["captures"],
                        args.closure_translation_tolerance_m,
                        args.closure_rotation_tolerance_deg,
                        args.closure_min_frame_gap,
                    )
                )
                closures = calculate_closure_errors(
                    icp_result["captures"], closure_pairs
                )
            except ValueError:
                if manual_closure_pairs is not None:
                    raise
                closures = []
            if closures:
                closure_path = (
                    output_root / profile_name / "row_center_closure.csv"
                )
                write_closure_csv(closure_path, closures)
                save_closure_plot(
                    output_root
                    / profile_name
                    / "row_center_closure_errors.png",
                    profile_name,
                    closures,
                )
            else:
                print(
                    f"WARNING: [{profile_name}] no repeated row-center TF "
                    "poses were detected. Supply --closure-pairs "
                    "START:END,... if the request IDs are known."
                )

    comparison_path = output_root / "comparison.csv"
    write_comparison_csv(comparison_path, results)
    print(f"Comparison complete: {comparison_path}")


if __name__ == "__main__":
    main()
