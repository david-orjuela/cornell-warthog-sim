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


PROFILE_NAMES = ("none", "current_radius", "dawood_radius", "statistical")
SCALE_NAMES = ("SN3", "SN2", "SN1", "SN0")


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
        help="Use TF-only fusion for every profile.",
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
    use_icp: bool,
    output_dir: Path,
) -> Dict[str, Any]:
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

    for metadata in captures:
        request_id = int(metadata["request_id"])
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

        capture_results.append(
            {
                "request_id": request_id,
                "accepted": is_accepted,
                "reason": reason,
                "raw_points": raw_points,
                "cropped_points": cropped_points,
                "filtered_points": filtered_points,
                "fitness": fitness,
                "inlier_rmse_m": rmse,
                "icp_correction": transform.tolist(),
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


def write_comparison_csv(path: Path, results: List[Dict[str, Any]]) -> None:
    fieldnames = [
        "profile",
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
    parameters = run_config["parameters"]

    if args.local_voxel_size_m is not None:
            parameters["local_voxel_size_m"] = args.local_voxel_size_m
    
    if args.global_voxel_size_m is not None:
        parameters["global_voxel_size_m"] = args.global_voxel_size_m

    local_voxel_mm = parameters["local_voxel_size_m"] * 1e3
    global_voxel_mm = parameters["global_voxel_size_m"] * 1e3

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

    use_icp = bool(parameters.get("use_icp", True)) and not args.no_icp

    results = []
    for profile_name in args.profiles:
        print(f"[{profile_name}] rebuilding {len(captures)} captures...")
        results.append(
            fuse_profile(
                profile_name=profile_name,
                captures=captures,
                run_config=run_config,
                use_icp=use_icp,
                output_dir=output_root / profile_name,
            )
        )

    comparison_path = output_root / "comparison.csv"
    write_comparison_csv(comparison_path, results)
    print(f"Comparison complete: {comparison_path}")


if __name__ == "__main__":
    main()
