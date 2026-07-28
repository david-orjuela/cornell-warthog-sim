#!/usr/bin/env python3
"""Compare TF-only, incremental ICP, and TF-anchored pose-graph fusion.

Place this script beside ``compare_filters.py``.  It deliberately reuses that
script's capture selection, target cropping, filter profiles, ICP parameters,
metric conventions, and baseline fusion implementation.

The added experiment has four parts:

1. Rebuild the chronological TF-only and incremental TF+ICP baselines.
2. Replay incremental ICP in reverse and seeded shuffled orders.
3. Register TF-predicted overlapping capture pairs with robust point-to-plane
   ICP, optimize a graph containing absolute TF priors, and fuse once.
4. Compare pose corrections, closures, held-out edges, and fixed visual views
   from the identical selected capture set.

Transform convention
--------------------
``T_base_camera`` maps camera-frame points into ``base_link``.  A geometric
edge from camera i to camera j stores ``T_camera_j_camera_i``.  Open3D graph
node poses store ``T_base_camera``.  Node 0 is a fixed identity ``base_link``
node; the TF prior for capture i is therefore an edge 0 -> i whose measured
transform is ``T_camera_i_base = inverse(T_base_camera_i)``.
"""

from __future__ import annotations

import argparse
import colorsys
import copy
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d

try:
    import compare_filters as baseline
except ImportError as exc:
    raise ImportError(
        "compare_registration.py must be placed beside compare_filters.py"
    ) from exc


GRAPH_PRIORS: Dict[str, Tuple[float, float]] = {
    # Nominal one-sigma values used to construct diagonal information matrices.
    # These are an experiment sweep, not claims about calibrated UR5e accuracy.
    "loose": (0.020, 2.0),
    "nominal": (0.010, 1.0),
    "strong": (0.005, 0.5),
}


@dataclass(frozen=True)
class PairCandidate:
    source_index: int
    target_index: int
    kind: str
    overlap_score: float
    held_out: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a same-capture comparison of TF-only fusion, current "
            "incremental ICP, and TF-anchored batch pose-graph fusion."
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
        choices=baseline.PROFILE_NAMES,
        default=["statistical"],
        help=(
            "Filter profiles to evaluate. Default: statistical. Use more than "
            "one only when filter choice is part of the experiment."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Output directory. Default: "
            "<scan_dir>/registration_comparisons/<timestamp>."
        ),
    )
    parser.add_argument(
        "--local-voxel-size-m",
        type=float,
        default=None,
        help="Override the saved local voxel size for every variant.",
    )
    parser.add_argument(
        "--global-voxel-size-m",
        type=float,
        default=None,
        help="Override the saved final/global voxel size for every variant.",
    )
    parser.add_argument(
        "--closure-pairs",
        default=None,
        help=(
            "Comma-separated repeated-pose request IDs, for example "
            "'3:12,13:22,23:32'. Default: detect from TF."
        ),
    )
    parser.add_argument(
        "--closure-translation-tolerance-m",
        type=float,
        default=0.005,
        help="TF translation tolerance for automatic closure detection.",
    )
    parser.add_argument(
        "--closure-rotation-tolerance-deg",
        type=float,
        default=1.0,
        help="TF rotation tolerance for automatic closure detection.",
    )
    parser.add_argument(
        "--closure-min-frame-gap",
        type=int,
        default=8,
        help="Minimum frame gap for automatic closure detection.",
    )

    order = parser.add_argument_group("incremental ICP order test")
    order.add_argument(
        "--skip-order-test",
        action="store_true",
        help="Skip reverse and shuffled incremental-ICP replays.",
    )
    order.add_argument(
        "--shuffle-count",
        type=int,
        default=3,
        help="Number of deterministic shuffled replays (default: 3).",
    )
    order.add_argument(
        "--random-seed",
        type=int,
        default=23,
        help="Seed for shuffled orders and held-out edges (default: 23).",
    )

    graph = parser.add_argument_group("TF-anchored pose graph")
    graph.add_argument(
        "--graph-priors",
        nargs="+",
        choices=tuple(GRAPH_PRIORS),
        default=list(GRAPH_PRIORS),
        help="TF prior strengths to sweep (default: loose nominal strong).",
    )
    graph.add_argument(
        "--graph-scales",
        nargs="+",
        choices=baseline.SCALE_NAMES,
        default=["SN2", "SN1", "SN0"],
        help=(
            "ICP scales for graph edges. Default omits the 50 mm SN3 stage "
            "because relative TF already supplies the initialization."
        ),
    )
    graph.add_argument(
        "--graph-min-aabb-overlap",
        type=float,
        default=0.05,
        help=(
            "Minimum TF-placed AABB intersection/min-volume ratio for an "
            "additional non-neighbor candidate (default: 0.05)."
        ),
    )
    graph.add_argument(
        "--graph-extra-neighbors",
        type=int,
        default=3,
        help=(
            "Maximum selected extra overlap candidates incident on a capture "
            "(default: 3). Adjacent and closure pairs are always retained."
        ),
    )
    graph.add_argument(
        "--graph-holdout-fraction",
        type=float,
        default=0.20,
        help=(
            "Fraction of extra overlap candidates withheld from optimization "
            "for independent evaluation (default: 0.20)."
        ),
    )
    graph.add_argument(
        "--graph-max-correction-translation-m",
        type=float,
        default=0.030,
        help="Maximum accepted edge displacement from relative TF.",
    )
    graph.add_argument(
        "--graph-max-correction-rotation-deg",
        type=float,
        default=5.0,
        help="Maximum accepted edge rotation from relative TF.",
    )
    graph.add_argument(
        "--robust-kernel-k-fraction",
        type=float,
        default=0.5,
        help=(
            "Tukey k as a fraction of each correspondence distance "
            "(default: 0.5)."
        ),
    )
    graph.add_argument(
        "--robust-kernel-min-m",
        type=float,
        default=0.004,
        help="Lower bound for Tukey k (default: 0.004 m).",
    )
    graph.add_argument(
        "--graph-edge-information-scale",
        type=float,
        default=1.0,
        help="Multiplier applied to geometric edge information matrices.",
    )
    graph.add_argument(
        "--graph-edge-prune-threshold",
        type=float,
        default=0.25,
        help="Open3D uncertain-edge line-process pruning threshold.",
    )
    graph.add_argument(
        "--graph-loop-closure-preference",
        type=float,
        default=1.0,
        help="Open3D balance between certain and uncertain edges.",
    )

    evaluation = parser.add_argument_group("numerical and visual evaluation")
    evaluation.add_argument(
        "--evaluation-voxel-size-m",
        type=float,
        default=0.010,
        help="Voxel size for held-out geometric error (default: 0.010 m).",
    )
    evaluation.add_argument(
        "--evaluation-max-distance-m",
        type=float,
        default=0.025,
        help="Nearest-neighbor limit for held-out error (default: 0.025 m).",
    )
    evaluation.add_argument(
        "--evaluation-trim-fraction",
        type=float,
        default=0.90,
        help="Lowest residual fraction retained for trimmed error.",
    )
    evaluation.add_argument(
        "--skip-visuals",
        action="store_true",
        help="Skip fixed-view RGB and capture-ID PNG comparisons.",
    )
    evaluation.add_argument(
        "--visual-max-points",
        type=int,
        default=150000,
        help="Maximum plotted points per variant in fixed views.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    nonnegative = {
        "--local-voxel-size-m": args.local_voxel_size_m,
        "--global-voxel-size-m": args.global_voxel_size_m,
        "--closure-translation-tolerance-m": (
            args.closure_translation_tolerance_m
        ),
        "--closure-rotation-tolerance-deg": (
            args.closure_rotation_tolerance_deg
        ),
        "--graph-min-aabb-overlap": args.graph_min_aabb_overlap,
        "--graph-max-correction-translation-m": (
            args.graph_max_correction_translation_m
        ),
        "--graph-max-correction-rotation-deg": (
            args.graph_max_correction_rotation_deg
        ),
        "--robust-kernel-k-fraction": args.robust_kernel_k_fraction,
        "--robust-kernel-min-m": args.robust_kernel_min_m,
        "--graph-edge-information-scale": (
            args.graph_edge_information_scale
        ),
        "--evaluation-voxel-size-m": args.evaluation_voxel_size_m,
        "--evaluation-max-distance-m": args.evaluation_max_distance_m,
    }
    for name, value in nonnegative.items():
        if value is not None and value < 0.0:
            raise ValueError(f"{name} must be non-negative")
    if args.shuffle_count < 0:
        raise ValueError("--shuffle-count must be non-negative")
    if args.closure_min_frame_gap < 1:
        raise ValueError("--closure-min-frame-gap must be at least 1")
    if args.graph_extra_neighbors < 0:
        raise ValueError("--graph-extra-neighbors must be non-negative")
    if not 0.0 <= args.graph_min_aabb_overlap <= 1.0:
        raise ValueError("--graph-min-aabb-overlap must be in [0, 1]")
    if not 0.0 <= args.graph_holdout_fraction < 1.0:
        raise ValueError("--graph-holdout-fraction must be in [0, 1)")
    if not 0.0 <= args.graph_edge_prune_threshold <= 1.0:
        raise ValueError("--graph-edge-prune-threshold must be in [0, 1]")
    if args.graph_loop_closure_preference < 0.0:
        raise ValueError(
            "--graph-loop-closure-preference must be non-negative"
        )
    if (
        args.robust_kernel_k_fraction == 0.0
        and args.robust_kernel_min_m == 0.0
    ):
        raise ValueError(
            "At least one robust-kernel k control must be positive"
        )
    if not 0.0 < args.evaluation_trim_fraction <= 1.0:
        raise ValueError("--evaluation-trim-fraction must be in (0, 1]")
    if args.evaluation_max_distance_m <= 0.0:
        raise ValueError("--evaluation-max-distance-m must be positive")
    if args.visual_max_points < 1:
        raise ValueError("--visual-max-points must be positive")
    for option_name, values in (
        ("--profiles", args.profiles),
        ("--graph-priors", args.graph_priors),
        ("--graph-scales", args.graph_scales),
    ):
        if len(values) != len(set(values)):
            raise ValueError(f"{option_name} cannot contain duplicates")
    scale_positions = [
        baseline.SCALE_NAMES.index(name) for name in args.graph_scales
    ]
    if scale_positions != sorted(scale_positions):
        raise ValueError(
            "--graph-scales must be ordered coarse-to-fine "
            "(SN3 SN2 SN1 SN0)"
        )


def json_ready_args(args: argparse.Namespace) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in vars(args).items():
        if isinstance(value, Path):
            result[key] = str(value)
        else:
            result[key] = value
    return result


def transform_delta(
    reference: np.ndarray, estimate: np.ndarray
) -> Tuple[np.ndarray, float, Tuple[float, float, float], float]:
    """Return the left correction that maps ``reference`` to ``estimate``."""

    delta = estimate @ np.linalg.inv(reference)
    translation = delta[:3, 3].copy()
    translation_norm = float(np.linalg.norm(translation))
    rpy_deg = baseline.rotation_matrix_to_rpy_deg(delta[:3, :3])
    rotation_deg = baseline.rotation_angle_deg(delta[:3, :3])
    return translation, translation_norm, rpy_deg, rotation_deg


def prepare_captures(
    captures: Sequence[Dict[str, Any]],
    profile_name: str,
    run_config: Dict[str, Any],
) -> List[Dict[str, Any]]:
    """Load identical filtered points in both base and camera coordinates.

    Filtering is performed on the already TF-placed saved cloud, exactly as in
    ``compare_filters.py``.  The selected points are then transformed back to
    the camera frame.  This avoids changing voxel membership between variants.
    """

    profile = baseline.profile_parameters(
        profile_name, run_config["parameters"]
    )
    prepared: List[Dict[str, Any]] = []
    for frame_index, metadata in enumerate(captures, start=1):
        tf_pose = baseline.selected_tf_transform(metadata)
        if tf_pose is None:
            raise ValueError(
                f"Capture {metadata.get('request_id')} has no saved "
                "tf_target_from_camera; TF-only/pose-graph comparison is "
                "not possible."
            )
        target_cloud = o3d.io.read_point_cloud(metadata["_cloud_path"])
        raw_points = len(target_cloud.points)
        target_cloud = baseline.crop_saved_cloud(target_cloud, metadata)
        cropped_points = len(target_cloud.points)
        target_cloud = baseline.filter_cloud(target_cloud, profile)
        if len(target_cloud.points) == 0:
            raise ValueError(
                f"Capture {metadata.get('request_id')} is empty after the "
                f"{profile_name} filter"
            )
        camera_cloud = copy.deepcopy(target_cloud)
        camera_cloud.transform(np.linalg.inv(tf_pose))
        prepared.append(
            {
                "frame_index": frame_index,
                "request_id": int(metadata["request_id"]),
                "metadata": metadata,
                "tf_pose": tf_pose,
                "cloud_target": target_cloud,
                "cloud_camera": camera_cloud,
                "raw_points": raw_points,
                "cropped_points": cropped_points,
                "filtered_points": len(target_cloud.points),
                "aabb_min": np.asarray(
                    target_cloud.get_min_bound(), dtype=np.float64
                ),
                "aabb_max": np.asarray(
                    target_cloud.get_max_bound(), dtype=np.float64
                ),
            }
        )
    return prepared


def pseudo_capture_results(
    prepared: Sequence[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    return [
        {
            "frame_index": item["frame_index"],
            "request_id": item["request_id"],
            "accepted": True,
            "tf_target_from_camera": item["tf_pose"].tolist(),
        }
        for item in prepared
    ]


def resolve_closure_pairs(
    prepared: Sequence[Dict[str, Any]],
    manual_value: Optional[str],
    translation_tolerance_m: float,
    rotation_tolerance_deg: float,
    minimum_frame_gap: int,
) -> List[Tuple[int, int]]:
    manual = baseline.parse_closure_pairs(manual_value)
    if manual is not None:
        known = {item["request_id"] for item in prepared}
        for first, second in manual:
            if first not in known or second not in known:
                raise ValueError(
                    f"Closure pair {first}:{second} is absent from selected "
                    "captures"
                )
        return manual
    return baseline.detect_closure_pairs(
        pseudo_capture_results(prepared),
        translation_tolerance_m,
        rotation_tolerance_deg,
        minimum_frame_gap,
    )


def aabb_overlap_ratio(first: Dict[str, Any], second: Dict[str, Any]) -> float:
    intersection_extent = np.maximum(
        0.0,
        np.minimum(first["aabb_max"], second["aabb_max"])
        - np.maximum(first["aabb_min"], second["aabb_min"]),
    )
    intersection_volume = float(np.prod(intersection_extent))
    first_volume = float(
        np.prod(np.maximum(first["aabb_max"] - first["aabb_min"], 0.0))
    )
    second_volume = float(
        np.prod(np.maximum(second["aabb_max"] - second["aabb_min"], 0.0))
    )
    denominator = min(first_volume, second_volume)
    return intersection_volume / denominator if denominator > 0.0 else 0.0


def select_pair_candidates(
    prepared: Sequence[Dict[str, Any]],
    closure_pairs: Sequence[Tuple[int, int]],
    minimum_overlap: float,
    maximum_extra_neighbors: int,
    holdout_fraction: float,
    random_seed: int,
) -> List[PairCandidate]:
    """Choose adjacent, known-closure, and strongest TF-overlap pairs."""

    request_to_index = {
        item["request_id"]: index for index, item in enumerate(prepared)
    }
    selected: Dict[Tuple[int, int], PairCandidate] = {}

    def add_pair(
        first_index: int,
        second_index: int,
        kind: str,
        overlap_score: Optional[float] = None,
    ) -> None:
        source_index, target_index = sorted((first_index, second_index))
        key = (source_index, target_index)
        score = (
            aabb_overlap_ratio(
                prepared[source_index], prepared[target_index]
            )
            if overlap_score is None
            else overlap_score
        )
        priority = {"overlap": 0, "adjacent": 1, "closure": 2}
        existing = selected.get(key)
        if existing is None or priority[kind] > priority[existing.kind]:
            selected[key] = PairCandidate(
                source_index, target_index, kind, score
            )

    for index in range(len(prepared) - 1):
        add_pair(index, index + 1, "adjacent")

    for first_request, second_request in closure_pairs:
        add_pair(
            request_to_index[first_request],
            request_to_index[second_request],
            "closure",
        )

    overlap_pool: List[Tuple[float, int, int]] = []
    for first_index in range(len(prepared)):
        for second_index in range(first_index + 1, len(prepared)):
            if (first_index, second_index) in selected:
                continue
            score = aabb_overlap_ratio(
                prepared[first_index], prepared[second_index]
            )
            if score >= minimum_overlap:
                overlap_pool.append((score, first_index, second_index))

    extra_degree = [0 for _ in prepared]
    for score, first_index, second_index in sorted(
        overlap_pool, reverse=True
    ):
        if (
            extra_degree[first_index] >= maximum_extra_neighbors
            or extra_degree[second_index] >= maximum_extra_neighbors
        ):
            continue
        add_pair(first_index, second_index, "overlap", score)
        extra_degree[first_index] += 1
        extra_degree[second_index] += 1

    overlap_keys = [
        key for key, item in selected.items() if item.kind == "overlap"
    ]
    holdout_count = (
        min(
            len(overlap_keys),
            max(1, int(round(len(overlap_keys) * holdout_fraction))),
        )
        if overlap_keys and holdout_fraction > 0.0
        else 0
    )
    if holdout_count:
        generator = np.random.default_rng(random_seed)
        chosen = {
            overlap_keys[int(index)]
            for index in generator.choice(
                len(overlap_keys), size=holdout_count, replace=False
            )
        }
        for key in chosen:
            item = selected[key]
            selected[key] = PairCandidate(
                item.source_index,
                item.target_index,
                item.kind,
                item.overlap_score,
                held_out=True,
            )

    return [
        selected[key]
        for key in sorted(
            selected,
            key=lambda pair: (
                selected[pair].kind != "adjacent",
                pair[0],
                pair[1],
            ),
        )
    ]


def robust_multiscale_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    initial_transform: np.ndarray,
    scales: Dict[str, Dict[str, float]],
    scale_names: Sequence[str],
    kernel_fraction: float,
    kernel_minimum_m: float,
) -> Tuple[
    bool,
    np.ndarray,
    Optional[float],
    Optional[float],
    str,
    float,
]:
    """Run relative-TF-initialized point-to-plane ICP with Tukey losses."""

    transform = initial_transform.copy()
    final_fitness: Optional[float] = None
    final_rmse: Optional[float] = None
    final_correspondence = 0.0

    for name in scale_names:
        parameters = scales[name]
        voxel = float(parameters["voxel"])
        correspondence = float(parameters["correspondence"])
        final_correspondence = correspondence
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
                final_correspondence,
            )

        normal_search = o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius, max_nn=40
        )
        source_scale.estimate_normals(search_param=normal_search)
        target_scale.estimate_normals(search_param=normal_search)
        tukey_k = max(
            kernel_minimum_m, kernel_fraction * correspondence
        )
        loss = o3d.pipelines.registration.TukeyLoss(k=tukey_k)
        estimation = (
            o3d.pipelines.registration.TransformationEstimationPointToPlane(
                loss
            )
        )
        registration = o3d.pipelines.registration.registration_icp(
            source=source_scale,
            target=target_scale,
            max_correspondence_distance=correspondence,
            init=transform,
            estimation_method=estimation,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_rmse=float(parameters["relative_rmse"]),
                relative_fitness=float(parameters["relative_fitness"]),
                max_iteration=int(parameters["iterations"]),
            ),
        )
        transform = np.asarray(
            registration.transformation, dtype=np.float64
        )
        final_fitness = float(registration.fitness)
        final_rmse = float(registration.inlier_rmse)
        if (
            final_fitness < float(parameters["fitness_threshold"])
            or final_rmse > float(parameters["rmse_threshold"])
        ):
            return (
                False,
                transform,
                final_fitness,
                final_rmse,
                f"{name}: robust ICP threshold failure",
                final_correspondence,
            )

    return (
        True,
        transform,
        final_fitness,
        final_rmse,
        "robust ICP accepted",
        final_correspondence,
    )


def estimate_pair_edges(
    prepared: Sequence[Dict[str, Any]],
    candidates: Sequence[PairCandidate],
    run_config: Dict[str, Any],
    args: argparse.Namespace,
) -> Tuple[List[Dict[str, Any]], float]:
    scales = baseline.normalized_scales(run_config)
    results: List[Dict[str, Any]] = []
    start = time.perf_counter()

    for number, candidate in enumerate(candidates, start=1):
        source = prepared[candidate.source_index]
        target = prepared[candidate.target_index]
        initial = np.linalg.inv(target["tf_pose"]) @ source["tf_pose"]
        print(
            "  [edge "
            f"{number}/{len(candidates)}] "
            f"{source['request_id']}->{target['request_id']} "
            f"({candidate.kind}"
            f"{', held out' if candidate.held_out else ''})"
        )
        (
            accepted,
            transform,
            fitness,
            rmse,
            reason,
            information_distance,
        ) = robust_multiscale_icp(
            source["cloud_camera"],
            target["cloud_camera"],
            initial,
            scales,
            args.graph_scales,
            args.robust_kernel_k_fraction,
            args.robust_kernel_min_m,
        )
        correction = transform @ np.linalg.inv(initial)
        correction_translation_m = float(
            np.linalg.norm(correction[:3, 3])
        )
        correction_rotation_deg = baseline.rotation_angle_deg(
            correction[:3, :3]
        )
        if accepted and (
            correction_translation_m
            > args.graph_max_correction_translation_m
            or correction_rotation_deg
            > args.graph_max_correction_rotation_deg
        ):
            accepted = False
            reason = (
                "relative-TF correction limit failure "
                f"({1000.0 * correction_translation_m:.1f} mm, "
                f"{correction_rotation_deg:.2f} deg)"
            )

        information: Optional[np.ndarray] = None
        if accepted:
            information = (
                o3d.pipelines.registration
                .get_information_matrix_from_point_clouds(
                    source["cloud_camera"],
                    target["cloud_camera"],
                    information_distance,
                    transform,
                )
            )
            information = (
                np.asarray(information, dtype=np.float64)
                * args.graph_edge_information_scale
            )
            information += np.eye(6, dtype=np.float64) * 1.0e-9

        print(
            "    "
            f"{'accepted' if accepted else 'rejected'}: {reason}; "
            f"fitness={fitness if fitness is not None else float('nan'):.3f}, "
            "rmse="
            f"{1000.0 * rmse if rmse is not None else float('nan'):.2f} mm, "
            f"TF delta={1000.0 * correction_translation_m:.1f} mm/"
            f"{correction_rotation_deg:.2f} deg"
        )
        results.append(
            {
                "source_index": candidate.source_index,
                "target_index": candidate.target_index,
                "source_request_id": source["request_id"],
                "target_request_id": target["request_id"],
                "kind": candidate.kind,
                "uncertain": candidate.kind != "adjacent",
                "held_out": candidate.held_out,
                "overlap_score": candidate.overlap_score,
                "accepted": accepted,
                "reason": reason,
                "fitness": fitness,
                "inlier_rmse_m": rmse,
                "initial_relative_tf": initial.tolist(),
                "measured_relative_transform": transform.tolist(),
                "correction_from_relative_tf": correction.tolist(),
                "correction_translation_m": correction_translation_m,
                "correction_rotation_deg": correction_rotation_deg,
                "information_matrix": (
                    information.tolist() if information is not None else None
                ),
                "information_distance_m": information_distance,
            }
        )
    return results, time.perf_counter() - start


def tf_prior_information(
    translation_sigma_m: float, rotation_sigma_deg: float
) -> np.ndarray:
    if translation_sigma_m <= 0.0 or rotation_sigma_deg <= 0.0:
        raise ValueError("TF prior sigmas must be positive")
    rotation_sigma_rad = math.radians(rotation_sigma_deg)
    # Open3D's 6D twist ordering is rotation XYZ, then translation XYZ.
    diagonal = [
        1.0 / (rotation_sigma_rad * rotation_sigma_rad),
        1.0 / (rotation_sigma_rad * rotation_sigma_rad),
        1.0 / (rotation_sigma_rad * rotation_sigma_rad),
        1.0 / (translation_sigma_m * translation_sigma_m),
        1.0 / (translation_sigma_m * translation_sigma_m),
        1.0 / (translation_sigma_m * translation_sigma_m),
    ]
    return np.diag(diagonal).astype(np.float64)


def build_pose_graph(
    prepared: Sequence[Dict[str, Any]],
    edge_records: Sequence[Dict[str, Any]],
    translation_sigma_m: float,
    rotation_sigma_deg: float,
) -> o3d.pipelines.registration.PoseGraph:
    pose_graph = o3d.pipelines.registration.PoseGraph()
    pose_graph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(
            np.eye(4, dtype=np.float64)
        )
    )
    for item in prepared:
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(item["tf_pose"])
        )

    prior_information = tf_prior_information(
        translation_sigma_m, rotation_sigma_deg
    )
    for capture_index, item in enumerate(prepared):
        # Source=fixed base, target=camera. Open3D edges map source -> target.
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                0,
                capture_index + 1,
                np.linalg.inv(item["tf_pose"]),
                prior_information,
                False,
                1.0,
            )
        )

    for record in edge_records:
        if not record["accepted"] or record["held_out"]:
            continue
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                int(record["source_index"]) + 1,
                int(record["target_index"]) + 1,
                baseline.validate_transform(
                    record["measured_relative_transform"],
                    "graph edge measurement",
                ),
                np.asarray(
                    record["information_matrix"], dtype=np.float64
                ),
                bool(record["uncertain"]),
                1.0,
            )
        )
    return pose_graph


def color_for_capture(index: int, count: int) -> Tuple[float, float, float]:
    hue = (index / max(count, 1) + 0.07) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.78, 0.95)


def fuse_clouds_once(
    prepared: Sequence[Dict[str, Any]],
    poses: Sequence[np.ndarray],
    global_voxel_m: float,
    output_path: Optional[Path],
    capture_id_output_path: Optional[Path] = None,
    accepted_request_ids: Optional[Iterable[int]] = None,
) -> Tuple[int, Optional[int]]:
    if len(prepared) != len(poses):
        raise ValueError(
            f"Pose count {len(poses)} does not match capture count "
            f"{len(prepared)}"
        )
    accepted_set = (
        None
        if accepted_request_ids is None
        else {int(value) for value in accepted_request_ids}
    )
    fused = o3d.geometry.PointCloud()
    capture_id_fused = o3d.geometry.PointCloud()
    for index, (item, pose) in enumerate(zip(prepared, poses)):
        if (
            accepted_set is not None
            and item["request_id"] not in accepted_set
        ):
            continue
        transformed = copy.deepcopy(item["cloud_camera"])
        transformed.transform(pose)
        fused += transformed
        if capture_id_output_path is not None:
            diagnostic = copy.deepcopy(transformed)
            diagnostic.paint_uniform_color(
                color_for_capture(index, len(prepared))
            )
            capture_id_fused += diagnostic

    if global_voxel_m > 0.0:
        fused = fused.voxel_down_sample(global_voxel_m)
        if capture_id_output_path is not None:
            capture_id_fused = capture_id_fused.voxel_down_sample(
                global_voxel_m
            )

    if len(fused.points) == 0:
        raise RuntimeError("No points remained for final fusion")
    if output_path is not None:
        if not o3d.io.write_point_cloud(
            str(output_path), fused, write_ascii=False, compressed=False
        ):
            raise OSError(f"Failed to write {output_path}")

    capture_id_count: Optional[int] = None
    if capture_id_output_path is not None:
        if not o3d.io.write_point_cloud(
            str(capture_id_output_path),
            capture_id_fused,
            write_ascii=False,
            compressed=False,
        ):
            raise OSError(f"Failed to write {capture_id_output_path}")
        capture_id_count = len(capture_id_fused.points)
    return len(fused.points), capture_id_count


def capture_results_from_poses(
    prepared: Sequence[Dict[str, Any]],
    poses: Sequence[np.ndarray],
) -> List[Dict[str, Any]]:
    captures: List[Dict[str, Any]] = []
    for item, pose in zip(prepared, poses):
        (
            correction_translation,
            correction_translation_norm,
            correction_rpy_deg,
            correction_rotation_deg,
        ) = transform_delta(item["tf_pose"], pose)
        correction = pose @ np.linalg.inv(item["tf_pose"])
        captures.append(
            {
                "frame_index": item["frame_index"],
                "request_id": item["request_id"],
                "accepted": True,
                "reason": "optimized and fused once",
                "raw_points": item["raw_points"],
                "cropped_points": item["cropped_points"],
                "filtered_points": item["filtered_points"],
                "fitness": None,
                "inlier_rmse_m": None,
                "icp_correction": correction.tolist(),
                "icp_correction_translation_m": (
                    correction_translation.tolist()
                ),
                "icp_correction_translation_norm_m": (
                    correction_translation_norm
                ),
                "icp_correction_rpy_deg": list(correction_rpy_deg),
                "icp_correction_rotation_deg": correction_rotation_deg,
                "tf_target_from_camera": item["tf_pose"].tolist(),
                "final_target_from_camera": pose.tolist(),
            }
        )
    return captures


def correction_summary(
    captures: Sequence[Dict[str, Any]]
) -> Dict[str, Optional[float]]:
    accepted = [item for item in captures if item.get("accepted")]
    if not accepted:
        return {
            "mean_pose_deviation_translation_mm": None,
            "p95_pose_deviation_translation_mm": None,
            "max_pose_deviation_translation_mm": None,
            "mean_pose_deviation_rotation_deg": None,
            "p95_pose_deviation_rotation_deg": None,
            "max_pose_deviation_rotation_deg": None,
            "translation_deviation_slope_mm_per_frame": None,
            "translation_deviation_frame_correlation": None,
        }
    pose_deviations = []
    for item in accepted:
        if (
            item.get("tf_target_from_camera") is not None
            and item.get("final_target_from_camera") is not None
        ):
            pose_deviations.append(
                baseline.relative_pose_error(
                    baseline.validate_transform(
                        item["tf_target_from_camera"],
                        f"capture {item['request_id']} TF",
                    ),
                    baseline.validate_transform(
                        item["final_target_from_camera"],
                        f"capture {item['request_id']} final pose",
                    ),
                )
            )
        else:
            pose_deviations.append(
                (
                    float(item["icp_correction_translation_norm_m"]),
                    float(item["icp_correction_rotation_deg"]),
                )
            )
    translations_mm = 1000.0 * np.asarray(
        [value[0] for value in pose_deviations], dtype=np.float64
    )
    rotations_deg = np.asarray(
        [value[1] for value in pose_deviations], dtype=np.float64
    )
    frames = np.asarray(
        [item["frame_index"] for item in accepted], dtype=np.float64
    )
    if len(frames) >= 2:
        slope = float(np.polyfit(frames, translations_mm, 1)[0])
        if (
            float(np.std(frames)) > 0.0
            and float(np.std(translations_mm)) > 0.0
        ):
            correlation = float(
                np.corrcoef(frames, translations_mm)[0, 1]
            )
        else:
            correlation = 0.0
    else:
        slope = 0.0
        correlation = 0.0
    return {
        "mean_pose_deviation_translation_mm": float(
            np.mean(translations_mm)
        ),
        "p95_pose_deviation_translation_mm": float(
            np.percentile(translations_mm, 95.0)
        ),
        "max_pose_deviation_translation_mm": float(
            np.max(translations_mm)
        ),
        "mean_pose_deviation_rotation_deg": float(
            np.mean(rotations_deg)
        ),
        "p95_pose_deviation_rotation_deg": float(
            np.percentile(rotations_deg, 95.0)
        ),
        "max_pose_deviation_rotation_deg": float(
            np.max(rotations_deg)
        ),
        "translation_deviation_slope_mm_per_frame": slope,
        "translation_deviation_frame_correlation": correlation,
    }


def calculate_generic_closures(
    captures: Sequence[Dict[str, Any]],
    closure_pairs: Sequence[Tuple[int, int]],
) -> List[Dict[str, Any]]:
    by_request = {int(item["request_id"]): item for item in captures}
    rows: List[Dict[str, Any]] = []
    for row_index, (first_id, second_id) in enumerate(closure_pairs):
        first = by_request.get(first_id)
        second = by_request.get(second_id)
        if first is None or second is None:
            continue
        if (
            first.get("tf_target_from_camera") is None
            or second.get("tf_target_from_camera") is None
            or first.get("final_target_from_camera") is None
            or second.get("final_target_from_camera") is None
        ):
            continue
        tf_translation_m, tf_rotation_deg = baseline.relative_pose_error(
            baseline.validate_transform(
                first["tf_target_from_camera"], f"capture {first_id} TF"
            ),
            baseline.validate_transform(
                second["tf_target_from_camera"], f"capture {second_id} TF"
            ),
        )
        final_translation_m, final_rotation_deg = (
            baseline.relative_pose_error(
                baseline.validate_transform(
                    first["final_target_from_camera"],
                    f"capture {first_id} final pose",
                ),
                baseline.validate_transform(
                    second["final_target_from_camera"],
                    f"capture {second_id} final pose",
                ),
            )
        )
        rows.append(
            {
                "row_index": row_index,
                "start_request_id": first_id,
                "end_request_id": second_id,
                "frame_gap": abs(
                    int(second["frame_index"]) - int(first["frame_index"])
                ),
                "tf_translation_error_mm": 1000.0 * tf_translation_m,
                "tf_rotation_error_deg": tf_rotation_deg,
                "final_translation_error_mm": (
                    1000.0 * final_translation_m
                ),
                "final_rotation_error_deg": final_rotation_deg,
                "translation_error_change_mm": (
                    1000.0 * (final_translation_m - tf_translation_m)
                ),
                "rotation_error_change_deg": (
                    final_rotation_deg - tf_rotation_deg
                ),
            }
        )
    return rows


def write_dict_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(rows)


def edge_rows_for_csv(
    edge_records: Sequence[Dict[str, Any]],
    optimized_confidences: Optional[Dict[int, float]] = None,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for index, record in enumerate(edge_records):
        rows.append(
            {
                "edge_index": index,
                "source_request_id": record["source_request_id"],
                "target_request_id": record["target_request_id"],
                "kind": record["kind"],
                "uncertain": record["uncertain"],
                "held_out": record["held_out"],
                "overlap_score": record["overlap_score"],
                "accepted": record["accepted"],
                "reason": record["reason"],
                "fitness": record["fitness"],
                "inlier_rmse_mm": (
                    None
                    if record["inlier_rmse_m"] is None
                    else 1000.0 * record["inlier_rmse_m"]
                ),
                "relative_tf_correction_translation_mm": (
                    1000.0 * record["correction_translation_m"]
                ),
                "relative_tf_correction_rotation_deg": (
                    record["correction_rotation_deg"]
                ),
                "optimized_line_process_confidence": (
                    None
                    if optimized_confidences is None
                    else optimized_confidences.get(index)
                ),
            }
        )
    return rows


def run_pose_graph_variant(
    *,
    prior_name: str,
    prepared: Sequence[Dict[str, Any]],
    edge_records: Sequence[Dict[str, Any]],
    edge_estimation_time_sec: float,
    run_config: Dict[str, Any],
    closure_pairs: Sequence[Tuple[int, int]],
    output_dir: Path,
    args: argparse.Namespace,
) -> Dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=False)
    translation_sigma_m, rotation_sigma_deg = GRAPH_PRIORS[prior_name]
    pose_graph = build_pose_graph(
        prepared,
        edge_records,
        translation_sigma_m,
        rotation_sigma_deg,
    )
    start = time.perf_counter()
    final_correspondence = float(
        baseline.normalized_scales(run_config)[
            args.graph_scales[-1]
        ]["correspondence"]
    )
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=final_correspondence,
        edge_prune_threshold=args.graph_edge_prune_threshold,
        preference_loop_closure=args.graph_loop_closure_preference,
        reference_node=0,
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )
    optimization_time_sec = time.perf_counter() - start

    optimized_poses = [
        np.asarray(pose_graph.nodes[index + 1].pose, dtype=np.float64)
        for index in range(len(prepared))
    ]
    captures = capture_results_from_poses(prepared, optimized_poses)
    global_voxel_m = float(
        run_config["parameters"].get("global_voxel_size_m", 0.004)
    )
    cloud_path = output_dir / "global_point_cloud.ply"
    capture_id_cloud_path = output_dir / "global_capture_id_cloud.ply"
    fusion_start = time.perf_counter()
    final_point_count, capture_id_point_count = fuse_clouds_once(
        prepared,
        optimized_poses,
        global_voxel_m,
        cloud_path,
        capture_id_cloud_path,
    )
    fusion_time_sec = time.perf_counter() - fusion_start

    confidence_by_nodes = {
        (int(edge.source_node_id), int(edge.target_node_id)): float(
            edge.confidence
        )
        for edge in pose_graph.edges
    }
    optimized_confidences: Dict[int, float] = {}
    for record_index, record in enumerate(edge_records):
        if not record["accepted"] or record["held_out"]:
            continue
        key = (
            int(record["source_index"]) + 1,
            int(record["target_index"]) + 1,
        )
        optimized_confidences[record_index] = confidence_by_nodes.get(
            key,
            0.0 if record["uncertain"] else 1.0,
        )

    write_dict_csv(
        output_dir / "edges.csv",
        edge_rows_for_csv(edge_records, optimized_confidences),
    )
    baseline.write_frame_metrics_csv(
        output_dir / "frame_metrics.csv", captures
    )
    closures = calculate_generic_closures(captures, closure_pairs)
    write_dict_csv(output_dir / "closure_metrics.csv", closures)
    pose_graph_path = output_dir / "optimized_pose_graph.json"
    o3d.io.write_pose_graph(str(pose_graph_path), pose_graph)
    if not pose_graph_path.is_file() or pose_graph_path.stat().st_size == 0:
        raise OSError(f"Failed to write {pose_graph_path}")

    accepted_edges = [
        record
        for record in edge_records
        if record["accepted"] and not record["held_out"]
    ]
    uncertain_confidences = [
        optimized_confidences[index]
        for index, record in enumerate(edge_records)
        if (
            record["accepted"]
            and not record["held_out"]
            and record["uncertain"]
            and index in optimized_confidences
        )
    ]
    summary = correction_summary(captures)
    metrics: Dict[str, Any] = {
        "profile": run_config.get("_active_profile"),
        "registration_mode": f"tf_pose_graph_{prior_name}",
        "variant_label": f"graph_{prior_name}",
        "use_icp": True,
        "fusion_policy": "optimize all poses, then fuse once",
        "capture_count": len(prepared),
        "accepted_captures": len(prepared),
        "rejected_captures": 0,
        "accepted_percent": 100.0,
        "final_point_count": final_point_count,
        "capture_id_point_count": capture_id_point_count,
        "cloud_path": str(cloud_path),
        "capture_id_cloud_path": str(capture_id_cloud_path),
        "tf_prior_name": prior_name,
        "tf_prior_translation_sigma_m": translation_sigma_m,
        "tf_prior_rotation_sigma_deg": rotation_sigma_deg,
        "pair_edges_attempted": len(edge_records),
        "pair_edges_accepted_for_graph": len(accepted_edges),
        "pair_edges_rejected": sum(
            not record["accepted"] for record in edge_records
        ),
        "pair_edges_held_out": sum(
            record["accepted"] and record["held_out"]
            for record in edge_records
        ),
        "uncertain_edges_below_prune_threshold": sum(
            confidence < args.graph_edge_prune_threshold
            for confidence in uncertain_confidences
        ),
        "edge_estimation_time_sec": edge_estimation_time_sec,
        "optimization_time_sec": optimization_time_sec,
        "fusion_time_sec": fusion_time_sec,
        "processing_time_sec": (
            edge_estimation_time_sec
            + optimization_time_sec
            + fusion_time_sec
        ),
        "mean_pairwise_fitness": (
            float(
                np.mean(
                    [
                        record["fitness"]
                        for record in accepted_edges
                        if record["fitness"] is not None
                    ]
                )
            )
            if any(
                record["fitness"] is not None
                for record in accepted_edges
            )
            else None
        ),
        "mean_pairwise_rmse_mm": (
            float(
                1000.0
                * np.mean(
                    [
                        record["inlier_rmse_m"]
                        for record in accepted_edges
                        if record["inlier_rmse_m"] is not None
                    ]
                )
            )
            if any(
                record["inlier_rmse_m"] is not None
                for record in accepted_edges
            )
            else None
        ),
        "closures": closures,
        "captures": captures,
        **summary,
    }
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)
    return metrics


def poses_by_request(
    captures: Sequence[Dict[str, Any]],
) -> Dict[int, np.ndarray]:
    poses: Dict[int, np.ndarray] = {}
    for item in captures:
        if (
            item.get("accepted")
            and item.get("final_target_from_camera") is not None
        ):
            poses[int(item["request_id"])] = baseline.validate_transform(
                item["final_target_from_camera"],
                f"capture {item['request_id']} final pose",
            )
    return poses


def add_baseline_diagnostics(
    result: Dict[str, Any],
    prepared: Sequence[Dict[str, Any]],
    closure_pairs: Sequence[Tuple[int, int]],
    output_dir: Path,
    global_voxel_m: float,
) -> Dict[str, Any]:
    result["variant_label"] = result["registration_mode"]
    result["fusion_policy"] = (
        "chronological TF-only accumulation with global voxelization after "
        "each frame"
        if result["registration_mode"] == "tf_only"
        else (
            "chronological ICP against the accumulated map with global "
            "voxelization after each accepted frame"
        )
    )
    result.update(correction_summary(result["captures"]))
    closures = calculate_generic_closures(
        result["captures"], closure_pairs
    )
    result["closures"] = closures
    write_dict_csv(output_dir / "closure_metrics.csv", closures)

    pose_map = poses_by_request(result["captures"])
    poses: List[np.ndarray] = []
    accepted_ids: List[int] = []
    for item in prepared:
        pose = pose_map.get(item["request_id"])
        if pose is None:
            poses.append(item["tf_pose"])
        else:
            poses.append(pose)
            accepted_ids.append(item["request_id"])
    capture_id_path = output_dir / "global_capture_id_cloud.ply"
    _, capture_id_count = fuse_clouds_once(
        prepared,
        poses,
        global_voxel_m,
        None,
        capture_id_path,
        accepted_ids,
    )
    result["capture_id_cloud_path"] = str(capture_id_path)
    result["capture_id_point_count"] = capture_id_count
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)
    return result


def run_baselines(
    *,
    profile_name: str,
    selected_captures: Sequence[Dict[str, Any]],
    prepared: Sequence[Dict[str, Any]],
    run_config: Dict[str, Any],
    closure_pairs: Sequence[Tuple[int, int]],
    profile_output_dir: Path,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    baseline_root = profile_output_dir / "baselines"
    global_voxel_m = float(
        run_config["parameters"].get("global_voxel_size_m", 0.004)
    )
    results: Dict[str, Dict[str, Any]] = {}
    for mode in ("tf_only", "tf_icp"):
        output_dir = baseline_root / mode
        print(f"[{profile_name}/{mode}] rebuilding chronological baseline")
        result = baseline.fuse_profile(
            profile_name=profile_name,
            captures=selected_captures,
            run_config=run_config,
            registration_mode=mode,
            output_dir=output_dir,
        )
        baseline.write_frame_metrics_csv(
            output_dir / "frame_metrics.csv", result["captures"]
        )
        results[mode] = add_baseline_diagnostics(
            result,
            prepared,
            closure_pairs,
            output_dir,
            global_voxel_m,
        )
    return results["tf_only"], results["tf_icp"]


def order_variants(
    selected_captures: Sequence[Dict[str, Any]],
    shuffle_count: int,
    random_seed: int,
) -> List[Tuple[str, List[Dict[str, Any]]]]:
    chronological = list(selected_captures)
    variants: List[Tuple[str, List[Dict[str, Any]]]] = [
        ("chronological", chronological),
        ("reverse", list(reversed(chronological))),
    ]
    generator = np.random.default_rng(random_seed)
    for number in range(1, shuffle_count + 1):
        permutation = generator.permutation(len(chronological))
        variants.append(
            (
                f"shuffle_{number:02d}",
                [chronological[int(index)] for index in permutation],
            )
        )
    return variants


def compare_order_poses(
    reference: Dict[str, Any],
    candidate: Dict[str, Any],
    order_name: str,
) -> List[Dict[str, Any]]:
    reference_poses = poses_by_request(reference["captures"])
    candidate_poses = poses_by_request(candidate["captures"])
    rows: List[Dict[str, Any]] = []
    for request_id in sorted(set(reference_poses) & set(candidate_poses)):
        translation_m, rotation_deg = baseline.relative_pose_error(
            reference_poses[request_id], candidate_poses[request_id]
        )
        rows.append(
            {
                "order": order_name,
                "request_id": request_id,
                "translation_difference_mm": 1000.0 * translation_m,
                "rotation_difference_deg": rotation_deg,
            }
        )
    return rows


def run_order_test(
    *,
    profile_name: str,
    selected_captures: Sequence[Dict[str, Any]],
    prepared: Sequence[Dict[str, Any]],
    run_config: Dict[str, Any],
    closure_pairs: Sequence[Tuple[int, int]],
    profile_output_dir: Path,
    chronological_result: Dict[str, Any],
    shuffle_count: int,
    random_seed: int,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    global_voxel_m = float(
        run_config["parameters"].get("global_voxel_size_m", 0.004)
    )
    variants = order_variants(
        selected_captures, shuffle_count, random_seed
    )
    results: List[Dict[str, Any]] = []
    per_pose_rows: List[Dict[str, Any]] = []
    for order_name, ordered_captures in variants:
        if order_name == "chronological":
            result = chronological_result
        else:
            output_dir = profile_output_dir / "order_test" / order_name
            print(
                f"[{profile_name}/order:{order_name}] replaying "
                f"{len(ordered_captures)} captures"
            )
            result = baseline.fuse_profile(
                profile_name=profile_name,
                captures=ordered_captures,
                run_config=run_config,
                registration_mode="tf_icp",
                output_dir=output_dir,
            )
            baseline.write_frame_metrics_csv(
                output_dir / "frame_metrics.csv", result["captures"]
            )
            result = add_baseline_diagnostics(
                result,
                prepared,
                closure_pairs,
                output_dir,
                global_voxel_m,
            )

        rows = (
            []
            if order_name == "chronological"
            else compare_order_poses(
                chronological_result, result, order_name
            )
        )
        per_pose_rows.extend(rows)
        translations = np.asarray(
            [row["translation_difference_mm"] for row in rows],
            dtype=np.float64,
        )
        rotations = np.asarray(
            [row["rotation_difference_deg"] for row in rows],
            dtype=np.float64,
        )
        visual_result = dict(result)
        visual_result["variant_label"] = order_name
        results.append(
            {
                "order": order_name,
                "request_id_order": [
                    int(item["request_id"]) for item in ordered_captures
                ],
                "accepted_captures": result["accepted_captures"],
                "rejected_captures": result["rejected_captures"],
                "final_point_count": result["final_point_count"],
                "processing_time_sec": result["processing_time_sec"],
                "mean_pose_difference_from_chronological_mm": (
                    float(np.mean(translations))
                    if translations.size
                    else 0.0
                ),
                "p95_pose_difference_from_chronological_mm": (
                    float(np.percentile(translations, 95.0))
                    if translations.size
                    else 0.0
                ),
                "max_pose_difference_from_chronological_mm": (
                    float(np.max(translations))
                    if translations.size
                    else 0.0
                ),
                "mean_rotation_difference_from_chronological_deg": (
                    float(np.mean(rotations)) if rotations.size else 0.0
                ),
                "max_rotation_difference_from_chronological_deg": (
                    float(np.max(rotations)) if rotations.size else 0.0
                ),
                "cloud_path": result["cloud_path"],
                "capture_id_cloud_path": result.get(
                    "capture_id_cloud_path"
                ),
                "_full_result": visual_result,
            }
        )

    output_dir = profile_output_dir / "order_test"
    output_dir.mkdir(parents=True, exist_ok=True)
    write_dict_csv(
        output_dir / "order_summary.csv",
        [
            {key: value for key, value in row.items() if key != "_full_result"}
            for row in results
        ],
    )
    write_dict_csv(
        output_dir / "order_pose_differences.csv", per_pose_rows
    )
    with open(
        output_dir / "order_summary.json", "w", encoding="utf-8"
    ) as handle:
        json.dump(
            [
                {
                    key: value
                    for key, value in row.items()
                    if key != "_full_result"
                }
                for row in results
            ],
            handle,
            indent=2,
        )
    return results, per_pose_rows


def estimate_normals_for_evaluation(
    cloud: o3d.geometry.PointCloud, voxel_m: float
) -> o3d.geometry.PointCloud:
    evaluated = (
        cloud.voxel_down_sample(voxel_m)
        if voxel_m > 0.0
        else copy.deepcopy(cloud)
    )
    radius = max(0.025, voxel_m * 3.0)
    evaluated.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius, max_nn=40
        )
    )
    evaluated.normalize_normals()
    return evaluated


def directional_point_to_plane_residuals(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    maximum_distance_m: float,
) -> np.ndarray:
    source_points = np.asarray(source.points, dtype=np.float64)
    target_points = np.asarray(target.points, dtype=np.float64)
    target_normals = np.asarray(target.normals, dtype=np.float64)
    if (
        source_points.size == 0
        or target_points.size == 0
        or target_normals.shape != target_points.shape
    ):
        return np.empty(0, dtype=np.float64)
    tree = o3d.geometry.KDTreeFlann(target)
    residuals: List[float] = []
    maximum_squared = maximum_distance_m * maximum_distance_m
    for point in source_points:
        count, indices, squared_distances = tree.search_knn_vector_3d(
            point, 1
        )
        if count < 1 or float(squared_distances[0]) > maximum_squared:
            continue
        index = int(indices[0])
        residuals.append(
            abs(
                float(
                    np.dot(
                        point - target_points[index],
                        target_normals[index],
                    )
                )
            )
        )
    return np.asarray(residuals, dtype=np.float64)


def trimmed_symmetric_point_to_plane(
    source_camera: o3d.geometry.PointCloud,
    target_camera: o3d.geometry.PointCloud,
    target_from_source: np.ndarray,
    voxel_m: float,
    maximum_distance_m: float,
    trim_fraction: float,
) -> Dict[str, Optional[float]]:
    source = estimate_normals_for_evaluation(source_camera, voxel_m)
    target = estimate_normals_for_evaluation(target_camera, voxel_m)
    source.transform(target_from_source)
    forward = directional_point_to_plane_residuals(
        source, target, maximum_distance_m
    )
    reverse = directional_point_to_plane_residuals(
        target, source, maximum_distance_m
    )
    residuals = np.concatenate((forward, reverse))
    if residuals.size == 0:
        return {
            "symmetric_inlier_count": 0,
            "trimmed_mean_point_to_plane_mm": None,
            "trimmed_rmse_point_to_plane_mm": None,
            "p90_point_to_plane_mm": None,
        }
    keep_count = max(1, int(math.floor(residuals.size * trim_fraction)))
    retained = np.partition(residuals, keep_count - 1)[:keep_count]
    return {
        "symmetric_inlier_count": int(residuals.size),
        "trimmed_mean_point_to_plane_mm": (
            1000.0 * float(np.mean(retained))
        ),
        "trimmed_rmse_point_to_plane_mm": (
            1000.0 * float(np.sqrt(np.mean(retained * retained)))
        ),
        "p90_point_to_plane_mm": (
            1000.0 * float(np.percentile(residuals, 90.0))
        ),
    }


def evaluate_held_out_edges(
    variants: Sequence[Dict[str, Any]],
    prepared: Sequence[Dict[str, Any]],
    edge_records: Sequence[Dict[str, Any]],
    args: argparse.Namespace,
) -> List[Dict[str, Any]]:
    held_out = [
        record
        for record in edge_records
        if record["accepted"] and record["held_out"]
    ]
    rows: List[Dict[str, Any]] = []
    for variant in variants:
        pose_map = poses_by_request(variant["captures"])
        for record in held_out:
            source_id = int(record["source_request_id"])
            target_id = int(record["target_request_id"])
            if source_id not in pose_map or target_id not in pose_map:
                continue
            predicted = (
                np.linalg.inv(pose_map[target_id]) @ pose_map[source_id]
            )
            measured = baseline.validate_transform(
                record["measured_relative_transform"],
                "held-out edge measurement",
            )
            pose_translation_m, pose_rotation_deg = (
                baseline.relative_pose_error(measured, predicted)
            )
            geometry = trimmed_symmetric_point_to_plane(
                prepared[int(record["source_index"])]["cloud_camera"],
                prepared[int(record["target_index"])]["cloud_camera"],
                predicted,
                args.evaluation_voxel_size_m,
                args.evaluation_max_distance_m,
                args.evaluation_trim_fraction,
            )
            rows.append(
                {
                    "variant": variant["variant_label"],
                    "source_request_id": source_id,
                    "target_request_id": target_id,
                    "kind": record["kind"],
                    "overlap_score": record["overlap_score"],
                    "pose_residual_translation_mm": (
                        1000.0 * pose_translation_m
                    ),
                    "pose_residual_rotation_deg": pose_rotation_deg,
                    **geometry,
                }
            )
    return rows


def heldout_summary(
    heldout_rows: Sequence[Dict[str, Any]], variant_label: str
) -> Dict[str, Optional[float]]:
    rows = [
        row for row in heldout_rows if row["variant"] == variant_label
    ]
    rmse_values = [
        row["trimmed_rmse_point_to_plane_mm"]
        for row in rows
        if row["trimmed_rmse_point_to_plane_mm"] is not None
    ]
    return {
        "heldout_edge_count": len(rows),
        "mean_heldout_pose_residual_translation_mm": (
            float(
                np.mean(
                    [
                        row["pose_residual_translation_mm"]
                        for row in rows
                    ]
                )
            )
            if rows
            else None
        ),
        "mean_heldout_pose_residual_rotation_deg": (
            float(
                np.mean(
                    [
                        row["pose_residual_rotation_deg"]
                        for row in rows
                    ]
                )
            )
            if rows
            else None
        ),
        "mean_heldout_trimmed_point_to_plane_rmse_mm": (
            float(np.mean(rmse_values)) if rmse_values else None
        ),
    }


def closure_summary(
    closures: Sequence[Dict[str, Any]]
) -> Dict[str, Optional[float]]:
    if not closures:
        return {
            "mean_closure_translation_mm": None,
            "mean_closure_rotation_deg": None,
            "max_closure_translation_mm": None,
            "max_closure_rotation_deg": None,
        }
    translations = [
        row["final_translation_error_mm"] for row in closures
    ]
    rotations = [row["final_rotation_error_deg"] for row in closures]
    return {
        "mean_closure_translation_mm": float(np.mean(translations)),
        "mean_closure_rotation_deg": float(np.mean(rotations)),
        "max_closure_translation_mm": float(np.max(translations)),
        "max_closure_rotation_deg": float(np.max(rotations)),
    }


def comparison_rows(
    variants: Sequence[Dict[str, Any]],
    heldout_rows: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    keys = [
        "profile",
        "variant_label",
        "registration_mode",
        "fusion_policy",
        "capture_count",
        "accepted_captures",
        "rejected_captures",
        "final_point_count",
        "mean_icp_fitness",
        "mean_icp_rmse_mm",
        "mean_pairwise_fitness",
        "mean_pairwise_rmse_mm",
        "processing_time_sec",
        "mean_pose_deviation_translation_mm",
        "p95_pose_deviation_translation_mm",
        "max_pose_deviation_translation_mm",
        "mean_pose_deviation_rotation_deg",
        "p95_pose_deviation_rotation_deg",
        "max_pose_deviation_rotation_deg",
        "translation_deviation_slope_mm_per_frame",
        "translation_deviation_frame_correlation",
        "tf_prior_name",
        "tf_prior_translation_sigma_m",
        "tf_prior_rotation_sigma_deg",
        "pair_edges_attempted",
        "pair_edges_accepted_for_graph",
        "pair_edges_rejected",
        "pair_edges_held_out",
        "uncertain_edges_below_prune_threshold",
        "edge_estimation_time_sec",
        "optimization_time_sec",
        "fusion_time_sec",
        "cloud_path",
        "capture_id_cloud_path",
    ]
    rows: List[Dict[str, Any]] = []
    for variant in variants:
        row = {key: variant.get(key) for key in keys}
        row.update(closure_summary(variant.get("closures", [])))
        row.update(
            heldout_summary(heldout_rows, variant["variant_label"])
        )
        rows.append(row)
    return rows


def sampled_cloud_arrays(
    path: str, maximum_points: int, seed: int
) -> Tuple[np.ndarray, np.ndarray]:
    cloud = o3d.io.read_point_cloud(path)
    points = np.asarray(cloud.points, dtype=np.float64)
    colors = np.asarray(cloud.colors, dtype=np.float64)
    if colors.shape != points.shape:
        colors = np.full(points.shape, 0.18, dtype=np.float64)
    if len(points) > maximum_points:
        generator = np.random.default_rng(seed)
        indices = generator.choice(
            len(points), size=maximum_points, replace=False
        )
        points = points[indices]
        colors = colors[indices]
    return points, np.clip(colors, 0.0, 1.0)


def fixed_view_limits(
    arrays: Sequence[Tuple[np.ndarray, np.ndarray]]
) -> Tuple[np.ndarray, np.ndarray]:
    nonempty = [points for points, _ in arrays if len(points)]
    if not nonempty:
        raise ValueError("No points available for visual comparison")
    combined = np.concatenate(nonempty, axis=0)
    minimum = np.percentile(combined, 0.25, axis=0)
    maximum = np.percentile(combined, 99.75, axis=0)
    extent = np.maximum(maximum - minimum, 1.0e-3)
    padding = extent * 0.04
    return minimum - padding, maximum + padding


def save_fixed_view_grid(
    variants: Sequence[Dict[str, Any]],
    path_key: str,
    output_path: Path,
    maximum_points: int,
    title: str,
) -> None:
    plt = baseline.plt
    if plt is None:
        print(
            "WARNING: matplotlib is unavailable; fixed-view PNGs skipped."
        )
        return
    available = [
        variant for variant in variants if variant.get(path_key)
    ]
    if not available:
        return
    arrays = [
        sampled_cloud_arrays(
            str(variant[path_key]), maximum_points, seed=101 + index
        )
        for index, variant in enumerate(available)
    ]
    minimum, maximum = fixed_view_limits(arrays)
    views = [
        ("Front: X-Z", 0, 2),
        ("Side: Y-Z", 1, 2),
        ("Top: X-Y", 0, 1),
    ]
    figure, axes = plt.subplots(
        len(views),
        len(available),
        figsize=(4.0 * len(available), 11.0),
        squeeze=False,
    )
    for column, (variant, (points, colors)) in enumerate(
        zip(available, arrays)
    ):
        for row, (view_name, horizontal, vertical) in enumerate(views):
            axis = axes[row, column]
            axis.scatter(
                points[:, horizontal],
                points[:, vertical],
                c=colors,
                s=0.20,
                linewidths=0,
                rasterized=True,
            )
            axis.set_xlim(minimum[horizontal], maximum[horizontal])
            axis.set_ylim(minimum[vertical], maximum[vertical])
            axis.set_aspect("equal", adjustable="box")
            axis.grid(alpha=0.12, linewidth=0.4)
            if row == 0:
                axis.set_title(variant["variant_label"])
            if column == 0:
                axis.set_ylabel(view_name)
            axis.set_xticks([])
            axis.set_yticks([])
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def save_visual_comparisons(
    variants: Sequence[Dict[str, Any]],
    output_dir: Path,
    maximum_points: int,
    prefix: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    save_fixed_view_grid(
        variants,
        "cloud_path",
        output_dir / f"{prefix}_rgb_fixed_views.png",
        maximum_points,
        f"{prefix}: identical base-frame views (RGB)",
    )
    save_fixed_view_grid(
        variants,
        "capture_id_cloud_path",
        output_dir / f"{prefix}_capture_id_fixed_views.png",
        maximum_points,
        f"{prefix}: identical views colored by capture",
    )


def write_profile_manifest(
    path: Path,
    profile_name: str,
    prepared: Sequence[Dict[str, Any]],
    closure_pairs: Sequence[Tuple[int, int]],
    candidates: Sequence[PairCandidate],
    args: argparse.Namespace,
) -> None:
    manifest = {
        "profile": profile_name,
        "request_ids": [item["request_id"] for item in prepared],
        "closure_pairs": [list(pair) for pair in closure_pairs],
        "pair_candidates": [
            {
                "source_request_id": prepared[item.source_index][
                    "request_id"
                ],
                "target_request_id": prepared[item.target_index][
                    "request_id"
                ],
                "kind": item.kind,
                "overlap_score": item.overlap_score,
                "held_out": item.held_out,
            }
            for item in candidates
        ],
        "graph_priors": {
            name: {
                "translation_sigma_m": GRAPH_PRIORS[name][0],
                "rotation_sigma_deg": GRAPH_PRIORS[name][1],
            }
            for name in args.graph_priors
        },
        "graph_scales": list(args.graph_scales),
        "identical_capture_policy": (
            "Every variant uses the same selected attempt per request ID and "
            "the same filtered point selection. Only poses/fusion order vary."
        ),
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)


def main() -> None:
    args = parse_args()
    validate_args(args)
    scan_dir = args.scan_dir.expanduser().resolve()
    run_config_path = scan_dir / "run_config.json"
    if not run_config_path.is_file():
        raise FileNotFoundError(f"Missing {run_config_path}")

    selected_captures = baseline.discover_captures(scan_dir)
    if not selected_captures:
        raise RuntimeError(
            f"No raw/capture_*/raw_target_cloud.ply inputs found in "
            f"{scan_dir}"
        )
    run_config = baseline.load_json(run_config_path)
    run_config["parameters"] = run_config["parameters"].copy()
    if args.local_voxel_size_m is not None:
        run_config["parameters"]["local_voxel_size_m"] = (
            args.local_voxel_size_m
        )
    if args.global_voxel_size_m is not None:
        run_config["parameters"]["global_voxel_size_m"] = (
            args.global_voxel_size_m
        )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_root = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else scan_dir / "registration_comparisons" / timestamp
    )
    output_root.mkdir(parents=True, exist_ok=False)
    root_manifest = {
        "source_scan_dir": str(scan_dir),
        "run_config_path": str(run_config_path),
        "capture_metadata_paths": [
            item["_metadata_path"] for item in selected_captures
        ],
        "request_ids": [
            int(item["request_id"]) for item in selected_captures
        ],
        "arguments": json_ready_args(args),
        "effective_local_voxel_size_m": float(
            run_config["parameters"].get("local_voxel_size_m", 0.004)
        ),
        "effective_global_voxel_size_m": float(
            run_config["parameters"].get("global_voxel_size_m", 0.004)
        ),
        "open3d_version": getattr(o3d, "__version__", "unknown"),
        "transform_convention": {
            "node_pose": "T_base_camera",
            "pair_edge": "T_camera_target_camera_source",
            "tf_prior_edge": (
                "source=fixed base node, target=camera node, "
                "measurement=inverse(T_base_camera)"
            ),
        },
    }
    with open(
        output_root / "experiment_manifest.json", "w", encoding="utf-8"
    ) as handle:
        json.dump(root_manifest, handle, indent=2)

    for profile_name in args.profiles:
        print(f"\n=== Filter profile: {profile_name} ===")
        profile_output_dir = output_root / profile_name
        profile_output_dir.mkdir(parents=True, exist_ok=False)
        run_config["_active_profile"] = profile_name
        prepared = prepare_captures(
            selected_captures, profile_name, run_config
        )
        closure_pairs = resolve_closure_pairs(
            prepared,
            args.closure_pairs,
            args.closure_translation_tolerance_m,
            args.closure_rotation_tolerance_deg,
            args.closure_min_frame_gap,
        )
        if not closure_pairs:
            print(
                "WARNING: no repeated TF closure pairs detected; closure "
                "metrics will be empty. Supply --closure-pairs if known."
            )
        candidates = select_pair_candidates(
            prepared,
            closure_pairs,
            args.graph_min_aabb_overlap,
            args.graph_extra_neighbors,
            args.graph_holdout_fraction,
            args.random_seed,
        )
        write_profile_manifest(
            profile_output_dir / "profile_manifest.json",
            profile_name,
            prepared,
            closure_pairs,
            candidates,
            args,
        )

        tf_only, tf_icp = run_baselines(
            profile_name=profile_name,
            selected_captures=selected_captures,
            prepared=prepared,
            run_config=run_config,
            closure_pairs=closure_pairs,
            profile_output_dir=profile_output_dir,
        )
        variants: List[Dict[str, Any]] = [tf_only, tf_icp]

        order_results: List[Dict[str, Any]] = []
        if not args.skip_order_test:
            order_results, _ = run_order_test(
                profile_name=profile_name,
                selected_captures=selected_captures,
                prepared=prepared,
                run_config=run_config,
                closure_pairs=closure_pairs,
                profile_output_dir=profile_output_dir,
                chronological_result=tf_icp,
                shuffle_count=args.shuffle_count,
                random_seed=args.random_seed,
            )

        print(
            f"[{profile_name}/pose_graph] estimating "
            f"{len(candidates)} pairwise edges"
        )
        edge_records, edge_estimation_time_sec = estimate_pair_edges(
            prepared, candidates, run_config, args
        )
        write_dict_csv(
            profile_output_dir / "pairwise_edges.csv",
            edge_rows_for_csv(edge_records),
        )
        with open(
            profile_output_dir / "pairwise_edges.json",
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(edge_records, handle, indent=2)

        for prior_name in args.graph_priors:
            print(
                f"[{profile_name}/graph_{prior_name}] optimizing and "
                "fusing once"
            )
            result = run_pose_graph_variant(
                prior_name=prior_name,
                prepared=prepared,
                edge_records=edge_records,
                edge_estimation_time_sec=edge_estimation_time_sec,
                run_config=run_config,
                closure_pairs=closure_pairs,
                output_dir=(
                    profile_output_dir
                    / "pose_graph"
                    / f"tf_prior_{prior_name}"
                ),
                args=args,
            )
            variants.append(result)

        heldout_rows = evaluate_held_out_edges(
            variants, prepared, edge_records, args
        )
        write_dict_csv(
            profile_output_dir / "heldout_edge_comparison.csv",
            heldout_rows,
        )
        rows = comparison_rows(variants, heldout_rows)
        write_dict_csv(
            profile_output_dir / "registration_comparison.csv", rows
        )
        with open(
            profile_output_dir / "registration_comparison.json",
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(rows, handle, indent=2)

        if not args.skip_visuals:
            save_visual_comparisons(
                variants,
                profile_output_dir / "visuals",
                args.visual_max_points,
                "registration",
            )
            if order_results:
                save_visual_comparisons(
                    [item["_full_result"] for item in order_results],
                    profile_output_dir / "visuals",
                    args.visual_max_points,
                    "incremental_order",
                )

        print(
            f"[{profile_name}] comparison table: "
            f"{profile_output_dir / 'registration_comparison.csv'}"
        )

    print(f"\nRegistration experiment complete: {output_root}")


if __name__ == "__main__":
    main()