#!/usr/bin/env python3
"""Capture-triggered RGB-D tree reconstruction for ROS 2 Humble.

The node consumes the UR5e end-effector Jetson camera's aligned depth stream,
places each RGB-D frame in a common TF frame, optionally refines the TF estimate
with coarse-to-fine ICP, fuses accepted frames, publishes a PointCloud2, and
writes a continuously updated PLY file.
"""

from __future__ import annotations

import copy
import json
import math
import os
import time
import traceback
import warnings
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import open3d as o3d
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import Bool, Int32, UInt32
from tf2_ros import Buffer, TransformException, TransformListener

from reconstruct.pc_utils import o3dpc_to_pointcloud2


class TreePointCloudReconstructor(Node):
    """Fuse synchronized RGB-D captures into a colored tree point cloud."""

    SCALE_NAMES = ("SN3", "SN2", "SN1", "SN0")

    def __init__(self) -> None:
        super().__init__("tree_point_cloud_reconstructor")
        self._declare_parameters()
        self._validate_configuration()

        self.bridge = CvBridge()
        self.target_frame = self._p("target_frame")
        self.camera_frame_override = self._p("camera_frame_override")
        self.output_root = Path(os.path.expanduser(self._p("output_root"))).resolve()
        self.scan_id = self._p("scan_id")
        self.scan_dir = self.output_root / self.scan_id
        self.raw_dir = self.scan_dir / "raw"
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.global_cloud_path = self.scan_dir / "global_point_cloud.ply"
        self.summary_path = self.scan_dir / "summary.json"
        self.run_config_path = self.scan_dir / "run_config.json"

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera drivers and Gazebo bridges commonly use best-effort sensor QoS.
        color_topic = self._p("color_topic")
        depth_topic = self._p("depth_topic")
        camera_info_topic = self._p("camera_info_topic")
        self.color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, camera_info_topic, qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=int(self._p("sync_queue_size")),
            slop=float(self._p("sync_slop_sec")),
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.rgbd_callback)

        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.capture_sub = self.create_subscription(
            UInt32,
            self._p("capture_request_topic"),
            self.capture_request_callback,
            control_qos,
        )
        self.finished_sub = self.create_subscription(
            Bool, self._p("scan_finished_topic"), self.scan_finished_callback, control_qos
        )
        self.capture_result_pub = self.create_publisher(
            Int32, self._p("capture_result_topic"), control_qos
        )
        self.pcd_pub = self.create_publisher(
            PointCloud2, self._p("output_cloud_topic"), cloud_qos
        )
        self.original_pcd_pub = self.create_publisher(
            PointCloud2, self._p("original_cloud_topic"), cloud_qos
        )
        self.latest_joint_state: Optional[JointState] = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            self._p("joint_state_topic"),
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        self.pending_capture_requests: Deque[Tuple[int, int]] = deque()
        if bool(self._p("capture_on_start")):
            self.pending_capture_requests.append((1, self.get_clock().now().nanoseconds))
        self.active_request_id: Optional[int] = None
        self.active_request_received_ns = 0
        self.active_burst: List[Dict[str, Any]] = []
        self.last_burst_stamp_ns = -1
        self.active_attempt_number = 0
        self.active_retry_not_before_ns = 0
        self.capture_attempts: Dict[int, int] = {}
        self.processing = False
        self.global_pc = o3d.geometry.PointCloud()
        self.target_crop_center: Optional[np.ndarray] = None

        self.total_frames = 0
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.completed_capture_requests = 0
        self.accepted_capture_requests = 0
        self.rejected_capture_requests = 0
        self.icp_fitnesses: List[float] = []
        self.icp_rmses: List[float] = []
        self.reconstruction_times_ms: List[float] = []
        self.summary_written = False
        self.registration_scales = self._load_registration_scales()
        self._write_run_config()

        self.get_logger().info(
            "Tree reconstructor ready. Waiting for capture triggers on "
            f"'{self._p('capture_request_topic')}'."
        )
        self.get_logger().info(
            f"RGB-D: {color_topic} + {depth_topic} + {camera_info_topic}"
        )
        self.get_logger().info(
            f"PLY output will be written to: {self.global_cloud_path}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "color_topic": "/sensors/camera_jetson/color/image_raw",
            "depth_topic": "/sensors/camera_jetson/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/sensors/camera_jetson/aligned_depth_to_color/camera_info",
            "capture_request_topic": "/tree_scan/capture_request",
            "capture_result_topic": "/tree_scan/capture_result",
            "scan_finished_topic": "/tree_scan/scan_finished",
            "output_cloud_topic": "/tree_scan/global_cloud",
            "original_cloud_topic": "/tree_scan/latest_tf_cloud",
            "joint_state_topic": "/joint_states",
            "target_frame": "base_link",
            "camera_frame_override": "",
            "output_root": "~/tree_scans",
            "scan_id": "indoor_model_tree",
            "capture_on_start": False,
            "sync_queue_size": 30,
            "sync_slop_sec": 0.08,
            "tf_timeout_sec": 0.75,
            "post_request_guard_sec": 0.0,
            "sensor_clock_warning_sec": 0.25,
            "depth_burst_size": 5,
            "depth_burst_min_valid_samples": 2,
            "capture_max_attempts": 3,
            "capture_retry_delay_sec": 0.25,
            "burst_pose_translation_tolerance_m": 0.005,
            "burst_pose_rotation_tolerance_deg": 0.5,
            "min_depth_m": 0.30,
            "max_depth_m": 2.50,
            "depth_scale_override": 0.0,
            "target_crop_enabled": False,
            "target_center_mode": "camera_forward",
            "target_distance_m": 1.0,
            "target_center_target_frame_m": [0.0, 0.0, 0.0],
            "target_box_size_m": [1.0, 1.0, 1.5],
            "local_voxel_size_m": 0.006,
            "global_voxel_size_m": 0.005,
            "outlier_removal_enabled": True,
            "outlier_filter_mode": "radius",
            "outlier_radius_m": 0.020,
            "outlier_min_neighbors": 6,
            "statistical_nb_neighbors": 20,
            "statistical_std_ratio": 2.0,
            "use_icp": True,
            "icp_crop_margin_m": 0.15,
            "icp_min_target_points": 200,
            "max_icp_correction_translation_m": 0.10,
            "max_icp_correction_rotation_deg": 12.0,
            "save_raw_point_clouds": True,
            "publish_original_cloud": True,
            "shutdown_on_scan_finished": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        scale_defaults = {
            "SN3": (0.05, 0.05, 50, 0.025, 0.35),
            "SN2": (0.02, 0.03, 30, 0.015, 0.40),
            "SN1": (0.01, 0.018, 20, 0.010, 0.45),
            "SN0": (0.00, 0.012, 15, 0.008, 0.45),
        }
        for scale, values in scale_defaults.items():
            voxel, correspondence, iterations, rmse, fitness = values
            prefix = f"registration.{scale}"
            self.declare_parameter(f"{prefix}.voxel_size_m", voxel)
            self.declare_parameter(
                f"{prefix}.max_correspondence_distance_m", correspondence
            )
            self.declare_parameter(f"{prefix}.max_iterations", iterations)
            self.declare_parameter(f"{prefix}.relative_rmse", 1.0e-6)
            self.declare_parameter(f"{prefix}.relative_fitness", 1.0e-6)
            self.declare_parameter(f"{prefix}.rmse_threshold_m", rmse)
            self.declare_parameter(f"{prefix}.fitness_threshold", fitness)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _validate_configuration(self) -> None:
        burst_size = int(self.get_parameter("depth_burst_size").value)
        min_valid = int(
            self.get_parameter("depth_burst_min_valid_samples").value
        )
        if burst_size < 1:
            raise ValueError("depth_burst_size must be at least 1")
        if min_valid < 1 or min_valid > burst_size:
            raise ValueError(
                "depth_burst_min_valid_samples must be between 1 and "
                "depth_burst_size"
            )
        max_attempts = int(
            self.get_parameter("capture_max_attempts").value
        )
        if max_attempts < 1:
            raise ValueError("capture_max_attempts must be at least 1")
        retry_delay = float(
            self.get_parameter("capture_retry_delay_sec").value
        )
        if retry_delay < 0.0:
            raise ValueError("capture_retry_delay_sec must be nonnegative")
        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        if min_depth < 0.0 or max_depth <= min_depth:
            raise ValueError(
                "Depth limits must satisfy 0 <= min_depth_m < max_depth_m"
            )

    def _load_registration_scales(self) -> Dict[str, Dict[str, float]]:
        scales: Dict[str, Dict[str, float]] = {}
        for name in self.SCALE_NAMES:
            prefix = f"registration.{name}"
            scales[name] = {
                "voxel": float(self._p(f"{prefix}.voxel_size_m")),
                "correspondence": float(
                    self._p(f"{prefix}.max_correspondence_distance_m")
                ),
                "iterations": int(self._p(f"{prefix}.max_iterations")),
                "relative_rmse": float(self._p(f"{prefix}.relative_rmse")),
                "relative_fitness": float(
                    self._p(f"{prefix}.relative_fitness")
                ),
                "rmse_threshold": float(self._p(f"{prefix}.rmse_threshold_m")),
                "fitness_threshold": float(
                    self._p(f"{prefix}.fitness_threshold")
                ),
            }
        return scales

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def capture_request_callback(self, msg: UInt32) -> None:
        request_id = int(msg.data)
        if request_id <= 0 or request_id > np.iinfo(np.int32).max:
            self.get_logger().error(
                f"Ignoring invalid capture request id {request_id}; "
                "valid ids are 1..2147483647."
            )
            return

        queued_ids = {item[0] for item in self.pending_capture_requests}
        if request_id == self.active_request_id or request_id in queued_ids:
            self.get_logger().warning(
                f"Ignoring duplicate capture request {request_id}."
            )
            return

        received_ns = self.get_clock().now().nanoseconds
        self.pending_capture_requests.append((request_id, received_ns))
        self.get_logger().info(
            f"Capture {request_id} queued; "
            f"pending={len(self.pending_capture_requests)}."
        )

    def scan_finished_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._write_global_cloud()
        self._write_summary()
        self.get_logger().info(
            f"Scan complete. Final PLY: {self.global_cloud_path}"
        )
        if bool(self._p("shutdown_on_scan_finished")):
            self.create_timer(0.25, rclpy.shutdown)

    def rgbd_callback(
        self, color_msg: Image, depth_msg: Image, camera_info: CameraInfo
    ) -> None:
        if self.processing:
            return

        if self.active_request_id is None:
            if not self.pending_capture_requests:
                return
            (
                self.active_request_id,
                self.active_request_received_ns,
            ) = self.pending_capture_requests.popleft()
            self.active_burst = []
            self.last_burst_stamp_ns = -1
            self.active_attempt_number = 1
            self.active_retry_not_before_ns = 0
            self.get_logger().info(
                f"Capture {self.active_request_id} attempt "
                f"{self.active_attempt_number}/"
                f"{int(self._p('capture_max_attempts'))}: waiting for "
                f"{int(self._p('depth_burst_size'))} new synchronized RGB-D "
                "frames."
            )

        if (
            self.active_retry_not_before_ns > 0
            and self.get_clock().now().nanoseconds
            < self.active_retry_not_before_ns
        ):
            return

        stamp_ns = (
            int(depth_msg.header.stamp.sec) * 1_000_000_000
            + int(depth_msg.header.stamp.nanosec)
        )
        minimum_stamp_ns = self.active_request_received_ns + int(
            float(self._p("post_request_guard_sec")) * 1.0e9
        )
        if stamp_ns <= minimum_stamp_ns:
            # The synchronizer may still contain frames captured before the request.
            return
        if stamp_ns <= self.last_burst_stamp_ns:
            return

        try:
            camera_frame = (
                self.camera_frame_override
                or depth_msg.header.frame_id
                or camera_info.header.frame_id
                or color_msg.header.frame_id
            )
            if not camera_frame:
                self.get_logger().error(
                    "The synchronized camera messages have no TF frame_id."
                )
                return

            tf_matrix = self._lookup_transform_matrix(
                self.target_frame, camera_frame, depth_msg.header.stamp
            )
            if tf_matrix is None:
                # Keep the request active and try a later sensor frame.
                return

            color_rgb, depth_array, depth_scale, intrinsic = self._decode_images(
                color_msg, depth_msg, camera_info
            )

            if self.active_burst:
                first = self.active_burst[0]
                if (
                    color_rgb.shape != first["color_rgb"].shape
                    or depth_array.shape != first["depth_array"].shape
                    or not math.isclose(
                        float(depth_scale),
                        float(first["depth_scale"]),
                        rel_tol=0.0,
                        abs_tol=1.0e-12,
                    )
                ):
                    self._complete_or_retry_active_capture(
                        accepted=False,
                        reason="camera stream profile changed inside depth burst",
                    )
                    return

            now_ns = self.get_clock().now().nanoseconds
            sensor_age_sec = (now_ns - stamp_ns) / 1.0e9
            if not self.active_burst and abs(sensor_age_sec) > float(
                self._p("sensor_clock_warning_sec")
            ):
                self.get_logger().warning(
                    f"Capture {self.active_request_id}: sensor stamp differs from "
                    f"this computer by {sensor_age_sec:+.3f} s (includes transport "
                    "latency). Verify the lab chrony/NTP source on both hosts."
                )

            self.active_burst.append(
                {
                    "color_rgb": color_rgb,
                    "depth_array": depth_array,
                    "depth_scale": float(depth_scale),
                    "intrinsic": intrinsic,
                    "camera_info": self._camera_info_to_dict(camera_info),
                    "tf_matrix": tf_matrix,
                    "camera_frame": camera_frame,
                    "stamp_ns": stamp_ns,
                    "stamp_sec": int(depth_msg.header.stamp.sec),
                    "stamp_nanosec": int(depth_msg.header.stamp.nanosec),
                    "stamp_msg": depth_msg.header.stamp,
                    "color_encoding": color_msg.encoding,
                    "depth_encoding": depth_msg.encoding,
                    "color_frame_id": color_msg.header.frame_id,
                    "depth_frame_id": depth_msg.header.frame_id,
                    "joint_state": self._joint_state_to_dict(
                        self.latest_joint_state
                    ),
                }
            )
            self.last_burst_stamp_ns = stamp_ns

            burst_size = int(self._p("depth_burst_size"))
            if len(self.active_burst) < burst_size:
                return

            self.processing = True
            accepted, reason = self._process_active_burst()
            self._complete_or_retry_active_capture(
                accepted=accepted,
                reason=reason,
            )
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
        except Exception as exc:
            self.get_logger().error(
                f"RGB-D reconstruction failed: {exc}\n{traceback.format_exc()}"
            )
            if self.active_request_id is not None and len(self.active_burst) >= int(
                self._p("depth_burst_size")
            ):
                self._complete_or_retry_active_capture(
                    accepted=False,
                    reason=f"processing exception: {exc}",
                )

    def _complete_or_retry_active_capture(
        self, *, accepted: bool, reason: str
    ) -> None:
        request_id = self.active_request_id
        if request_id is None:
            self.processing = False
            return

        if accepted:
            self._finish_active_capture(accepted=True, reason=reason)
            return

        max_attempts = int(self._p("capture_max_attempts"))
        if self.active_attempt_number < max_attempts:
            failed_attempt = self.active_attempt_number
            self.active_attempt_number += 1
            retry_delay_sec = float(self._p("capture_retry_delay_sec"))
            now_ns = self.get_clock().now().nanoseconds
            self.active_request_received_ns = now_ns
            self.active_retry_not_before_ns = now_ns + int(
                retry_delay_sec * 1.0e9
            )
            self.active_burst = []
            self.last_burst_stamp_ns = -1
            self.processing = False
            self.get_logger().warning(
                f"Capture {request_id} attempt {failed_attempt}/{max_attempts} "
                f"rejected: {reason}. The cloud was not merged. Retrying at "
                f"the same pose with a fresh {int(self._p('depth_burst_size'))}-"
                f"frame burst (attempt {self.active_attempt_number}/"
                f"{max_attempts}) after {retry_delay_sec:.2f} s."
            )
            return

        self._finish_active_capture(
            accepted=False,
            reason=(
                f"all {max_attempts} attempts rejected; "
                f"last attempt: {reason}"
            ),
        )

    def _finish_active_capture(self, *, accepted: bool, reason: str) -> None:
        request_id = self.active_request_id
        if request_id is None:
            self.processing = False
            return

        attempts_used = self.active_attempt_number
        result = Int32()
        result.data = request_id if accepted else -request_id
        self.capture_result_pub.publish(result)
        status = "accepted" if accepted else "rejected"
        self.completed_capture_requests += 1
        if accepted:
            self.accepted_capture_requests += 1
        else:
            self.rejected_capture_requests += 1
        self.get_logger().info(
            f"Capture {request_id} {status} after {attempts_used} attempt(s); "
            f"result={result.data}; reason={reason}."
        )
        self.active_request_id = None
        self.active_request_received_ns = 0
        self.active_burst = []
        self.last_burst_stamp_ns = -1
        self.active_attempt_number = 0
        self.active_retry_not_before_ns = 0
        self.processing = False

    def _process_active_burst(self) -> Tuple[bool, str]:
        if self.active_request_id is None or not self.active_burst:
            return False, "no active depth burst"

        request_id = self.active_request_id
        start_wall = time.perf_counter()
        median_depth_m, valid_counts = self._zero_aware_depth_median(
            self.active_burst
        )
        selected_index = len(self.active_burst) // 2
        selected = self.active_burst[selected_index]

        capture_dir, metadata_path = self._save_raw_capture(
            request_id=request_id,
            samples=self.active_burst,
            median_depth_m=median_depth_m,
            valid_counts=valid_counts,
            selected_index=selected_index,
        )

        translation_spread_m, rotation_spread_deg = self._burst_pose_spread(
            self.active_burst
        )
        if (
            translation_spread_m
            > float(self._p("burst_pose_translation_tolerance_m"))
            or rotation_spread_deg
            > float(self._p("burst_pose_rotation_tolerance_deg"))
        ):
            reason = (
                "camera moved during burst "
                f"({translation_spread_m * 1000.0:.1f} mm, "
                f"{rotation_spread_deg:.2f} deg)"
            )
            self.total_frames += 1
            self.rejected_frames += 1
            self._update_capture_metadata(
                metadata_path,
                {
                    "accepted": False,
                    "reason": reason,
                    "burst_pose_translation_spread_m": translation_spread_m,
                    "burst_pose_rotation_spread_deg": rotation_spread_deg,
                },
            )
            return False, reason

        pcd_camera = self._rgbd_to_cloud(
            selected["color_rgb"],
            median_depth_m,
            1.0,
            selected["intrinsic"],
        )
        if len(pcd_camera.points) == 0:
            reason = "zero-aware median produced no in-range points"
            self.total_frames += 1
            self.rejected_frames += 1
            self._update_capture_metadata(
                metadata_path, {"accepted": False, "reason": reason}
            )
            return False, reason

        pcd_target = copy.deepcopy(pcd_camera)
        pcd_target.transform(selected["tf_matrix"])

        if bool(self._p("save_raw_point_clouds")):
            o3d.io.write_point_cloud(
                str(capture_dir / "raw_camera_cloud.ply"),
                pcd_camera,
                write_ascii=False,
                compressed=False,
            )
            o3d.io.write_point_cloud(
                str(capture_dir / "raw_target_cloud.ply"),
                pcd_target,
                write_ascii=False,
                compressed=False,
            )

        cropped_target, crop_bounds = self._crop_to_target(
            pcd_target, selected["tf_matrix"]
        )
        if bool(self._p("publish_original_cloud")) and len(cropped_target.points) > 0:
            original_msg = o3dpc_to_pointcloud2(
                cropped_target,
                self.target_frame,
                selected["stamp_msg"],
            )
            self.original_pcd_pub.publish(original_msg)

        accepted, icp_transform, fitness, rmse, filtered_cloud = self._fuse_frame(
            cropped_target
        )

        self.total_frames += 1
        if accepted:
            self.accepted_frames += 1
            if fitness is not None:
                self.icp_fitnesses.append(float(fitness))
            if rmse is not None:
                self.icp_rmses.append(float(rmse))
        else:
            self.rejected_frames += 1

        self._publish_global_cloud(selected["stamp_msg"])
        self._write_global_cloud()

        elapsed_ms = (time.perf_counter() - start_wall) * 1000.0
        self.reconstruction_times_ms.append(elapsed_ms)
        if accepted and fitness is None:
            metric_text = "first frame / TF only"
        elif not accepted and len(filtered_cloud.points) == 0:
            metric_text = "empty cloud after crop/filter"
        elif fitness is None:
            metric_text = "rejected before ICP"
        else:
            metric_text = (
                f"fitness={fitness:.3f}, rmse={rmse * 1000.0:.1f} mm"
            )
        status = "accepted" if accepted else "rejected"
        self.get_logger().info(
            f"Capture {request_id} attempt {self.active_attempt_number}/"
            f"{int(self._p('capture_max_attempts'))} {status}: {metric_text}; "
            f"burst valid median pixels="
            f"{100.0 * np.count_nonzero(median_depth_m) / median_depth_m.size:.1f}%; "
            f"global points={len(self.global_pc.points)}; {elapsed_ms:.1f} ms"
        )

        reason = metric_text
        self._update_capture_metadata(
            metadata_path,
            {
                "accepted": bool(accepted),
                "reason": reason,
                "fitness": None if fitness is None else float(fitness),
                "inlier_rmse_m": None if rmse is None else float(rmse),
                "icp_correction_in_target_frame": icp_transform.tolist(),
                "burst_pose_translation_spread_m": translation_spread_m,
                "burst_pose_rotation_spread_deg": rotation_spread_deg,
                "raw_camera_point_count": len(pcd_camera.points),
                "raw_target_point_count": len(pcd_target.points),
                "cropped_target_point_count": len(cropped_target.points),
                "filtered_point_count": len(filtered_cloud.points),
                "target_crop_bounds_m": crop_bounds,
                "processing_time_ms": elapsed_ms,
                "global_point_count_after_capture": len(self.global_pc.points),
            },
        )
        return accepted, reason

    def _lookup_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp,
    ) -> Optional[np.ndarray]:
        # Camera-only mode: the cloud is already expressed in the desired frame.
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=float(self._p("tf_timeout_sec"))),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"TF unavailable: {target_frame} <- {source_frame} "
                f"at sensor time: {exc}"
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation

        matrix = self._quaternion_xyzw_to_matrix(
            q.x,
            q.y,
            q.z,
            q.w,
        )
        matrix[:3, 3] = [t.x, t.y, t.z]

        return matrix

    @staticmethod
    def _quaternion_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-12:
            raise ValueError("TF quaternion has zero norm")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), 0.0],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), 0.0],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _decode_images(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info: CameraInfo,
    ) -> Tuple[
        np.ndarray,
        np.ndarray,
        float,
        o3d.camera.PinholeCameraIntrinsic,
    ]:
        color_rgb = self._color_to_rgb(color_msg)

        depth = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough",
        )
        depth = np.asarray(depth)

        if depth.ndim != 2:
            raise ValueError(
                f"Expected one-channel depth image, got shape {depth.shape}"
            )

        # Determine depth units separately from the dtype conversion.
        override = float(self._p("depth_scale_override"))

        if override > 0.0:
            depth_scale = override
        elif depth_msg.encoding in ("16UC1", "mono16"):
            depth_scale = 1000.0
        elif depth_msg.encoding == "32FC1":
            depth_scale = 1.0
        elif np.issubdtype(depth.dtype, np.unsignedinteger):
            depth_scale = 1000.0
        elif np.issubdtype(depth.dtype, np.floating):
            depth_scale = 1.0
        else:
            raise ValueError(
                f"Unsupported depth encoding '{depth_msg.encoding}' / "
                f"dtype {depth.dtype}. Set depth_scale_override if the "
                "depth units are known."
            )

        # Normalize to an Open3D-supported, native-endian dtype.
        if depth_msg.encoding in ("16UC1", "mono16"):
            depth = depth.astype(np.uint16, copy=False)
        elif depth_msg.encoding == "32FC1":
            depth = depth.astype(np.float32, copy=False)
        elif np.issubdtype(depth.dtype, np.floating):
            depth = depth.astype(np.float32, copy=False)
        elif np.issubdtype(depth.dtype, np.integer):
            if np.any(depth < 0):
                raise ValueError(
                    f"Depth image has negative integer values with dtype {depth.dtype}"
                )
            depth = depth.astype(np.uint16, copy=False)
        else:
            raise ValueError(
                f"Cannot convert depth dtype {depth.dtype} for Open3D"
            )

        # Ensure native byte order. Some ROS image buffers may be big-endian.
        if not depth.dtype.isnative:
            depth = depth.byteswap().view(depth.dtype.newbyteorder("="))

        depth = np.ascontiguousarray(depth)

        if color_rgb.shape[:2] != depth.shape[:2]:
            raise ValueError(
                "Aligned depth and color dimensions differ: "
                f"color={color_rgb.shape[:2]}, depth={depth.shape[:2]}"
            )

        height, width = depth.shape
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("CameraInfo contains invalid focal lengths")

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width,
            height,
            fx,
            fy,
            cx,
            cy,
        )

        return (
            np.ascontiguousarray(color_rgb, dtype=np.uint8),
            depth,
            depth_scale,
            intrinsic,
        )

    def _color_to_rgb(self, msg: Image) -> np.ndarray:
        image = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            rgb = image
        elif encoding == "bgr8":
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif encoding == "rgba8":
            rgb = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        elif encoding == "bgra8":
            rgb = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        elif encoding in ("mono8", "8uc1"):
            rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        else:
            # cv_bridge handles many Bayer/YUV variants when a target encoding is supplied.
            rgb = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        return rgb

    @staticmethod
    def _camera_info_to_dict(msg: CameraInfo) -> Dict[str, Any]:
        return {
            "width": int(msg.width),
            "height": int(msg.height),
            "distortion_model": msg.distortion_model,
            "d": [float(value) for value in msg.d],
            "k": [float(value) for value in msg.k],
            "r": [float(value) for value in msg.r],
            "p": [float(value) for value in msg.p],
            "binning_x": int(msg.binning_x),
            "binning_y": int(msg.binning_y),
            "roi": {
                "x_offset": int(msg.roi.x_offset),
                "y_offset": int(msg.roi.y_offset),
                "height": int(msg.roi.height),
                "width": int(msg.roi.width),
                "do_rectify": bool(msg.roi.do_rectify),
            },
        }

    @staticmethod
    def _joint_state_to_dict(msg: Optional[JointState]) -> Optional[Dict[str, Any]]:
        if msg is None:
            return None
        return {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "name": list(msg.name),
            "position": [float(value) for value in msg.position],
            "velocity": [float(value) for value in msg.velocity],
            "effort": [float(value) for value in msg.effort],
        }

    def _zero_aware_depth_median(
        self, samples: List[Dict[str, Any]]
    ) -> Tuple[np.ndarray, np.ndarray]:
        min_valid = int(self._p("depth_burst_min_valid_samples"))
        if min_valid < 1 or min_valid > len(samples):
            raise ValueError(
                "depth_burst_min_valid_samples must be between 1 and "
                "depth_burst_size"
            )

        depth_frames_m = []
        for sample in samples:
            depth_m = sample["depth_array"].astype(np.float32) / float(
                sample["depth_scale"]
            )
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            depth_frames_m.append(np.where(valid, depth_m, np.nan))

        stack = np.stack(depth_frames_m, axis=0)
        valid_counts = np.count_nonzero(np.isfinite(stack), axis=0).astype(np.uint8)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            median_depth_m = np.nanmedian(stack, axis=0).astype(np.float32)
        median_depth_m[~np.isfinite(median_depth_m)] = 0.0
        median_depth_m[valid_counts < min_valid] = 0.0
        return np.ascontiguousarray(median_depth_m), valid_counts

    @staticmethod
    def _burst_pose_spread(
        samples: List[Dict[str, Any]]
    ) -> Tuple[float, float]:
        max_translation = 0.0
        max_rotation_deg = 0.0
        for index, first in enumerate(samples):
            first_tf = first["tf_matrix"]
            for second in samples[index + 1 :]:
                second_tf = second["tf_matrix"]
                translation = float(
                    np.linalg.norm(first_tf[:3, 3] - second_tf[:3, 3])
                )
                relative_rotation = first_tf[:3, :3].T @ second_tf[:3, :3]
                trace_term = float(
                    (np.trace(relative_rotation) - 1.0) / 2.0
                )
                rotation_deg = math.degrees(
                    math.acos(float(np.clip(trace_term, -1.0, 1.0)))
                )
                max_translation = max(max_translation, translation)
                max_rotation_deg = max(max_rotation_deg, rotation_deg)
        return max_translation, max_rotation_deg

    def _crop_to_target(
        self,
        cloud: o3d.geometry.PointCloud,
        tf_target_from_camera: np.ndarray,
    ) -> Tuple[o3d.geometry.PointCloud, Optional[Dict[str, List[float]]]]:
        if not bool(self._p("target_crop_enabled")):
            return copy.deepcopy(cloud), None

        if self.target_crop_center is None:
            mode = str(self._p("target_center_mode")).strip().lower()
            if mode == "camera_forward":
                distance = float(self._p("target_distance_m"))
                if distance <= 0.0:
                    raise ValueError(
                        "target_distance_m must be positive in camera_forward mode"
                    )
                center_camera = np.array([0.0, 0.0, distance, 1.0])
                self.target_crop_center = (
                    tf_target_from_camera @ center_camera
                )[:3]
            elif mode == "fixed_target_frame":
                center = np.asarray(
                    self._p("target_center_target_frame_m"), dtype=np.float64
                )
                if center.shape != (3,):
                    raise ValueError(
                        "target_center_target_frame_m must contain exactly 3 values"
                    )
                self.target_crop_center = center
            else:
                raise ValueError(
                    "target_center_mode must be 'camera_forward' or "
                    "'fixed_target_frame'"
                )
            self.get_logger().info(
                "Locked target crop center in "
                f"{self.target_frame}: {self.target_crop_center.tolist()}"
            )

        box_size = np.asarray(self._p("target_box_size_m"), dtype=np.float64)
        if box_size.shape != (3,) or np.any(box_size <= 0.0):
            raise ValueError(
                "target_box_size_m must contain 3 positive X/Y/Z dimensions"
            )
        minimum = self.target_crop_center - box_size / 2.0
        maximum = self.target_crop_center + box_size / 2.0
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(minimum, maximum)
        cropped = cloud.crop(bounding_box)
        return cropped, {
            "min": minimum.tolist(),
            "max": maximum.tolist(),
            "center": self.target_crop_center.tolist(),
            "size": box_size.tolist(),
            "frame": self.target_frame,
        }

    def _save_raw_capture(
        self,
        *,
        request_id: int,
        samples: List[Dict[str, Any]],
        median_depth_m: np.ndarray,
        valid_counts: np.ndarray,
        selected_index: int,
    ) -> Tuple[Path, Path]:
        attempt = self.capture_attempts.get(request_id, 0) + 1
        while True:
            suffix = "" if attempt == 1 else f"_attempt_{attempt:02d}"
            capture_dir = self.raw_dir / f"capture_{request_id:04d}{suffix}"
            if not capture_dir.exists():
                break
            attempt += 1
        self.capture_attempts[request_id] = attempt
        capture_dir.mkdir(parents=True, exist_ok=False)

        sample_metadata = []
        for index, sample in enumerate(samples):
            stem = f"frame_{index:02d}"
            color_name = f"{stem}_color.png"
            depth_npy_name = f"{stem}_depth.npy"
            color_ok = cv2.imwrite(
                str(capture_dir / color_name),
                cv2.cvtColor(sample["color_rgb"], cv2.COLOR_RGB2BGR),
            )
            if not color_ok:
                raise OSError(f"Failed to save {capture_dir / color_name}")
            np.save(capture_dir / depth_npy_name, sample["depth_array"])

            depth_png_name: Optional[str] = None
            if sample["depth_array"].dtype == np.uint16:
                depth_png_name = f"{stem}_depth.png"
                if not cv2.imwrite(
                    str(capture_dir / depth_png_name), sample["depth_array"]
                ):
                    raise OSError(
                        f"Failed to save {capture_dir / depth_png_name}"
                    )

            sample_metadata.append(
                {
                    "index": index,
                    "stamp_sec": sample["stamp_sec"],
                    "stamp_nanosec": sample["stamp_nanosec"],
                    "stamp_ns": sample["stamp_ns"],
                    "camera_frame": sample["camera_frame"],
                    "color_frame_id": sample["color_frame_id"],
                    "depth_frame_id": sample["depth_frame_id"],
                    "color_encoding": sample["color_encoding"],
                    "depth_encoding": sample["depth_encoding"],
                    "depth_dtype": str(sample["depth_array"].dtype),
                    "depth_scale_raw_units_per_meter": sample["depth_scale"],
                    "tf_target_from_camera": sample["tf_matrix"].tolist(),
                    "joint_state_snapshot": sample["joint_state"],
                    "color_file": color_name,
                    "depth_npy_file": depth_npy_name,
                    "depth_png_file": depth_png_name,
                }
            )

        selected_color_name = "selected_color.png"
        if not cv2.imwrite(
            str(capture_dir / selected_color_name),
            cv2.cvtColor(
                samples[selected_index]["color_rgb"], cv2.COLOR_RGB2BGR
            ),
        ):
            raise OSError(
                f"Failed to save {capture_dir / selected_color_name}"
            )
        np.save(capture_dir / "median_depth_m.npy", median_depth_m)
        np.save(capture_dir / "median_valid_counts.npy", valid_counts)
        median_mm = np.rint(
            np.clip(median_depth_m * 1000.0, 0.0, np.iinfo(np.uint16).max)
        ).astype(np.uint16)
        if not cv2.imwrite(
            str(capture_dir / "median_depth_mm.png"), median_mm
        ):
            raise OSError(
                f"Failed to save {capture_dir / 'median_depth_mm.png'}"
            )

        metadata = {
            "schema_version": 1,
            "request_id": request_id,
            "attempt": attempt,
            "request_attempt": self.active_attempt_number,
            "request_max_attempts": int(self._p("capture_max_attempts")),
            "accepted": None,
            "reason": "raw burst saved before filtering",
            "target_frame": self.target_frame,
            "camera_frame": samples[selected_index]["camera_frame"],
            "sample_count": len(samples),
            "minimum_valid_samples": int(
                self._p("depth_burst_min_valid_samples")
            ),
            "selected_sample_index": selected_index,
            "selected_color_file": selected_color_name,
            "median_depth_m_file": "median_depth_m.npy",
            "median_depth_mm_file": "median_depth_mm.png",
            "median_valid_counts_file": "median_valid_counts.npy",
            "median_valid_pixel_fraction": float(
                np.count_nonzero(median_depth_m) / median_depth_m.size
            ),
            "minimum_depth_m_applied_to_cloud": float(
                self._p("min_depth_m")
            ),
            "maximum_depth_m_applied_to_cloud": float(
                self._p("max_depth_m")
            ),
            "camera_info": samples[selected_index]["camera_info"],
            "samples": sample_metadata,
        }
        metadata_path = capture_dir / "capture.json"
        with open(metadata_path, "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)
        return capture_dir, metadata_path

    @staticmethod
    def _update_capture_metadata(
        metadata_path: Path, updates: Dict[str, Any]
    ) -> None:
        with open(metadata_path, "r", encoding="utf-8") as handle:
            metadata = json.load(handle)
        metadata.update(updates)
        with open(metadata_path, "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)

    def _write_run_config(self) -> None:
        parameter_names = list(self.get_parameters_by_prefix('').keys())
        parameters: Dict[str, Any] = {}
        for name in parameter_names:
            value = self.get_parameter(name).value
            parameters[name] = list(value) if isinstance(value, tuple) else value
        config = {
            "schema_version": 1,
            "capture_protocol": {
                "request": "std_msgs/msg/UInt32; positive pose id",
                "result": (
                    "std_msgs/msg/Int32; +id accepted, -id rejected"
                ),
            },
            "parameters": parameters,
            "registration_scales": self.registration_scales,
        }
        with open(self.run_config_path, "w", encoding="utf-8") as handle:
            json.dump(config, handle, indent=2)

    def _rgbd_to_cloud(
        self,
        color_rgb: np.ndarray,
        depth_array: np.ndarray,
        depth_scale: float,
        intrinsic: o3d.camera.PinholeCameraIntrinsic,
    ) -> o3d.geometry.PointCloud:
        color_rgb = np.asarray(color_rgb, dtype=np.uint8)
        depth_array = np.asarray(depth_array)

        if depth_array.ndim != 2:
            raise ValueError(
                f"Depth image must be 2D, got shape {depth_array.shape}"
            )

        if color_rgb.ndim != 3 or color_rgb.shape[2] != 3:
            raise ValueError(
                f"Color image must have shape HxWx3, got {color_rgb.shape}"
            )

        if color_rgb.shape[:2] != depth_array.shape:
            raise ValueError(
                f"Color/depth size mismatch: "
                f"color={color_rgb.shape[:2]}, depth={depth_array.shape}"
            )

        if depth_scale <= 0.0:
            raise ValueError(f"Invalid depth scale: {depth_scale}")

        # Convert the raw depth units into meters.
        depth_m = depth_array.astype(np.float32) / float(depth_scale)

        height, width = depth_m.shape

        intrinsic_matrix = intrinsic.intrinsic_matrix
        fx = float(intrinsic_matrix[0, 0])
        fy = float(intrinsic_matrix[1, 1])
        cx = float(intrinsic_matrix[0, 2])
        cy = float(intrinsic_matrix[1, 2])

        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(
                f"Invalid camera intrinsics: fx={fx}, fy={fy}"
            )

        # Pixel coordinates.
        u, v = np.meshgrid(
            np.arange(width, dtype=np.float32),
            np.arange(height, dtype=np.float32),
        )

        valid = np.isfinite(depth_m)
        valid &= depth_m > 0.0
        valid &= depth_m >= float(self._p("min_depth_m"))
        valid &= depth_m <= float(self._p("max_depth_m"))

        z = depth_m[valid]

        if z.size == 0:
            return o3d.geometry.PointCloud()

        # Pinhole-camera back-projection:
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy

        points = np.column_stack((x, y, z)).astype(
            np.float64,
            copy=False,
        )

        # Open3D stores colors as floating-point RGB values in [0, 1].
        colors = color_rgb[valid].astype(np.float64) / 255.0

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.ascontiguousarray(points)
        )
        cloud.colors = o3d.utility.Vector3dVector(
            np.ascontiguousarray(colors)
        )

        return cloud

    def _filter_cloud(self, cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        voxel = float(self._p("local_voxel_size_m"))
        filtered = cloud.voxel_down_sample(voxel) if voxel > 0.0 else copy.deepcopy(cloud)
        if not bool(self._p("outlier_removal_enabled")) or len(filtered.points) == 0:
            return filtered

        mode = str(self._p("outlier_filter_mode")).strip().lower()
        if mode == "none":
            return filtered
        if mode == "radius":
            _, indices = filtered.remove_radius_outlier(
                nb_points=int(self._p("outlier_min_neighbors")),
                radius=float(self._p("outlier_radius_m")),
            )
            filtered = filtered.select_by_index(indices)
        elif mode == "statistical":
            _, indices = filtered.remove_statistical_outlier(
                nb_neighbors=int(self._p("statistical_nb_neighbors")),
                std_ratio=float(self._p("statistical_std_ratio")),
            )
            filtered = filtered.select_by_index(indices)
        else:
            raise ValueError(
                "outlier_filter_mode must be one of: none, radius, statistical"
            )
        return filtered

    def _fuse_frame(
        self, pcd_target: o3d.geometry.PointCloud
    ) -> Tuple[bool, np.ndarray, Optional[float], Optional[float], o3d.geometry.PointCloud]:
        filtered = self._filter_cloud(pcd_target)
        identity = np.eye(4, dtype=np.float64)
        if len(filtered.points) == 0:
            return False, identity, None, None, filtered

        if len(self.global_pc.points) == 0:
            self.global_pc = copy.deepcopy(filtered)
            return True, identity, None, None, filtered

        icp_transform = identity
        fitness: Optional[float] = None
        rmse: Optional[float] = None
        accepted = True
        if bool(self._p("use_icp")):
            target = self._local_icp_target(filtered)
            accepted, icp_transform, fitness, rmse = self._multiscale_icp(
                filtered, target
            )
            if accepted and not self._icp_correction_is_sane(icp_transform):
                self.get_logger().warning(
                    "ICP correction exceeded sanity limits; rejecting frame."
                )
                accepted = False

        if not accepted:
            return False, icp_transform, fitness, rmse, filtered

        corrected = copy.deepcopy(filtered)
        corrected.transform(icp_transform)
        self.global_pc += corrected
        global_voxel = float(self._p("global_voxel_size_m"))
        if global_voxel > 0.0:
            self.global_pc = self.global_pc.voxel_down_sample(global_voxel)
        return True, icp_transform, fitness, rmse, corrected

    def _local_icp_target(
        self, source: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        margin = float(self._p("icp_crop_margin_m"))
        source_box = source.get_axis_aligned_bounding_box()
        minimum = source_box.get_min_bound() - margin
        maximum = source_box.get_max_bound() + margin
        target = self.global_pc.crop(o3d.geometry.AxisAlignedBoundingBox(minimum, maximum))
        if len(target.points) < int(self._p("icp_min_target_points")):
            return self.global_pc
        return target

    def _multiscale_icp(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
    ) -> Tuple[bool, np.ndarray, Optional[float], Optional[float]]:
        transform = np.eye(4, dtype=np.float64)
        final_fitness: Optional[float] = None
        final_rmse: Optional[float] = None

        for name in self.SCALE_NAMES:
            params = self.registration_scales[name]
            voxel = params["voxel"]
            if voxel > 0.0:
                source_scale = source.voxel_down_sample(voxel)
                target_scale = target.voxel_down_sample(voxel)
                normal_radius = max(voxel * 2.5, params["correspondence"] * 1.5)
            else:
                source_scale = copy.deepcopy(source)
                target_scale = copy.deepcopy(target)
                normal_radius = max(0.03, params["correspondence"] * 2.0)

            if len(source_scale.points) < 30 or len(target_scale.points) < 30:
                return False, transform, final_fitness, final_rmse

            normal_search = o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius, max_nn=40
            )
            source_scale.estimate_normals(search_param=normal_search)
            target_scale.estimate_normals(search_param=normal_search)

            registration = o3d.pipelines.registration.registration_icp(
                source=source_scale,
                target=target_scale,
                max_correspondence_distance=params["correspondence"],
                init=transform,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_rmse=params["relative_rmse"],
                    relative_fitness=params["relative_fitness"],
                    max_iteration=int(params["iterations"]),
                ),
            )
            transform = registration.transformation
            final_fitness = float(registration.fitness)
            final_rmse = float(registration.inlier_rmse)

            if (
                final_fitness < params["fitness_threshold"]
                or final_rmse > params["rmse_threshold"]
            ):
                self.get_logger().warning(
                    f"ICP {name} failed: fitness={final_fitness:.3f} "
                    f"(min {params['fitness_threshold']:.3f}), "
                    f"rmse={final_rmse * 1000.0:.1f} mm "
                    f"(max {params['rmse_threshold'] * 1000.0:.1f} mm)"
                )
                return False, transform, final_fitness, final_rmse

        return True, transform, final_fitness, final_rmse

    def _icp_correction_is_sane(self, transform: np.ndarray) -> bool:
        translation = float(np.linalg.norm(transform[:3, 3]))
        trace_term = float((np.trace(transform[:3, :3]) - 1.0) / 2.0)
        rotation_deg = math.degrees(math.acos(float(np.clip(trace_term, -1.0, 1.0))))
        return (
            translation <= float(self._p("max_icp_correction_translation_m"))
            and rotation_deg <= float(self._p("max_icp_correction_rotation_deg"))
        )

    def _publish_global_cloud(self, stamp) -> None:
        if len(self.global_pc.points) == 0:
            return
        msg = o3dpc_to_pointcloud2(self.global_pc, self.target_frame, stamp)
        self.pcd_pub.publish(msg)

    def _write_global_cloud(self) -> None:
        if len(self.global_pc.points) == 0:
            return
        success = o3d.io.write_point_cloud(
            str(self.global_cloud_path), self.global_pc, write_ascii=False, compressed=False
        )
        if not success:
            self.get_logger().error(
                f"Open3D failed to write {self.global_cloud_path}"
            )

    def _write_summary(self) -> None:
        summary = {
            "scan_id": self.scan_id,
            "target_frame": self.target_frame,
            "capture_protocol": "+request_id accepted; -request_id rejected",
            "depth_burst_size": int(self._p("depth_burst_size")),
            "depth_burst_min_valid_samples": int(
                self._p("depth_burst_min_valid_samples")
            ),
            "capture_max_attempts": int(self._p("capture_max_attempts")),
            "capture_retry_delay_sec": float(
                self._p("capture_retry_delay_sec")
            ),
            "target_crop_enabled": bool(self._p("target_crop_enabled")),
            "target_crop_center_m": (
                None
                if self.target_crop_center is None
                else self.target_crop_center.tolist()
            ),
            "outlier_filter_mode": str(self._p("outlier_filter_mode")),
            "completed_capture_requests": self.completed_capture_requests,
            "accepted_capture_requests": self.accepted_capture_requests,
            "rejected_capture_requests": self.rejected_capture_requests,
            "total_processed_attempts": self.total_frames,
            "rejected_attempts": self.rejected_frames,
            "total_processed_captures": self.total_frames,
            "accepted_frames": self.accepted_frames,
            "rejected_frames": self.rejected_frames,
            "accepted_percent": (
                100.0 * self.accepted_frames / self.total_frames
                if self.total_frames
                else 0.0
            ),
            "mean_icp_fitness": (
                float(np.mean(self.icp_fitnesses)) if self.icp_fitnesses else None
            ),
            "std_icp_fitness": (
                float(np.std(self.icp_fitnesses)) if self.icp_fitnesses else None
            ),
            "mean_icp_rmse_mm": (
                float(np.mean(self.icp_rmses) * 1000.0) if self.icp_rmses else None
            ),
            "std_icp_rmse_mm": (
                float(np.std(self.icp_rmses) * 1000.0) if self.icp_rmses else None
            ),
            "mean_reconstruction_time_ms": (
                float(np.mean(self.reconstruction_times_ms))
                if self.reconstruction_times_ms
                else None
            ),
            "final_point_count": len(self.global_pc.points),
            "ply_path": str(self.global_cloud_path),
            "raw_capture_directory": str(self.raw_dir),
            "run_config_path": str(self.run_config_path),
        }
        self.scan_dir.mkdir(parents=True, exist_ok=True)
        with open(self.summary_path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
        self.summary_written = True

    def destroy_node(self) -> bool:
        try:
            self._write_global_cloud()
            self._write_summary()
        except Exception as exc:
            self.get_logger().error(f"Final save failed during shutdown: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TreePointCloudReconstructor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
