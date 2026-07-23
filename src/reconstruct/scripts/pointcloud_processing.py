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
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, UInt32
from tf2_ros import Buffer, TransformException, TransformListener

from reconstruct.pc_utils import o3dpc_to_pointcloud2


class TreePointCloudReconstructor(Node):
    """Fuse synchronized RGB-D captures into a colored tree point cloud."""

    SCALE_NAMES = ("SN3", "SN2", "SN1", "SN0")

    def __init__(self) -> None:
        super().__init__("tree_point_cloud_reconstructor")
        self._declare_parameters()

        self.bridge = CvBridge()
        self.target_frame = self._p("target_frame")
        self.camera_frame_override = self._p("camera_frame_override")
        self.output_root = Path(os.path.expanduser(self._p("output_root"))).resolve()
        self.scan_id = self._p("scan_id")
        self.scan_dir = self.output_root / self.scan_id
        self.frames_dir = self.scan_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.global_cloud_path = self.scan_dir / "global_point_cloud.ply"
        self.summary_path = self.scan_dir / "summary.json"

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
            Bool, self._p("capture_topic"), self.capture_alert_callback, control_qos
        )
        self.finished_sub = self.create_subscription(
            Bool, self._p("scan_finished_topic"), self.scan_finished_callback, control_qos
        )
        self.capture_done_pub = self.create_publisher(
            UInt32, self._p("capture_done_topic"), control_qos
        )
        self.pcd_pub = self.create_publisher(
            PointCloud2, self._p("output_cloud_topic"), cloud_qos
        )
        self.original_pcd_pub = self.create_publisher(
            PointCloud2, self._p("original_cloud_topic"), cloud_qos
        )

        self.pending_captures = 1 if bool(self._p("capture_on_start")) else 0
        self.processing = False
        self.capture_sequence = 0
        self.frame_index = 0
        self.global_pc = o3d.geometry.PointCloud()

        self.total_frames = 0
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.icp_fitnesses: List[float] = []
        self.icp_rmses: List[float] = []
        self.reconstruction_times_ms: List[float] = []
        self.summary_written = False
        self.registration_scales = self._load_registration_scales()

        self.get_logger().info(
            "Tree reconstructor ready. Waiting for capture triggers on "
            f"'{self._p('capture_topic')}'."
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
            "capture_topic": "/capture_alert",
            "capture_done_topic": "/capture_done",
            "scan_finished_topic": "/scan_finished",
            "output_cloud_topic": "/tree_scan/global_cloud",
            "original_cloud_topic": "/tree_scan/latest_tf_cloud",
            "target_frame": "base_link",
            "camera_frame_override": "",
            "output_root": "~/tree_scans",
            "scan_id": "indoor_model_tree",
            "capture_on_start": False,
            "sync_queue_size": 30,
            "sync_slop_sec": 0.08,
            "tf_timeout_sec": 0.75,
            "min_depth_m": 0.15,
            "max_depth_m": 2.50,
            "depth_scale_override": 0.0,
            "local_voxel_size_m": 0.006,
            "global_voxel_size_m": 0.005,
            "outlier_removal_enabled": True,
            "outlier_radius_m": 0.020,
            "outlier_min_neighbors": 6,
            "use_icp": True,
            "icp_crop_margin_m": 0.15,
            "icp_min_target_points": 200,
            "max_icp_correction_translation_m": 0.10,
            "max_icp_correction_rotation_deg": 12.0,
            "save_frames": True,
            "publish_original_cloud": True,
            "shutdown_on_scan_finished": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        scale_defaults = {
            "SN3": (0.05, 0.05, 50, 0.025, 0.35),
            "SN2": (0.02, 0.03, 30, 0.015, 0.40),
            "SN1": (0.01, 0.018, 20, 0.010, 0.45),
            "SN0": (0.00, 0.012, 15, 0.008, 0.50),
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

    def capture_alert_callback(self, msg: Bool) -> None:
        if msg.data:
            self.pending_captures += 1
            self.get_logger().info(
                f"Capture requested; pending captures: {self.pending_captures}"
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
        if self.pending_captures <= 0 or self.processing:
            return

        self.processing = True
        start_wall = time.perf_counter()
        processed = False
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
                # Keep the capture queued and retry on the next synchronized frame.
                return

            color_rgb, depth_array, depth_scale, intrinsic = self._process_images(
                color_msg, depth_msg, camera_info
            )
            self.get_logger().info(
                f"Depth encoding={depth_msg.encoding}, "
                f"dtype={depth_array.dtype}, "
                f"shape={depth_array.shape}, "
                f"depth_scale={depth_scale}"
            )
            pcd_camera = self._rgbd_to_cloud(
                color_rgb, depth_array, depth_scale, intrinsic
            )
            if len(pcd_camera.points) == 0:
                self.get_logger().warning("Captured RGB-D frame produced no points.")
                return

            pcd_target = copy.deepcopy(pcd_camera)
            pcd_target.transform(tf_matrix)

            if bool(self._p("publish_original_cloud")):
                original_msg = o3dpc_to_pointcloud2(
                    pcd_target, self.target_frame, depth_msg.header.stamp
                )
                self.original_pcd_pub.publish(original_msg)

            accepted, icp_transform, fitness, rmse, filtered_cloud = self._fuse_frame(
                pcd_target
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

            if bool(self._p("save_frames")):
                self._save_frame_data(
                    color_rgb=color_rgb,
                    depth_array=depth_array,
                    intrinsic=intrinsic,
                    tf_matrix=tf_matrix,
                    icp_transform=icp_transform,
                    camera_frame=camera_frame,
                    accepted=accepted,
                    fitness=fitness,
                    rmse=rmse,
                    color_encoding=color_msg.encoding,
                    depth_encoding=depth_msg.encoding,
                )

            self._publish_global_cloud(depth_msg.header.stamp)
            self._write_global_cloud()

            elapsed_ms = (time.perf_counter() - start_wall) * 1000.0
            self.reconstruction_times_ms.append(elapsed_ms)
            status = "accepted" if accepted else "rejected"
            if accepted and fitness is None:
                metric_text = "first frame / TF only"
            elif not accepted and len(filtered_cloud.points) == 0:
                metric_text = "empty cloud after filtering"
            elif fitness is None:
                metric_text = "rejected before ICP"
            else:
                metric_text = (
                    f"fitness={fitness:.3f}, "
                    f"rmse={rmse * 1000.0:.1f} mm"
                )
            self.get_logger().info(
                f"Capture {self.capture_sequence + 1} {status}: {metric_text}; "
                f"global points={len(self.global_pc.points)}; {elapsed_ms:.1f} ms"
            )
            processed = True
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
        except Exception as exc:  # Keep node alive and preserve queued capture.
            self.get_logger().error(
                f"RGB-D reconstruction failed: {exc}\n{traceback.format_exc()}"
            )
        finally:
            if processed:
                self.pending_captures = max(0, self.pending_captures - 1)
                self.capture_sequence += 1
                done = UInt32()
                done.data = self.capture_sequence
                self.capture_done_pub.publish(done)
            self.processing = False

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

    def _process_images(
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

        self.get_logger().info(
            f"Depth input: encoding={depth_msg.encoding}, "
            f"dtype={depth.dtype}, dtype.str={depth.dtype.str}, "
            f"shape={depth.shape}"
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

        depth_m = depth.astype(np.float32) / depth_scale

        valid = np.isfinite(depth_m)
        valid &= depth_m >= float(self._p("min_depth_m"))
        valid &= depth_m <= float(self._p("max_depth_m"))

        depth[~valid] = 0

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

        self.get_logger().info(
            f"Depth normalized: dtype={depth.dtype}, "
            f"dtype.str={depth.dtype.str}, contiguous={depth.flags.c_contiguous}, "
            f"depth_scale={depth_scale}"
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
        if bool(self._p("outlier_removal_enabled")) and len(filtered.points) > 0:
            _, indices = filtered.remove_radius_outlier(
                nb_points=int(self._p("outlier_min_neighbors")),
                radius=float(self._p("outlier_radius_m")),
            )
            filtered = filtered.select_by_index(indices)
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

    def _save_frame_data(
        self,
        *,
        color_rgb: np.ndarray,
        depth_array: np.ndarray,
        intrinsic: o3d.camera.PinholeCameraIntrinsic,
        tf_matrix: np.ndarray,
        icp_transform: np.ndarray,
        camera_frame: str,
        accepted: bool,
        fitness: Optional[float],
        rmse: Optional[float],
        color_encoding: str,
        depth_encoding: str,
    ) -> None:
        frame_name = f"{self.frame_index:04d}"
        self.frame_index += 1
        cv2.imwrite(
            str(self.frames_dir / f"{frame_name}_color.jpg"),
            cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR),
        )
        depth_path = self.frames_dir / f"{frame_name}_depth.png"
        if depth_array.dtype == np.float32:
            # PNG cannot preserve float depth; save meters losslessly as NPY.
            np.save(str(self.frames_dir / f"{frame_name}_depth_m.npy"), depth_array)
        else:
            cv2.imwrite(str(depth_path), depth_array)

        metadata = {
            "accepted": bool(accepted),
            "fitness": None if fitness is None else float(fitness),
            "inlier_rmse_m": None if rmse is None else float(rmse),
            "target_frame": self.target_frame,
            "camera_frame": camera_frame,
            "color_encoding": color_encoding,
            "depth_encoding": depth_encoding,
            "intrinsic": intrinsic.intrinsic_matrix.tolist(),
            "tf_target_from_camera": tf_matrix.tolist(),
            "icp_correction_in_target_frame": icp_transform.tolist(),
        }
        with open(self.frames_dir / f"{frame_name}_meta.json", "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)

    def _write_summary(self) -> None:
        summary = {
            "scan_id": self.scan_id,
            "target_frame": self.target_frame,
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
