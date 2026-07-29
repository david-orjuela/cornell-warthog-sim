#!/usr/bin/env python3
"""
Debug Tree Reconstruction & ICP Point Cloud Fusion Node (ROS 2 / rclpy)
========================================================================
Standalone debugging script to verify RGB-D synchronization, TF alignment,
Open3D point cloud generation, and multi-scale ICP registration end-to-end.

Adapted directly from ROS 1 UR5eImagePosePublisher logic.

Topics & Frames:
- Color Image:       /sensors/camera_jetson/color/image_raw
- Aligned Depth:     /sensors/camera_jetson/aligned_depth_to_color/image_rect_raw
- Camera Info:       /sensors/camera_jetson/color/camera_info
- World Frame:       base_link
- Camera Frame:      camera_jetson_color_optical_frame
- Output Debug PCD:  /debug_reconstruction/points
"""

import copy
import os
import sys
import time
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
import tf2_ros

import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, CompressedImage
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2

# ==============================================================================
# Segmentation Model Imports (Commented / Stubbed for Debug Script)
# Un-comment when YOLO or BiSeNet model weights are available in workspace
# ==============================================================================
# from ultralytics import YOLO
# from ultralytics.utils.ops import scale_image
# from bisenetv1_eca_predict import BisenetV1ECA


DEFAULT_REGISTRATION_PARAMS = {
    'Scales': {
        'SN0': {
            'Maximum correspondence distance': 0.02,
            'Inlier RMSE change threshold': 1e-6,
            'Fitness change threshold': 1e-6,
            'Maximum iterations': 50,
            'Inlier RMSE threshold': 0.015,
            'Fitness threshold': 0.3,
        },
        'SN1': {
            'Voxel size': 0.01,
            'Inlier RMSE change threshold': 1e-6,
            'Fitness change threshold': 1e-6,
            'Maximum iterations': 50,
            'Inlier RMSE threshold': 0.015,
            'Fitness threshold': 0.3,
        },
        'SN2': {
            'Voxel size': 0.02,
            'Inlier RMSE change threshold': 1e-6,
            'Fitness change threshold': 1e-6,
            'Maximum iterations': 50,
            'Inlier RMSE threshold': 0.015,
            'Fitness threshold': 0.3,
        },
        'SN3': {
            'Voxel size': 0.04,
            'Inlier RMSE change threshold': 1e-6,
            'Fitness change threshold': 1e-6,
            'Maximum iterations': 50,
            'Inlier RMSE threshold': 0.015,
            'Fitness threshold': 0.3,
        },
    }
}


class DebugReconstructionNode(Node):
    """ROS 2 Node for debugging live RGB-D synchronization, TF lookup, and Open3D ICP stitching."""

    def __init__(self):
        super().__init__('debug_reconstruction_node')

        # Declare ROS 2 parameters
        self.declare_parameter('color_topic', '/sensors/camera_jetson/color/image_raw/compressed')
        self.declare_parameter('depth_topic', '/sensors/camera_jetson/aligned_depth_to_color/image_raw/compressedDepth')
        self.declare_parameter('camera_info_topic', '/sensors/camera_jetson/color/camera_info')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_jetson_color_optical_frame')
        self.declare_parameter('output_topic', '/debug_reconstruction/points')
        self.declare_parameter('slop_seconds', 0.1)
        self.declare_parameter('depth_threshold_mm', 2000.0)
        self.declare_parameter('use_real_colors', True)

        # Retrieve parameters
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.slop_seconds = self.get_parameter('slop_seconds').get_parameter_value().double_value
        self.depth_threshold_mm = self.get_parameter('depth_threshold_mm').get_parameter_value().double_value
        self.use_real_colors = self.get_parameter('use_real_colors').get_parameter_value().bool_value

        self.get_logger().info("==========================================================")
        self.get_logger().info(" Initializing Debug Reconstruction Node (ROS 2 / rclpy)")
        self.get_logger().info(f"  Color Topic:       {self.color_topic}")
        self.get_logger().info(f"  Depth Topic:       {self.depth_topic}")
        self.get_logger().info(f"  Camera Info:       {self.camera_info_topic}")
        self.get_logger().info(f"  World Frame:       {self.world_frame}")
        self.get_logger().info(f"  Camera Frame:      {self.camera_frame}")
        self.get_logger().info(f"  Output Topic:      {self.output_topic}")
        self.get_logger().info(f"  Sync Slop:         {self.slop_seconds} s")
        self.get_logger().info(f"  Use Real Colors:   {self.use_real_colors}")
        self.get_logger().info("==========================================================")

        # CV Bridge
        self.bridge = CvBridge()

        self.declare_parameter('tf_timeout_seconds', 0.2)
        self.declare_parameter('allow_tf_fallback', True)

        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').get_parameter_value().double_value
        self.allow_tf_fallback = self.get_parameter('allow_tf_fallback').get_parameter_value().bool_value

        # TF2 Buffer (30-second cache duration) & Listener (spin_thread=True for dedicated TF ingestion thread)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # Dynamically select message types (CompressedImage vs raw Image)
        ColorMsgType = CompressedImage if 'compressed' in self.color_topic.lower() else Image
        DepthMsgType = CompressedImage if 'compressed' in self.depth_topic.lower() else Image

        # Message Filters Subscribers
        self.color_sub = message_filters.Subscriber(self, ColorMsgType, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, DepthMsgType, self.depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, self.camera_info_topic)

        # Approximate Time Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=self.slop_seconds
        )
        self.ts.registerCallback(self.synced_callback)

        # Publisher for fused debug point cloud
        self.pcd_publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        # Open3D Global Accumulated Point Cloud
        self.global_pc = o3d.geometry.PointCloud()

        # Registration ICP Parameters (ported 1-to-1 from ROS 1)
        self.registration_params = DEFAULT_REGISTRATION_PARAMS
        self.scales = ['SN0', 'SN1', 'SN2', 'SN3']

        # Statistics & Logging Counters
        self.total_frames = 0
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.tf_exact_success_count = 0
        self.tf_fallback_success_count = 0
        self.tf_fail_count = 0
        self.tf_fail_future_count = 0
        self.tf_fail_past_count = 0
        self.tf_fail_missing_frame_count = 0
        self.tf_fail_other_count = 0
        self.last_accepted_time = None
        self.icp_fitnesses = []
        self.icp_rmses = []
        self.processing_times_ms = []

        # Periodic Pipeline Health Summary Timer (every 5.0 seconds)
        self.health_timer = self.create_timer(5.0, self.log_health_summary)

        # Bit shifting for RGB point encoding
        self.BIT_MOVE_16 = 2 ** 16
        self.BIT_MOVE_8 = 2 ** 8

        # Default class colors (BGR format for segmentation rendering)
        self.colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

        # ==============================================================================
        # Model Initialization Stub
        # ==============================================================================
        self.model = None  # Placeholder for YOLO(model_path) or BisenetV1ECA(model_path)
        self.get_logger().info("Segmentation model currently STUBBED (using green pass-through mask for debugging).")

    def segment_image(self, color_image_bgr):
        """Segmentation placeholder method.
        
        Produces a default green mask [0, 255, 0] for all valid pixels so that
        the Open3D point cloud generation receives colorized data without requiring
        CUDA/YOLO weights during debugging.
        
        To re-enable real YOLO/BiSeNet inference:
        -----------------------------------------
        1. Un-comment model import at top of file
        2. Initialize self.model in __init__()
        3. Replace stub block below with:
           result = self.model.predict(source=color_image_bgr, conf=0.3)[0]
           # process masks & map class_id to self.colors
        """
        # === STUBBED SEGMENTATION (Green class mask for testing) ===
        h, w = color_image_bgr.shape[:2]
        seg_image = np.zeros((h, w, 3), dtype=np.uint8)
        seg_image[:, :] = (0, 255, 0)  # Green color mapping (Class ID 1)
        return seg_image

    def transform_to_matrix(self, transform):
        """Convert geometry_msgs.msg.Transform to a 4x4 homogenous matrix."""
        tx = transform.translation.x
        ty = transform.translation.y
        tz = transform.translation.z
        rx = transform.rotation.x
        ry = transform.rotation.y
        rz = transform.rotation.z
        rw = transform.rotation.w

        rot_mat = R.from_quat([rx, ry, rz, rw]).as_matrix()
        matrix = np.eye(4)
        matrix[0:3, 0:3] = rot_mat
        matrix[0:3, 3] = [tx, ty, tz]
        return matrix

    def synced_callback(self, color_msg, depth_msg, cam_info_msg):
        """Synchronized callback invoked when color, depth, and camera_info arrive together."""
        self.total_frames += 1
        t_start = time.time()
        cam_stamp = cam_info_msg.header.stamp

        # Calculate time delta between frame timestamp and current wall clock time
        now_ns = self.get_clock().now().nanoseconds
        cam_ns = cam_stamp.sec * 1_000_000_000 + cam_stamp.nanosec
        delta_t_sec = (now_ns - cam_ns) / 1e9

        self.get_logger().info(
            f"\n--- [Frame #{self.total_frames}] Synced Pair Received ---"
        )
        self.get_logger().info(
            f"  Camera Stamp: {cam_stamp.sec}.{cam_stamp.nanosec:09d} | Latency: {delta_t_sec:.3f} s"
        )

        # Log TF buffer staleness relative to node wall-clock time
        try:
            latest_tf = self.tf_buffer.lookup_transform(self.world_frame, self.camera_frame, rclpy.time.Time())
            latest_tf_sec = latest_tf.header.stamp.sec + latest_tf.header.stamp.nanosec * 1e-9
            now_sec = now_ns / 1e9
            staleness_sec = now_sec - latest_tf_sec
            self.get_logger().info(
                f"  TF Buffer Freshness: Latest TF is {staleness_sec:+.3f}s relative to wall clock (stamp: {latest_tf_sec:.3f})"
            )
        except Exception:
            pass

        # Lookup TF: world_frame ('base_link') -> camera_frame ('camera_jetson_color_optical_frame')
        tf_pc = None
        try:
            # Attempt exact timestamp lookup with configurable timeout
            transform_stamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                cam_stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_seconds)
            )
            tf_pc = self.transform_to_matrix(transform_stamped.transform)
            self.tf_exact_success_count += 1
            self.get_logger().info(
                f"  TF Lookup SUCCESS (Exact Stamp): {self.world_frame} -> {self.camera_frame} "
                f"at {cam_stamp.sec}.{cam_stamp.nanosec:09d}"
            )
        except Exception as e:
            self.tf_fail_count += 1
            err_msg = str(e)
            err_type = type(e).__name__

            # Categorize specific failure mode
            if "extrapolation into the future" in err_msg.lower() or "into the future" in err_msg.lower():
                self.tf_fail_future_count += 1
                cat = "EXTRAPOLATION_FUTURE (Camera timestamp ahead of available TF)"
            elif "extrapolation into the past" in err_msg.lower() or "into the past" in err_msg.lower():
                self.tf_fail_past_count += 1
                cat = "EXTRAPOLATION_PAST (Camera timestamp older than TF buffer)"
            elif "does not exist" in err_msg.lower() or "lookup" in err_type.lower():
                self.tf_fail_missing_frame_count += 1
                cat = "MISSING_FRAME / CONNECTIVITY"
            else:
                self.tf_fail_other_count += 1
                cat = f"OTHER ({err_type})"

            # Log exact lookup failure at debug/info level if fallback is enabled
            if self.allow_tf_fallback:
                self.get_logger().debug(f"  TF Exact Lookup FAILED [{cat}]: {err_msg}")
            else:
                self.get_logger().warn(f"  TF Exact Lookup FAILED [{cat}]: {err_msg}")

            # Optional fallback: lookup latest available transform if enabled for debugging
            if self.allow_tf_fallback:
                try:
                    fallback_stamped = self.tf_buffer.lookup_transform(
                        self.world_frame,
                        self.camera_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    tf_pc = self.transform_to_matrix(fallback_stamped.transform)
                    self.tf_fallback_success_count += 1
                    fb_ns = fallback_stamped.header.stamp.sec * 1_000_000_000 + fallback_stamped.header.stamp.nanosec
                    skew_sec = (fb_ns - cam_ns) / 1e9
                    self.get_logger().info(
                        f"  TF Fallback SUCCESS (Latest Available): Skew={skew_sec:+.3f}s relative to camera stamp"
                    )
                except Exception as fb_err:
                    self.get_logger().warn(f"  TF Lookup Unavailable (Skipping frame): {fb_err}")
                    self.rejected_frames += 1
                    return
            else:
                self.rejected_frames += 1
                return

        # Process image data into Open3D point cloud
        try:
            color_o3d, depth_o3d, intrinsic, color_rgb = self.process_images(
                color_msg, depth_msg, cam_info_msg
            )
        except Exception as e:
            self.rejected_frames += 1
            self.get_logger().error(f"  Image Processing Error: {e}")
            return

        # Create Open3D RGBD Image & Point Cloud with explicit metric conversion
        # depth_scale=1000.0 converts 16UC1 millimeter integers into meters
        # depth_trunc converts depth_threshold_mm to meters cutoff
        depth_trunc_m = self.depth_threshold_mm / 1000.0
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1000.0,
            depth_trunc=depth_trunc_m,
            convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        
        # Transform point cloud into world_frame ('base_link')
        pcd.transform(tf_pc)

        # Pre-filter incoming point cloud before registration
        pcd = pcd.voxel_down_sample(voxel_size=0.008)
        pcd = self.outlier_removal(pcd)

        # Sanity check: Log coordinate bounds for the first accepted frame
        pts = np.asarray(pcd.points)
        if self.accepted_frames == 0 and len(pts) > 0:
            self.get_logger().info(
                f"  [First Frame PCD Bounds in {self.world_frame}]\n"
                f"    X: [{pts[:, 0].min():.3f}, {pts[:, 0].max():.3f}] m\n"
                f"    Y: [{pts[:, 1].min():.3f}, {pts[:, 1].max():.3f}] m\n"
                f"    Z: [{pts[:, 2].min():.3f}, {pts[:, 2].max():.3f}] m\n"
                f"    Point Count: {len(pts)}"
            )

        # Multi-stage ICP Registration
        if len(self.global_pc.points) > 0:
            tf_icp, evaluation, fitness, rmse = self.register_pc(pcd, self.global_pc, 3, np.eye(4))
            if evaluation == 'y':
                pcd.transform(tf_icp)
                self.global_pc += pcd
                self.global_pc = self.voxel_downsample_based_on_class(self.global_pc)

                self.accepted_frames += 1
                self.last_accepted_time = time.time()
                self.icp_fitnesses.append(fitness)
                self.icp_rmses.append(rmse)

                self.get_logger().info(
                    f"  ICP Result: ACCEPTED | Fitness={fitness:.4f} | RMSE={rmse*1000.0:.2f} mm"
                )
            else:
                self.rejected_frames += 1
                fit_val = f"{fitness:.4f}" if fitness is not None else "N/A"
                rmse_val = f"{rmse*1000.0:.2f} mm" if rmse is not None else "N/A"
                self.get_logger().warn(
                    f"  ICP Result: REJECTED | Fitness={fit_val} (req: >=0.300), RMSE={rmse_val} (req: <=15.00 mm)"
                )
        else:
            # First frame: initialize global point cloud
            self.global_pc += pcd
            self.accepted_frames += 1
            self.last_accepted_time = time.time()
            self.get_logger().info("  ICP Result: FIRST FRAME ACCEPTED (Global cloud initialized)")
        # Convert global fused point cloud to ROS 2 PointCloud2 msg and publish
        result_pc = copy.copy(self.global_pc)
        result_pc = self.remove_black_colored_points(result_pc)
        
        ros_pcd_msg = self.o3dpc_to_rospc(result_pc, frame_id=self.world_frame, stamp=cam_stamp)
        self.pcd_publisher.publish(ros_pcd_msg)

        elapsed_ms = (time.time() - t_start) * 1000.0
        self.processing_times_ms.append(elapsed_ms)

        self.get_logger().info(
            f"  Published Fused Cloud: {len(result_pc.points)} points on '{self.output_topic}' "
            f"(Processing Time: {elapsed_ms:.1f} ms | Accepted: {self.accepted_frames}/{self.total_frames})"
        )

    def log_health_summary(self):
        """Periodic pipeline health snapshot printed every 5 seconds via rclpy timer."""
        total_synced = self.total_frames
        tf_total_success = self.tf_exact_success_count + self.tf_fallback_success_count
        tf_fail = self.tf_fail_count

        accepted = self.accepted_frames
        rejected = self.rejected_frames
        total_icp_attempts = accepted + rejected
        accept_pct = (accepted / total_icp_attempts * 100.0) if total_icp_attempts > 0 else 0.0
        reject_pct = (rejected / total_icp_attempts * 100.0) if total_icp_attempts > 0 else 0.0

        global_pts = len(self.global_pc.points)

        # ICP fitness stats
        if self.icp_fitnesses:
            mean_fit = float(np.mean(self.icp_fitnesses))
            min_fit = float(np.min(self.icp_fitnesses))
            max_fit = float(np.max(self.icp_fitnesses))
            fit_str = f"{mean_fit:.3f} (min: {min_fit:.3f}, max: {max_fit:.3f})"
        else:
            fit_str = "N/A"

        # ICP RMSE stats (converted to mm)
        if self.icp_rmses:
            rmses_mm = [r * 1000.0 for r in self.icp_rmses]
            mean_rmse = float(np.mean(rmses_mm))
            min_rmse = float(np.min(rmses_mm))
            max_rmse = float(np.max(rmses_mm))
            rmse_str = f"{mean_rmse:.1f}mm (min: {min_rmse:.1f}mm, max: {max_rmse:.1f}mm)"
        else:
            rmse_str = "N/A"

        # Mean processing time
        mean_proc_time = float(np.mean(self.processing_times_ms)) if self.processing_times_ms else 0.0

        # Time since last accepted frame
        if self.last_accepted_time is not None:
            elapsed_since_accept = time.time() - self.last_accepted_time
            time_since_accept_str = f"{elapsed_since_accept:.1f}s"
        else:
            time_since_accept_str = "N/A (no frames accepted yet)"

        tf_total_success = self.tf_exact_success_count + self.tf_fallback_success_count
        tf_breakdown_str = (
            f"ExtrapolateFuture: {self.tf_fail_future_count}, "
            f"ExtrapolatePast: {self.tf_fail_past_count}, "
            f"MissingFrame: {self.tf_fail_missing_frame_count}, "
            f"Other: {self.tf_fail_other_count}"
        )

        self.get_logger().info(
            f"\n==================== PIPELINE HEALTH (every 5s) ====================\n"
            f"Frames synced: {total_synced} | TF success: {tf_total_success} (exact: {self.tf_exact_success_count}, fallback: {self.tf_fallback_success_count}) | TF failed: {tf_fail} ({tf_breakdown_str})\n"
            f"ICP accepted: {accepted} ({accept_pct:.1f}%) | rejected: {rejected} ({reject_pct:.1f}%)\n"
            f"Global cloud size: {global_pts:,} points\n"
            f"Mean ICP fitness: {fit_str} | Mean inlier RMSE: {rmse_str}\n"
            f"Mean processing time: {mean_proc_time:.1f} ms/frame\n"
            f"Time since last accepted frame: {time_since_accept_str}\n"
            f"======================================================================"
        )

    def decompress_color_image(self, color_msg):
        """Decompress color image from CompressedImage or raw Image message."""
        if isinstance(color_msg, CompressedImage):
            try:
                img = self.bridge.compressed_imgmsg_to_cv2(color_msg, "bgr8")
                if img is not None:
                    return img
            except Exception:
                pass
            raw_bytes = bytes(color_msg.data)
            color_buf = np.frombuffer(raw_bytes, dtype=np.uint8)
            return cv2.imdecode(color_buf, cv2.IMREAD_COLOR)
        else:
            color_bgra = self.bridge.imgmsg_to_cv2(color_msg, "bgra8")
            return cv2.cvtColor(color_bgra, cv2.COLOR_BGRA2BGR)

    def decompress_depth_image(self, depth_msg):
        """Decompress depth image from CompressedImage (compressedDepth PNG) or raw Image message."""
        if not isinstance(depth_msg, CompressedImage):
            return self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        # Attempt cv_bridge passthrough
        try:
            depth_img = self.bridge.compressed_imgmsg_to_cv2(depth_msg, "passthrough")
            if depth_img is not None:
                return depth_img
        except Exception:
            pass

        # Search for PNG magic header (\x89PNG\r\n\x1a\n) inside compressedDepth payload
        raw_bytes = bytes(depth_msg.data)
        png_header = b'\x89PNG\r\n\x1a\n'
        png_idx = raw_bytes.find(png_header)
        if png_idx != -1:
            depth_buf = np.frombuffer(raw_bytes[png_idx:], dtype=np.uint8)
            depth_img = cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                return depth_img

        # Fallback to direct imdecode on payload or offset 12
        if len(raw_bytes) > 12:
            depth_buf = np.frombuffer(raw_bytes[12:], dtype=np.uint8)
            depth_img = cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                return depth_img

        depth_buf = np.frombuffer(raw_bytes, dtype=np.uint8)
        return cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)

    def process_images(self, color_msg, depth_msg, cam_info):
        """Convert ROS 2 raw or compressed image messages to Open3D Image objects with intrinsic pinhole camera model."""
        color_image_rgb = self.decompress_color_image(color_msg)
        depth_image = self.decompress_depth_image(depth_msg)

        if color_image_rgb is None:
            raise ValueError("Color image decompression failed (returned NoneType)")
        if depth_image is None:
            raise ValueError("Depth image decompression failed (returned NoneType)")

        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]

        # Apply depth thresholding
        depth_image[depth_image > self.depth_threshold_mm] = 0

        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]
        width = cam_info.width
        height = cam_info.height

        segmented_image = self.segment_image(color_image_rgb)

        if self.use_real_colors:
            # Convert BGR to RGB format for Open3D point cloud coloring
            o3d_color_source = cv2.cvtColor(color_image_rgb, cv2.COLOR_BGR2RGB)
        else:
            o3d_color_source = segmented_image

        color_image_o3d = o3d.geometry.Image(o3d_color_source)
        depth_image_o3d = o3d.geometry.Image(depth_image)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

        return color_image_o3d, depth_image_o3d, intrinsic, color_image_rgb

    def remove_black_colored_points(self, pcd):
        """Filter out uncolored/black background points."""
        colors = np.asarray(pcd.colors)
        if len(colors) == 0:
            return pcd
        mask = ~np.all(colors == 0, axis=1)
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[mask])
        pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        return pcd

    def outlier_removal(self, pcd, nb_points=8, radius=0.014):
        """Remove statistical radius outliers."""
        if len(pcd.points) == 0:
            return pcd
        cl, ind = pcd.remove_radius_outlier(nb_points, radius)
        return pcd.select_by_index(ind)

    def pcd_by_color(self, pcd, color):
        """Extract point cloud subset matching target color."""
        colors = np.asarray(pcd.colors)
        if len(colors) == 0:
            return pcd
        indices = np.where(np.all(colors == color, axis=1))[0]
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[indices])
        if len(np.array(pcd.normals)) > 0:
            pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[indices])
        pcd.colors = o3d.utility.Vector3dVector(colors[indices])
        return pcd

    def voxel_downsample_based_on_class(self, pcd):
        """Apply class-aware voxel downsampling to maintain tree structural density."""
        if len(pcd.points) == 0:
            return pcd

        if self.use_real_colors:
            return pcd.voxel_down_sample(voxel_size=0.005)

        trunk_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 0, 1])
        primary_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 1, 0])
        secondary_pc = self.pcd_by_color(copy.deepcopy(pcd), [1, 0, 0])
        noise_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 0, 0])

        noise_pc = noise_pc.voxel_down_sample(voxel_size=0.005)
        trunk_pc += primary_pc
        trunk_pc = trunk_pc.voxel_down_sample(voxel_size=0.005)
        trunk_pc += noise_pc
        trunk_pc += secondary_pc
        return trunk_pc

    def register_pc(self, sourceCP, targetP, scale_number, tf_init):
        """Recursive multi-scale Point-to-Plane ICP Registration (1-to-1 ROS 1 logic)."""
        scale_params = self.registration_params['Scales'][self.scales[scale_number]]
        if scale_number == 0:
            sourceCPk = copy.deepcopy(sourceCP)
            sourceCPk.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            targetPk = copy.deepcopy(targetP)
            targetPk.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            maximumCorrespondenceDistancek = scale_params["Maximum correspondence distance"]
        else:
            voxelSizek = scale_params['Voxel size']
            sourceCPk = sourceCP.voxel_down_sample(voxel_size=voxelSizek)
            sourceCPk.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSizek * 2, max_nn=30))
            targetPk = targetP.voxel_down_sample(voxel_size=voxelSizek)
            targetPk.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSizek * 2, max_nn=30))
            maximumCorrespondenceDistancek = voxelSizek

        inlierRMSEChangeThresholdk = scale_params["Inlier RMSE change threshold"]
        fitnessChangeThresholdk = scale_params["Fitness change threshold"]
        maximumIterationsk = scale_params["Maximum iterations"]
        inlierRMSEThresholdk = scale_params["Inlier RMSE threshold"]
        fitnessThresholdk = scale_params["Fitness threshold"]

        ICPRegistration = o3d.pipelines.registration.registration_icp(
            source=sourceCPk,
            target=targetPk,
            max_correspondence_distance=maximumCorrespondenceDistancek,
            init=tf_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_rmse=inlierRMSEChangeThresholdk,
                relative_fitness=fitnessChangeThresholdk,
                max_iteration=maximumIterationsk
            )
        )

        MfSCPktTPkcalculated = ICPRegistration.transformation
        currentInlierRMSE = ICPRegistration.inlier_rmse
        currentFitness = ICPRegistration.fitness

        if scale_number == 0 and currentInlierRMSE <= inlierRMSEThresholdk and currentFitness >= fitnessThresholdk:
            return MfSCPktTPkcalculated, 'y', currentFitness, currentInlierRMSE
        elif scale_number > 0 and currentInlierRMSE <= inlierRMSEThresholdk and currentFitness >= fitnessThresholdk:
            scale_number -= 1
            return self.register_pc(sourceCP, targetP, scale_number, MfSCPktTPkcalculated)
        else:
            return MfSCPktTPkcalculated, 'n', currentFitness, currentInlierRMSE

    def o3dpc_to_rospc(self, o3dpc, frame_id="base_link", stamp=None):
        """Convert Open3D PointCloud to ROS 2 sensor_msgs/msg/PointCloud2."""
        points = np.asarray(copy.deepcopy(o3dpc.points), dtype=np.float32)
        colors = np.asarray(copy.deepcopy(o3dpc.colors), dtype=np.float32)

        if len(points) == 0:
            header = std_msgs.msg.Header()
            header.frame_id = frame_id
            header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
            msg = PointCloud2()
            msg.header = header
            return msg

        header = std_msgs.msg.Header()
        header.frame_id = frame_id
        header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        if len(colors) == len(points):
            rgb_npy = np.floor(colors * 255.0).astype(np.uint32)
            rgb_packed = (rgb_npy[:, 0] << 16) | (rgb_npy[:, 1] << 8) | rgb_npy[:, 2]

            struct_arr = np.zeros(
                len(points),
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
            )
            struct_arr['x'] = points[:, 0]
            struct_arr['y'] = points[:, 1]
            struct_arr['z'] = points[:, 2]
            struct_arr['rgb'] = rgb_packed

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            return pc2.create_cloud(header, fields, struct_arr)
        else:
            return pc2.create_cloud_xyz32(header, points)

    def log_summary(self):
        """Log overall summary when the node stops."""
        accepted_pct = (self.accepted_frames / self.total_frames * 100.0) if self.total_frames > 0 else 0.0
        mean_fitness = float(np.mean(self.icp_fitnesses)) if self.icp_fitnesses else 0.0
        rmses_mm = [r * 1000.0 for r in self.icp_rmses]
        mean_rmse_mm = float(np.mean(rmses_mm)) if rmses_mm else 0.0
        mean_time = float(np.mean(self.processing_times_ms)) if self.processing_times_ms else 0.0

        tf_total_success = self.tf_exact_success_count + self.tf_fallback_success_count
        self.get_logger().info("\n================ DEBUG RECONSTRUCTION SUMMARY ================")
        self.get_logger().info(f"  Total Frames Received:    {self.total_frames}")
        self.get_logger().info(f"  TF Successes:             {tf_total_success} (exact: {self.tf_exact_success_count}, fallback: {self.tf_fallback_success_count})")
        self.get_logger().info(f"  TF Failures:              {self.tf_fail_count}")
        self.get_logger().info(f"  Accepted Frames:          {self.accepted_frames} ({accepted_pct:.1f}%)")
        self.get_logger().info(f"  Rejected Frames:          {self.rejected_frames}")
        self.get_logger().info(f"  Mean ICP Fitness:         {mean_fitness:.4f}")
        self.get_logger().info(f"  Mean ICP Inlier RMSE:     {mean_rmse_mm:.2f} mm")
        self.get_logger().info(f"  Mean Processing Time:     {mean_time:.1f} ms/frame")
        self.get_logger().info(f"  Final Point Cloud Size:   {len(self.global_pc.points)} points")
        self.get_logger().info("=============================================================\n")


def main(args=None):
    rclpy.init(args=args)
    node = DebugReconstructionNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 1. Print summary log BEFORE destroying node/context so rosout publishes cleanly
        try:
            node.log_summary()
        except Exception:
            pass

        # 2. Stop and join TF listener's dedicated background thread
        try:
            if hasattr(node, 'tf_listener') and node.tf_listener:
                if hasattr(node.tf_listener, 'unregister'):
                    node.tf_listener.unregister()
                if hasattr(node.tf_listener, 'executor') and node.tf_listener.executor:
                    node.tf_listener.executor.shutdown()
                if hasattr(node.tf_listener, 'dedicated_listener_thread') and node.tf_listener.dedicated_listener_thread:
                    node.tf_listener.dedicated_listener_thread.join(timeout=0.5)
        except Exception:
            pass

        # 3. Shutdown executor and destroy node
        try:
            executor.shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        # 4. Safely shutdown rclpy context
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
