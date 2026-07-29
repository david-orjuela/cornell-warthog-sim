#!/usr/bin/env python3
"""
ROS2 Tree Point Cloud Reconstructor
=====================================
Subscribes (compressed):
  /sensors/camera_jetson/color/image_raw/compressed              (CompressedImage)
  /sensors/camera_jetson/aligned_depth_to_color/image_raw/compressedDepth  (CompressedImage)
  /sensors/camera_jetson/aligned_depth_to_color/camera_info     (CameraInfo)

TF:
  arm_0_base_link -> camera_jetson_color_optical_frame  (direct lookup)
  Publish arm_0_tool0 -> camera_jetson_color_optical_frame via the launch file.

Publishes:
  /reconstruction/point_cloud   (PointCloud2, in arm_0_base_link frame)

Services:
  /reconstruction/enable   (std_srvs/SetBool)
  /reconstruction/save     (std_srvs/Trigger)
"""

import copy
import threading

import cv2
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import CompressedImage, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

COLOR_TOPIC   = "/sensors/camera_jetson/color/image_raw/compressed"
DEPTH_TOPIC   = "/sensors/camera_jetson/aligned_depth_to_color/image_raw/compressedDepth"
CAMINFO_TOPIC = "/sensors/camera_jetson/aligned_depth_to_color/camera_info"

BASE_FRAME   = "arm_0_base_link"
CAMERA_FRAME = "camera_jetson_color_optical_frame"  # direct TF lookup — no manual quaternion needed
TOOL_FRAME   = "arm_0_tool0"                        # fallback if CAMERA_FRAME not in TF

# Rigid offset applied ONLY when falling back to TOOL_FRAME [x, y, z, qx, qy, qz, qw]
CAMERA_TO_TOOL_OFFSET = [0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0]

DEPTH_MAX_M = 2.0   # metres; points beyond this are discarded

# Multi-scale ICP parameters (4-scale cascade, coarse -> fine)
ICP_SCALES = {
    "SN3": dict(voxel=0.04, max_iter=30,  rmse_thr=0.020, fit_thr=0.30, rmse_chg=1e-3, fit_chg=1e-3),
    "SN2": dict(voxel=0.02, max_iter=50,  rmse_thr=0.010, fit_thr=0.40, rmse_chg=1e-4, fit_chg=1e-4),
    "SN1": dict(voxel=0.01, max_iter=100, rmse_thr=0.005, fit_thr=0.50, rmse_chg=1e-5, fit_chg=1e-5),
    "SN0": dict(voxel=None, max_iter=200, rmse_thr=0.003, fit_thr=0.60,
                max_corr=0.005, rmse_chg=1e-6, fit_chg=1e-6),
}
SCALE_ORDER = ["SN3", "SN2", "SN1", "SN0"]  # coarse to fine

VOXEL_GLOBAL    = 0.005
VOXEL_NEW_FRAME = 0.010

SAVE_PATH = "/tmp/reconstructed_tree.pcd"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pose_to_matrix(trans, quat) -> np.ndarray:
    mat = tf_transformations.quaternion_matrix(quat)
    mat[:3, 3] = trans
    return mat


def o3d_to_ros2_pc2(pcd: o3d.geometry.PointCloud, frame_id: str, stamp) -> PointCloud2:
    pts = np.asarray(pcd.points, dtype=np.float32)
    has_color = len(pcd.colors) > 0

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(pts)
    msg.is_bigendian = False
    msg.is_dense = True

    fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]

    if has_color:
        fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
        msg.point_step = 16
        c = (np.asarray(pcd.colors) * 255).astype(np.uint32)
        rgb = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
        dt = np.dtype([("x", np.float32), ("y", np.float32),
                       ("z", np.float32), ("rgb", np.uint32)])
        data = np.zeros(len(pts), dtype=dt)
        data["x"] = pts[:, 0]; data["y"] = pts[:, 1]
        data["z"] = pts[:, 2]; data["rgb"] = rgb
    else:
        msg.point_step = 12
        dt = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)])
        data = np.zeros(len(pts), dtype=dt)
        data["x"] = pts[:, 0]; data["y"] = pts[:, 1]; data["z"] = pts[:, 2]

    msg.fields = fields
    msg.row_step = msg.point_step * msg.width
    msg.data = data.tobytes()
    return msg


def decode_compressed_depth(msg: CompressedImage) -> np.ndarray | None:
    """Decode compressedDepth (16UC1 PNG with 12-byte ROS header). Returns uint16 array (mm)."""
    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if len(buf) < 13:
            return None
        depth_img = cv2.imdecode(buf[12:], cv2.IMREAD_UNCHANGED)
        return depth_img
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Reconstruction node
# ---------------------------------------------------------------------------

class TreeReconstructorNode(Node):

    def __init__(self):
        super().__init__("tree_reconstructor")

        self.bridge = CvBridge()
        self.lock   = threading.Lock()

        self.enabled     = False
        self.global_pc   = o3d.geometry.PointCloud()
        self.frame_count = 0

        # Camera-to-tool fallback offset
        t = CAMERA_TO_TOOL_OFFSET[:3]
        q = CAMERA_TO_TOOL_OFFSET[3:]
        self.cam_offset_mat = pose_to_matrix(t, q)

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Synchronized subscribers
        color_sub = Subscriber(self, CompressedImage, COLOR_TOPIC,   qos_profile=sensor_qos)
        depth_sub = Subscriber(self, CompressedImage, DEPTH_TOPIC,   qos_profile=sensor_qos)
        info_sub  = Subscriber(self, CameraInfo,      CAMINFO_TOPIC, qos_profile=sensor_qos)

        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._image_callback)

        self.pc_pub = self.create_publisher(PointCloud2, "/reconstruction/point_cloud", 10)

        self.create_service(SetBool, "/reconstruction/enable", self._enable_srv)
        self.create_service(Trigger,  "/reconstruction/save",   self._save_srv)
        self.create_timer(5.0, self._status_log)

        self.get_logger().info("TreeReconstructorNode ready.")
        self.get_logger().info("  Enable:  ros2 service call /reconstruction/enable std_srvs/srv/SetBool \"data: true\"")
        self.get_logger().info("  Save:    ros2 service call /reconstruction/save std_srvs/srv/Trigger {}")

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _enable_srv(self, request, response):
        with self.lock:
            self.enabled = request.data
        state = "ENABLED" if request.data else "DISABLED"
        self.get_logger().info(f"Reconstruction {state}.")
        response.success = True
        response.message = f"Reconstruction {state}."
        return response

    def _save_srv(self, _request, response):
        with self.lock:
            n = len(self.global_pc.points)
            if n == 0:
                response.success = False
                response.message = "Cloud is empty; nothing saved."
                return response
            cloud_copy = copy.deepcopy(self.global_pc)

        o3d.io.write_point_cloud(SAVE_PATH, cloud_copy)
        self.get_logger().info(f"Saved {n} points -> {SAVE_PATH}")
        response.success = True
        response.message = f"Saved {n} points to {SAVE_PATH}"
        return response

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def _image_callback(self, color_msg, depth_msg, cam_info):
        with self.lock:
            if not self.enabled:
                return

        stamp = cam_info.header.stamp
        tf_base_cam = self._lookup_tf(stamp)
        if tf_base_cam is None:
            return

        try:
            color_bgr = self.bridge.compressed_imgmsg_to_cv2(color_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Color decode error: {e}")
            return

        depth_raw = decode_compressed_depth(depth_msg)
        if depth_raw is None:
            self.get_logger().warn("Failed to decode compressedDepth; skipping frame.")
            return

        depth_thresh_mm = int(DEPTH_MAX_M * 1000)
        depth_raw = depth_raw.copy()
        depth_raw[depth_raw > depth_thresh_mm] = 0

        dh, dw = depth_raw.shape[:2]
        if color_bgr.shape[:2] != (dh, dw):
            color_bgr = cv2.resize(color_bgr, (dw, dh), interpolation=cv2.INTER_LINEAR)

        fx = cam_info.k[0]; fy = cam_info.k[4]
        cx = cam_info.k[2]; cy = cam_info.k[5]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(dw, dh, fx, fy, cx, cy)

        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        color_o3d = o3d.geometry.Image(color_rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(depth_raw.astype(np.uint16))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1000.0,
            depth_trunc=DEPTH_MAX_M,
            convert_rgb_to_intensity=False,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        if len(pcd.points) < 100:
            self.get_logger().warn("Too few points in frame; skipping.")
            return

        pcd.transform(tf_base_cam)

        with self.lock:
            if len(self.global_pc.points) == 0:
                self.global_pc += pcd
                self.global_pc = self.global_pc.voxel_down_sample(VOXEL_GLOBAL)
                self.frame_count += 1
                self.get_logger().info(f"Frame 1 added (seed). Cloud: {len(self.global_pc.points)} pts")
            else:
                tf_icp, success = self._multi_scale_icp(pcd, self.global_pc)
                if success:
                    pcd.transform(tf_icp)
                    pcd = pcd.voxel_down_sample(VOXEL_NEW_FRAME)
                    pcd = self._remove_outliers(pcd)
                    self.global_pc += pcd
                    self.global_pc = self.global_pc.voxel_down_sample(VOXEL_GLOBAL)
                    self.frame_count += 1
                    self.get_logger().info(
                        f"Frame {self.frame_count} registered. Cloud: {len(self.global_pc.points)} pts"
                    )
                else:
                    self.get_logger().warn("ICP did not converge; frame discarded.")

            cloud_copy = copy.deepcopy(self.global_pc)

        ros_pc = o3d_to_ros2_pc2(cloud_copy, frame_id=BASE_FRAME,
                                  stamp=self.get_clock().now().to_msg())
        self.pc_pub.publish(ros_pc)

    # ------------------------------------------------------------------
    # TF: try CAMERA_FRAME first, fall back to TOOL_FRAME + offset
    # ------------------------------------------------------------------

    def _lookup_tf(self, stamp) -> np.ndarray | None:
        candidates = [
            (CAMERA_FRAME, np.eye(4),           "camera optical frame"),
            (TOOL_FRAME,   self.cam_offset_mat,  "tool0 + offset (fallback)"),
        ]
        for target_frame, extra_mat, label in candidates:
            for use_stamp in [stamp, rclpy.time.Time()]:
                try:
                    t = self.tf_buffer.lookup_transform(
                        BASE_FRAME, target_frame, use_stamp,
                        timeout=rclpy.duration.Duration(seconds=0.05),
                    )
                    trans = [t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z]
                    quat  = [t.transform.rotation.x,
                             t.transform.rotation.y,
                             t.transform.rotation.z,
                             t.transform.rotation.w]
                    return pose_to_matrix(trans, quat) @ extra_mat
                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue
        self.get_logger().warn(f"TF unavailable for {CAMERA_FRAME} and {TOOL_FRAME}.")
        return None

    # ------------------------------------------------------------------
    # Multi-scale ICP (coarse to fine: SN3 -> SN0)
    # ------------------------------------------------------------------

    def _multi_scale_icp(self, source: o3d.geometry.PointCloud,
                          target: o3d.geometry.PointCloud,
                          scale_idx: int = len(SCALE_ORDER) - 1,
                          tf_init: np.ndarray = None) -> tuple[np.ndarray, bool]:
        if tf_init is None:
            tf_init = np.eye(4)

        key    = SCALE_ORDER[scale_idx]
        params = ICP_SCALES[key]

        if params["voxel"] is None:
            src_k = copy.deepcopy(source)
            tgt_k = copy.deepcopy(target)
            src_k.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            tgt_k.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            max_corr = params["max_corr"]
        else:
            vox = params["voxel"]
            src_k = source.voxel_down_sample(vox)
            tgt_k = target.voxel_down_sample(vox)
            src_k.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox * 2, max_nn=30))
            tgt_k.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox * 2, max_nn=30))
            max_corr = vox

        result = o3d.pipelines.registration.registration_icp(
            source=src_k, target=tgt_k,
            max_correspondence_distance=max_corr,
            init=tf_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_rmse=params["rmse_chg"],
                relative_fitness=params["fit_chg"],
                max_iteration=params["max_iter"],
            ),
        )

        converged = (result.inlier_rmse <= params["rmse_thr"] and
                     result.fitness     >= params["fit_thr"])

        if converged:
            if scale_idx == 0:
                return result.transformation, True
            else:
                return self._multi_scale_icp(source, target, scale_idx - 1, result.transformation)
        else:
            return np.eye(4), False

    # ------------------------------------------------------------------
    # Outlier removal
    # ------------------------------------------------------------------

    def _remove_outliers(self, pcd: o3d.geometry.PointCloud,
                          nb_points: int = 10,
                          radius: float = 0.02) -> o3d.geometry.PointCloud:
        _, ind = pcd.remove_radius_outlier(nb_points, radius)
        return pcd.select_by_index(ind)

    # ------------------------------------------------------------------
    # Status log
    # ------------------------------------------------------------------

    def _status_log(self):
        with self.lock:
            state = "ENABLED" if self.enabled else "DISABLED"
            n     = len(self.global_pc.points)
            fc    = self.frame_count
        self.get_logger().info(f"Status: {state} | frames={fc} | cloud={n} pts")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TreeReconstructorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted. Saving cloud before exit...")
        with node.lock:
            if len(node.global_pc.points) > 0:
                o3d.io.write_point_cloud(SAVE_PATH, node.global_pc)
                node.get_logger().info(f"Saved -> {SAVE_PATH}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
