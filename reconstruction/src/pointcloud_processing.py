#!/usr/bin/python3
import copy

import copy
import json
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2
import message_filters
import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf
# import torch
# from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
# from curobo.types.base import TensorDeviceType
# from curobo.types.robot import RobotConfig
# from curobo.util_file import get_robot_path, join_path, load_yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointField, PointCloud2
from std_msgs.msg import Bool
from ultralytics import YOLO
from ultralytics.utils.ops import scale_image
from bisenetv1_eca_predict import BisenetV1ECA

# from geometry_msgs.msg import PoseStamped


class UR5eImagePosePublisher:
    def __init__(self):
        rospy.init_node('ur5e_image_pose_publisher', anonymous=True)
        # self.model = BisenetV1ECA(rospy.get_param("/model_path"))
        self.model = YOLO(rospy.get_param("/model_path"))
        self.bridge = CvBridge()
        self.processing_rate = rospy.Rate(10)

        # Subscribers using message_filters
        color_image_sub = message_filters.Subscriber("/rgb/image_raw", Image)
        depth_image_sub = message_filters.Subscriber("/depth_to_rgb/hw_registered/image_rect_raw", Image)
        cam_info_sub = message_filters.Subscriber("/rgb/camera_info", CameraInfo)

        # ApproximateTimeSynchronizer to synchronize messages
        self.ts = message_filters.ApproximateTimeSynchronizer([color_image_sub, depth_image_sub, cam_info_sub], 10, 0.001)
        self.ts.registerCallback(self.callback)

        # PointCloud2 and PoseStamped publishers
        self.pcd_publisher = rospy.Publisher("/output_point_cloud", PointCloud2, queue_size=10)
        self.original_pcd_publisher = rospy.Publisher("/original_point_cloud", PointCloud2, queue_size=10)
        self.ai_image_publisher = rospy.Publisher('/inference', Image, queue_size=10)
        rospy.Subscriber('capture_alert', Bool, self.capture_alert_callback)

        # Robot configuration for pose calculation
        self.joint_positions = [0.0] * 6
        self.start_pose = [0.0] * 7
        self.is_it_start = True

        # Transformation Listeners
        self.tf_listener = tf.TransformListener()
        self.init_transform = None
        self.tf_pc = np.identity(4)
        self.capture_data = True
        self.stamp = rospy.Time(0)

        # pointcloud ops
        self.global_pc = o3d.geometry.PointCloud()
        self.original_pc = o3d.geometry.PointCloud()
        self.registration_params = rospy.get_param('/Registration')
        self.scales = ['SN0', 'SN1', 'SN2', 'SN3']

        # bit operations
        self.BIT_MOVE_16 = 2 ** 16
        self.BIT_MOVE_8 = 2 ** 8

        # segmentation - REMOVE
        # self.colors = [(0, 0, 0), (0, 0, 255), (0, 255, 0), (255, 0, 0)]
        self.colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

        self.image_counter = 0
        
        # Logging & Metrics
        self.icp_fitnesses = []
        self.icp_rmses = []
        self.total_frames = 0
        self.accepted_frames = 0
        self.segmentation_times_ms = []
        self.reconstruction_times_ms = []
        self.skeleton_settings = rospy.get_param('/')
        rospy.on_shutdown(self.log_tree_summary)
        
        import time
        import threading
        self.last_msg_wall_time = time.time()
        self.processed_pcds = []
        self.initial_center = None
        # You can change this to any color (e.g. 'magenta', 'black', 'gold', 'deeppink')
        self.incoming_highlight_color = 'orange'
        self.shutdown_check_thread = threading.Thread(target=self.check_shutdown_loop)
        self.shutdown_check_thread.daemon = True
        self.shutdown_check_thread.start()

    def capture_alert_callback(self, data):
        if data.data:
            self.capture_data = True

    def update_transformation_matrix(self):
        try:
            # Use exact timestamp from camera data
            self.tf_listener.waitForTransform(
                'base_link', 'rgb_camera_link', self.stamp, rospy.Duration(0.5)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                'base_link', 'rgb_camera_link', self.stamp
            )
            self.tf_pc = self.transform_to_matrix(trans, rot)
            return True  
        except Exception as e:
            rospy.logwarn(f"TF not available at timestamp {self.stamp}: {e}")
            return False 

    def publish_image(self, cv_image):
        bridge = CvBridge()
        try:
            ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.ai_image_publisher.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    # def segment_image(self, image):
    #     # run semantic model
    #     mask = self.model.predict(image, return_mask=True)  # uint8 H×W, values 0–3
    #     h, w = mask.shape
    #     seg_image = np.zeros((h, w, 3), np.uint8)

    #     # map each class ID to its color
    #     for class_id, color in enumerate(self.colors):
    #         seg_image[mask == class_id] = color
    #     self.publish_image(seg_image)
    #     return seg_image

    def segment_image(self, image):
        import time
        start_time = time.time()
        # image_filename = f"image_{self.image_counter:04d}.jpg"
        # image_path = os.path.join('/datasets/Reconstruction/images', image_filename)
        # cv2.imwrite(image_path, image)
        self.image_counter += 1
        seg_image = np.zeros((720, 1280, 3), np.uint8)
        result = self.model.predict(source=image, conf=0.3)[0]
        res_plotted = result.plot(conf=False, font_size=0.5, line_width=2)
        self.publish_image(res_plotted)
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy()  # masks, (N, H, W)
            masks = np.moveaxis(masks, 0, -1)  # masks, (H, W, N)
            masks = scale_image(masks, result.masks.orig_shape)
            masks = np.array(masks, dtype=bool)
            cls = result.boxes.cls.cpu().numpy()
            probs = result.boxes.conf.cpu().numpy()
            number_of_instances = masks.shape[2]
            sorted_indices = np.argsort(cls)
            cls = np.array(cls[sorted_indices], dtype=int)
            masks = masks[:, :, sorted_indices]
            for instance in range(number_of_instances):
                class_id = cls[instance]
                seg_image[masks[:, :, instance]] = self.colors[class_id]
        
        duration_ms = (time.time() - start_time) * 1000.0
        self.segmentation_times_ms.append(duration_ms)
        return seg_image
        
    def transform_to_matrix(self, translation, rotation):
        # Create a transformation matrix from translation and rotation (quaternion)
        matrix = tf.transformations.quaternion_matrix(rotation)
        matrix[0:3, 3] = translation
        return matrix

    def callback(self, color_msg, depth_msg, cam_info):
        import time
        self.last_msg_wall_time = time.time()
        if self.capture_data:
            # Handle image processing and point cloud publishing
            try:
                self.stamp = cam_info.header.stamp
                if not self.update_transformation_matrix():
                    rospy.logwarn("Skipping frame due to missing TF")
                    return
                recon_start = time.time()
                self.capture_data = False
                self.total_frames += 1
                original_image, color_image, depth_image, intrinsic, color_image_rgb = self.process_images(color_msg, depth_msg,
                                                                                          cam_info)
            except CvBridgeError as e:
                rospy.logerr(e)
                return

            # Compute and publish point cloud
            self.publish_point_cloud(original_image, color_image, depth_image,
                                     intrinsic, color_image_rgb)
            
            recon_duration_ms = (time.time() - recon_start) * 1000.0
            self.reconstruction_times_ms.append(recon_duration_ms)  # rospy.sleep(0.25)  # self.capture_data = False  # Compute and publish pose  # self.update_joint_positions(joint_state)  # pose = self.compute_pose()  # self.publish_pose(pose)

    def process_images(self, color_msg, depth_msg, cam_info):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgra8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        # Apply depth thresholding
        depth_threshold = 1000  # 2 meters in millimeters
        depth_image[depth_image > depth_threshold] = 0

        fx = cam_info.K[0]
        fy = cam_info.K[4]
        cx = cam_info.K[2]
        cy = cam_info.K[5]
        width = cam_info.width
        height = cam_info.height

        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
        segmented_image = self.segment_image(color_image_rgb)
        original_image_o3d = o3d.geometry.Image(color_image_rgb)
        color_image_o3d = o3d.geometry.Image(segmented_image)
        depth_image_o3d = o3d.geometry.Image(depth_image)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

        return original_image_o3d, color_image_o3d, depth_image_o3d, intrinsic, color_image_rgb

    def remove_black_colored_points(self, pcd):
        colors = np.asarray(pcd.colors)
        mask = ~np.all(colors == 0, axis=1)  
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[mask])
        pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        return pcd

    def save_progressive_frame(self):
        try:
            tree_id = self.get_tree_id()
            output_dir = f"/datasets/paper_figs/progressive_{tree_id}"
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            frame_idx = len(self.processed_pcds) - 1
            if frame_idx < 0:
                return
            if frame_idx % 2 != 0:
                return

            # Compute stable limits based on the first frame center
            if self.initial_center is None:
                first_pts = np.asarray(self.processed_pcds[0].points)
                if len(first_pts) > 0:
                    self.initial_center = first_pts.mean(axis=0)
                else:
                    self.initial_center = np.array([0.0, 0.0, 0.0])
            
            xlim = [self.initial_center[0] - 1.1, self.initial_center[0] + 1.1]
            ylim = [self.initial_center[2] - 1.1, self.initial_center[2] + 1.1]

            fig, ax = plt.subplots(figsize=(10, 10), dpi=300)
            ax.axis("off")
            ax.set_facecolor('white')
            fig.patch.set_facecolor('white')

            # 1. Plot all previously accumulated points in gray (alpha=0.25, smaller size)
            if frame_idx > 0:
                prev_pts_list = [np.asarray(self.processed_pcds[i].points) for i in range(frame_idx)]
                prev_pts = np.vstack(prev_pts_list)
                if len(prev_pts) > 0:
                    ax.scatter(prev_pts[:, 0], prev_pts[:, 2], 
                               c='lightgray', s=2.5, alpha=0.25, edgecolors='none')

            # 2. Plot new incoming frame points in their semantic class colors (alpha=1.0, 2x size)
            new_pts = np.asarray(self.processed_pcds[frame_idx].points)
            new_cols = np.asarray(self.processed_pcds[frame_idx].colors)
            if len(new_pts) > 0:
                ax.scatter(new_pts[:, 0], new_pts[:, 2], 
                           c=new_cols, s=5.0, alpha=1.0, edgecolors='none')

            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            ax.set_aspect('equal')

            filepath = os.path.join(output_dir, f"frame_{frame_idx:04d}.jpg")
            plt.tight_layout(pad=0)
            plt.savefig(filepath, bbox_inches='tight', pad_inches=0.05, dpi=600, facecolor='white')
            plt.close(fig)
            rospy.loginfo(f"Saved progressive frame visualization to {filepath}")
        except Exception as e:
            rospy.logerr(f"Error saving progressive frame: {e}")

    def publish_point_cloud(self, original_image, color_image_o3d, depth_image_o3d, intrinsic, color_image_rgb):
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image_o3d, depth_image_o3d,
                                                                        convert_rgb_to_intensity=False)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        
        pcd.transform(self.tf_pc)
        
        # Initialize tfICP as Identity
        tfICP = np.eye(4)
        
        # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        if len(self.global_pc.points) > 0:
            tfICP, evaluation, fitness, rmse = self.register_pc(pcd, self.global_pc, 3, np.eye(4))
            if evaluation == 'y': 
                pcd.transform(tfICP)
                pcd = pcd.voxel_down_sample(voxel_size=0.008)
                pcd = self.outlier_removal(pcd)
                self.global_pc += pcd
                self.global_pc = self.voxel_downsample_based_on_class(self.global_pc)
                self.accepted_frames += 1
                self.icp_fitnesses.append(fitness)
                self.icp_rmses.append(rmse)
                
                clean_pcd = copy.deepcopy(pcd)
                clean_pcd = self.remove_black_colored_points(clean_pcd)
                self.processed_pcds.append(clean_pcd)
                # self.save_progressive_frame()

        else:
            pcd = pcd.voxel_down_sample(voxel_size=0.01)
            pcd = self.outlier_removal(pcd)
            self.global_pc += pcd
            self.accepted_frames += 1
            
            clean_pcd = copy.deepcopy(pcd)
            clean_pcd = self.remove_black_colored_points(clean_pcd)
            self.processed_pcds.append(clean_pcd)
            # self.save_progressive_frame()

        # Save frame data
        # self.save_frame_data(color_image_rgb, intrinsic, self.tf_pc, tfICP)

        # self.global_pc = self.remove_black_colored_points(self.global_pc)
        result_pc = copy.copy(self.global_pc)
        result_pc = self.remove_black_colored_points(result_pc)
        msg = self.o3dpc_to_rospc(result_pc, frame_id="base_link")

        # Publish the PointCloud2
        self.pcd_publisher.publish(msg)
        self.capture_data = True

    def save_frame_data(self, image, intrinsic, tf_pc, tf_icp):
        """Save image and metadata for the current frame."""
        output_dir = "/datasets/Reconstruction/output/tree25/frames"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # Format counter with 4 digits (e.g., 0000)
        frame_idx = max(0, self.image_counter - 1)
        frame_name = f"{frame_idx:04d}"
        
        # Save Image
        img_path = os.path.join(output_dir, f"{frame_name}.jpg")
        cv2.imwrite(img_path, image)
        
        # Save Metadata
        meta_path = os.path.join(output_dir, f"{frame_name}_meta.json")
        metadata = {
            "intrinsic": intrinsic.intrinsic_matrix.tolist(),
            "tf_pc": tf_pc.tolist(),
            "tf_icp": tf_icp.tolist()
        }
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=4)
        # rospy.loginfo(f"Saved frame data {frame_name}")

    def outlier_removal(self, pcd, nb_points=8, radius=0.014):
        cl, ind = pcd.remove_radius_outlier(nb_points, radius)
        # Display the inlier cloud
        pcd = pcd.select_by_index(ind)
        return pcd

    def pcd_by_color(self, pcd, color):
        colors = np.asarray(pcd.colors)
        indices = np.where(np.all(colors == color, axis=1))[0]
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[indices])
        if len(np.array(pcd.normals)) > 0:
            pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[indices])
        pcd.colors = o3d.utility.Vector3dVector(colors[indices])
        return pcd

    def voxel_downsample_based_on_class(self, pcd): # TODO: update voxel downsampling as was before
        trunk_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 0, 1])
        primary_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 1, 0])
        secondary_pc = self.pcd_by_color(copy.deepcopy(pcd), [1, 0, 0])
        noise_pc = self.pcd_by_color(copy.deepcopy(pcd), [0, 0, 0])

        noise_pc = noise_pc.voxel_down_sample(voxel_size=0.005)
        # trunk_pc = trunk_pc.voxel_down_sample(voxel_size=0.003)
        # primary_pc = primary_pc.voxel_down_sample(voxel_size=0.002)

        # noise_pc += trunk_pc
        trunk_pc += primary_pc
        trunk_pc = trunk_pc.voxel_down_sample(voxel_size = 0.005)
        trunk_pc += noise_pc
        trunk_pc += secondary_pc
        return trunk_pc

    def register_pc(self, sourceCP, targetP, scale_number, tf_init):
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
        ICPRegistration = o3d.pipelines.registration.registration_icp(source=sourceCPk, target=targetPk,
                                                                      max_correspondence_distance=maximumCorrespondenceDistancek,
                                                                      init=tf_init,
                                                                      estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                                      criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                          relative_rmse=inlierRMSEChangeThresholdk,
                                                                          relative_fitness=fitnessChangeThresholdk,
                                                                          max_iteration=maximumIterationsk))
        MfSCPktTPkcalculated = ICPRegistration.transformation
        currentInlierRMSE = ICPRegistration.inlier_rmse
        currentFitness = ICPRegistration.fitness
        if scale_number == 0 and currentInlierRMSE <= inlierRMSEThresholdk and currentFitness >= fitnessThresholdk:
            return MfSCPktTPkcalculated, 'y', currentFitness, currentInlierRMSE
        elif scale_number > 0 and currentInlierRMSE <= inlierRMSEThresholdk and currentFitness >= fitnessThresholdk:
            scale_number -= 1
            return self.register_pc(sourceCP, targetP, scale_number, MfSCPktTPkcalculated)
        elif currentInlierRMSE > inlierRMSEThresholdk or currentFitness < fitnessThresholdk:
            return [], 'n', None, None

    def o3dpc_to_rospc(self, o3dpc, frame_id=None, stamp=None):
        cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
        is_color = o3dpc.colors

        n_points = len(cloud_npy[:, 0])
        if is_color:
            data = np.zeros(n_points,
                            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
        else:
            data = np.zeros(n_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        data['x'] = cloud_npy[:, 0]
        data['y'] = cloud_npy[:, 1]
        data['z'] = cloud_npy[:, 2]

        if is_color:
            rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
            rgb_npy = np.floor(rgb_npy * 255)  # nx3 matrix
            rgb_npy = rgb_npy[:, 0] * self.BIT_MOVE_16 + rgb_npy[:, 1] * self.BIT_MOVE_8 + rgb_npy[:, 2]
            rgb_npy = rgb_npy.astype(np.uint32)
            data['rgb'] = rgb_npy

        rospc = ros_numpy.msgify(PointCloud2, data)
        if frame_id is not None:
            rospc.header.frame_id = frame_id

        if stamp is None:
            rospc.header.stamp = rospy.Time.now()
        else:
            rospc.header.stamp = stamp
        rospc.height = 1
        rospc.width = n_points
        rospc.fields = []
        rospc.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        rospc.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        rospc.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))

        if is_color:
            rospc.fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
            rospc.point_step = 16
        else:
            rospc.point_step = 12

        rospc.is_bigendian = False
        rospc.row_step = rospc.point_step * n_points
        rospc.is_dense = True
        return rospc

    def get_tree_id(self):
        bag_path = rospy.get_param('bag_file_path', 'tree15_2024-02-17-15-10-56')
        bag_name = os.path.basename(bag_path)
        return os.path.splitext(bag_name)[0]

    def check_shutdown_loop(self):
        import time
        # Wait until we have started receiving frames
        while not rospy.is_shutdown():
            if self.total_frames > 0:
                break
            time.sleep(1.0)
            
        # Once frames have started, monitor the idle time
        while not rospy.is_shutdown():
            time.sleep(1.0)
            if time.time() - self.last_msg_wall_time > 8.0:
                rospy.loginfo("No new messages received for 8 seconds. Shutting down node...")
                rospy.signal_shutdown("Bag file playback finished")
                break

    def log_tree_summary(self):
        # Calculate stats
        tree_id = self.get_tree_id()
        total_frames = self.total_frames
        accepted_frames = self.accepted_frames
        accepted_percent = (accepted_frames / total_frames * 100.0) if total_frames > 0 else 0.0

        mean_fitness = float(np.mean(self.icp_fitnesses)) if self.icp_fitnesses else 0.0
        std_fitness = float(np.std(self.icp_fitnesses)) if self.icp_fitnesses else 0.0
        
        # Convert RMSE to mm (multiply by 1000)
        rmses_mm = [r * 1000.0 for r in self.icp_rmses]
        mean_rmse_mm = float(np.mean(rmses_mm)) if rmses_mm else 0.0
        std_rmse_mm = float(np.std(rmses_mm)) if rmses_mm else 0.0

        final_points = len(self.global_pc.points)

        summary = {
            "tree_id": tree_id,
            "total_frames": total_frames,
            "accepted_frames": accepted_frames,
            "accepted_frames_percent": accepted_percent,
            "mean_icp_fitness": mean_fitness,
            "std_icp_fitness": std_fitness,
            "mean_icp_inlier_rmse_mm": mean_rmse_mm,
            "std_icp_inlier_rmse_mm": std_rmse_mm,
            "final_point_count": final_points
        }

        # Print to console
        rospy.loginfo("================ TREE RECONSTRUCTION SUMMARY ================")
        for k, v in summary.items():
            rospy.loginfo(f"  {k}: {v}")
        rospy.loginfo("=============================================================")

        # Save to individual summary file
        output_dir = "/datasets/Reconstruction/output"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        single_path = os.path.join(output_dir, f"{tree_id}_summary.json")
        try:
            with open(single_path, 'w') as f:
                json.dump(summary, f, indent=4)
            rospy.loginfo(f"Saved summary to {single_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save summary file: {e}")

        # Append to central log file
        central_path = os.path.join(output_dir, "all_trees_log.json")
        try:
            logs = []
            if os.path.exists(central_path):
                with open(central_path, 'r') as f:
                    try:
                        logs = json.load(f)
                        if not isinstance(logs, list):
                            logs = []
                    except Exception:
                        logs = []
            
            # Remove existing entry for the same tree if exists, then append new one
            logs = [entry for entry in logs if entry.get("tree_id") != tree_id]
            logs.append(summary)
            
            with open(central_path, 'w') as f:
                json.dump(logs, f, indent=4)
            rospy.loginfo(f"Updated central log at {central_path}")
        except Exception as e:
            rospy.logerr(f"Failed to update central log file: {e}")

        # Final skeletonization on the fused point cloud
        import time
        from skeletonization import TreeSegmentation

        skeletonization_time_s = 0.0
        if len(self.global_pc.points) > 0:
            rospy.loginfo("Running final skeletonization for timing...")
            skelet_pc = copy.copy(self.global_pc)
            skelet_pc = self.remove_black_colored_points(skelet_pc)
            
            t_start = time.time()
            try:
                tree_seg = TreeSegmentation(self.skeleton_settings)
                tree_seg.segment(skelet_pc)
                skeletonization_time_s = time.time() - t_start
                rospy.loginfo(f"Final skeletonization completed in {skeletonization_time_s:.4f} seconds.")
            except Exception as e:
                rospy.logerr(f"Final skeletonization failed: {e}")

        # Compute timing statistics
        mean_seg = float(np.mean(self.segmentation_times_ms)) if self.segmentation_times_ms else 0.0
        std_seg = float(np.std(self.segmentation_times_ms)) if self.segmentation_times_ms else 0.0
        
        mean_recon = float(np.mean(self.reconstruction_times_ms)) if self.reconstruction_times_ms else 0.0
        std_recon = float(np.std(self.reconstruction_times_ms)) if self.reconstruction_times_ms else 0.0

        # Log timing data
        rospy.loginfo("================ TIMING METRICS ================")
        rospy.loginfo(f"  Segmentation Time (ms): mean={mean_seg:.4f}, std={std_seg:.4f}")
        rospy.loginfo(f"  Reconstruction Time (ms): mean={mean_recon:.4f}, std={std_recon:.4f}")
        rospy.loginfo(f"  Skeletonization Time (s): {skeletonization_time_s:.4f}")
        rospy.loginfo("================================================")

        # Write to timing_per_tree.csv
        csv_path = os.path.join(output_dir, "timing_per_tree.csv")
        file_exists = os.path.exists(csv_path)
        try:
            with open(csv_path, 'a') as f:
                if not file_exists:
                    f.write("tree_id,mean_segmentation_time_ms,std_segmentation_time_ms,mean_reconstruction_time_ms,std_reconstruction_time_ms,skeletonization_time_s\n")
                f.write(f"{tree_id},{mean_seg:.4f},{std_seg:.4f},{mean_recon:.4f},{std_recon:.4f},{skeletonization_time_s:.4f}\n")
            rospy.loginfo(f"Appended timing data to {csv_path}")
        except Exception as e:
            rospy.logerr(f"Failed to write timing data to CSV: {e}")

        # Generate paper figures
        # try:
        #     self.generate_paper_figures()
        # except Exception as e:
        #     rospy.logerr(f"Failed to generate paper figures: {e}")

    def generate_paper_figures(self):


        output_dir = "/datasets/paper_figs"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        rospy.loginfo("Generating paper figures...")

        if not self.processed_pcds:
            rospy.logwarn("No processed point clouds to generate figures from!")
            return

        # Prepare final global point cloud by stacking all processed_pcds (exactly like progression saves)
        pts_list = [np.asarray(pcd.points) for pcd in self.processed_pcds]
        cols_list = [np.asarray(pcd.colors) for pcd in self.processed_pcds]
        
        final_points = np.vstack(pts_list)
        final_colors = np.vstack(cols_list)

        if len(final_points) == 0:
            rospy.logwarn("Final global point cloud is empty!")
            return

        # Compute limits based on initial center to match the progressive saves exactly
        if self.initial_center is not None:
            xlim = [self.initial_center[0] - 1.1, self.initial_center[0] + 1.1]
            ylim = [self.initial_center[2] - 1.1, self.initial_center[2] + 1.1]
        else:
            min_x, max_x = final_points[:, 0].min(), final_points[:, 0].max()
            min_z, max_z = final_points[:, 2].min(), final_points[:, 2].max()
            pad_x = (max_x - min_x) * 0.05
            pad_z = (max_z - min_z) * 0.05
            xlim = [min_x - pad_x, max_x + pad_x]
            ylim = [min_z - pad_z, max_z + pad_z]

        fig, ax = plt.subplots(figsize=(10, 10), dpi=300)
        ax.axis("off")
        ax.set_facecolor('white')
        fig.patch.set_facecolor('white')

        # Render 2D X-Z projection matching progressive settings exactly (but in original class colors)
        ax.scatter(final_points[:, 0], final_points[:, 2], 
                   c=final_colors, s=2.5, alpha=1.0, edgecolors='none')
        
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_aspect('equal')
        
        filepath = os.path.join(output_dir, "final_reconstruction.jpg")
        
        plt.tight_layout(pad=0)
        plt.savefig(filepath, bbox_inches='tight', pad_inches=0.05, dpi=600, facecolor='white')
        plt.close(fig)
        
        rospy.loginfo(f"Successfully generated final reconstruction figure at {filepath}")

if __name__ == '__main__':
    ur5e_image_pose_publisher = UR5eImagePosePublisher()
    try:
        rospy.spin()
        ur5e_image_pose_publisher.processing_rate.sleep()
    except rospy.ROSInterruptException:
        # cv2.destroyAllWindows()
        pass
