#!/usr/bin/python3
import copy
import json
import os
import cv2
import message_filters
import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointField, PointCloud2
from std_msgs.msg import Bool
from utils.pc_utils import o3dpc_to_rospc

class TreePointCloudReconstructor:
    '''
    Image synchronization.
    TF lookup. RGB-D reconstruction.
    ICP registration. Point-cloud fusion.
    Filtering. ROS publishing.
    Metrics. Figure generation.
    '''
    def __init__(self):
        rospy.init_node(
            'tree_point_cloud_reconstructor', 
            anonymous=True
        )
        self.bridge = CvBridge()
        self.processing_rate = rospy.Rate(10)

        # Synchronized Subscribers using message_filters | The node expects:
        
        # 1. A color image
        color_image_sub = message_filters.Subscriber("/rgb/image_raw", Image) 
        
        # 2. A depth image already registered to the RGB image
        depth_image_sub = message_filters.Subscriber("/depth_to_rgb/hw_registered/image_rect_raw", Image) 
        
        # 3. RGB camera intrinsics
        cam_info_sub = message_filters.Subscriber("/rgb/camera_info", CameraInfo)
    

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_image_sub, depth_image_sub, cam_info_sub], 
            10, 
            0.025 # Permitted timestamp difference (extremely tight, changing from 0.001)
        )

        self.ts.registerCallback(self.callback)

        # PointCloud2 and PoseStamped publishers
        self.pcd_publisher = rospy.Publisher("/output_point_cloud", PointCloud2, queue_size=10)
        self.original_pcd_publisher = rospy.Publisher("/original_point_cloud", PointCloud2, queue_size=10) # declared; never used; TODO: publish
        
        # Intended to coordinate scanning with robot movement
        rospy.Subscriber(
            'capture_alert', 
            Bool, 
            self.capture_alert_callback
        )
        # TODO: Fix reconstruction node; starts with capture_data = True, and sets it beack to true after every frame.
        # Therefore, processes frames continuously at the callback rate rather than only after explicit motion events.

        # Robot configuration for pose calculation
        self.joint_positions = [0.0] * 6
        self.start_pose = [0.0] * 7
        self.is_it_start = True

        # Transformation Listeners / TF state
        # Node queries the transform `base_link` <- `rgb_camera_link` at the camera frame timestamp
        # Essential! Supplies initial placement of every frame in a common coordinate system
        self.tf_listener = tf.TransformListener()
        self.init_transform = None
        self.tf_pc = np.identity(4)
        self.capture_data = True # TODO FIX; SEE ABOVE
        self.stamp = rospy.Time(0)

        # Global point cloud
        self.global_pc = o3d.geometry.PointCloud() # eery accepted frame is merged into this object
        self.original_pc = o3d.geometry.PointCloud() # TODO initialized; never used
        self.registration_params = rospy.get_param('/Registration')
        self.scales = ['SN0', 'SN1', 'SN2', 'SN3'] # ROS YAML parameter tree required

        self.image_counter = 0
        
        # Logging & Metrics
        self.icp_fitnesses = []
        self.icp_rmses = []
        self.total_frames = 0
        self.accepted_frames = 0
        self.reconstruction_times_ms = []
        rospy.on_shutdown(self.log_tree_summary) # on exit, automatically logs metrics & writes JSON
        
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

    # Enables the next capture. Currently does not effect; capture is auto re-eabled after each frame.
    def capture_alert_callback(self, data):
        if data.data:
            self.capture_data = True

    # Obtains camera pose at the exact time image was captured
    def update_transformation_matrix(self):
        try:
            # Use exact timestamp from camera data
            self.tf_listener.waitForTransform(
                'base_link', 
                'rgb_camera_link', 
                self.stamp, 
                rospy.Duration(0.5)
            )

            (trans, rot) = self.tf_listener.lookupTransform(
                'base_link', 
                'rgb_camera_link', 
                self.stamp
            )
            self.tf_pc = self.transform_to_matrix(trans, rot)
            return True
            # Returned transform maps camera-frame points into `base_link`
            # p_base = T_base_camera × p_camera

        except Exception as e:
            rospy.logwarn(f"TF not available at timestamp {self.stamp}: {e}")
            return False 
    
    # TODO
    # Missing dependency; TF chain that produces `base_link → ... → tool0 → camera mount → rgb_camera_link`
    '''
    may require:

    UR5e robot description.
    Camera URDF/xacro.
    Hand-eye calibration.
    Static transform publisher.
    Robot driver TF.
    Correct time synchronization.

    Without accurate camera extrinsics, ICP will be forced to repair large pose errors and may fail or drift.
    '''
        
    def transform_to_matrix(self, translation, rotation):
        # Create a transformation matrix from translation and rotation (quaternion)
        '''Standard 4x4 homogeneous transform:
            [R t]
            [0 1]
        '''
        matrix = tf.transformations.quaternion_matrix(rotation)
        matrix[0:3, 3] = translation
        return matrix

    def callback(self, color_msg, depth_msg, cam_info):
            """Entry point for time-synchronized camera messages."""
            import time
            self.last_msg_wall_time = time.time()
            
            # Process image and publish point cloud if system is ready
            if self.capture_data: 
                try:
                    self.stamp = cam_info.header.stamp  # Store the Gazebo frame timestamp
                    
                    # Ensure valid position data exists before processing
                    if not self.update_transformation_matrix():
                        rospy.logwarn("Skipping frame due to missing TF")
                        return
                    
                    recon_start = time.time()
                    self.capture_data = False  # Lock capture state (mark as busy)
                    self.total_frames += 1     # Increment frame counter
                    
                    # Convert raw ROS messages to OpenCV formats
                    color_image, depth_image, intrinsic, color_image_rgb = self.process_images(
                        color_msg, 
                        depth_msg, 
                        cam_info
                    )
                    
                except CvBridgeError as e:
                    rospy.logerr(e)
                    return

                # Build and register the 3D point cloud
                self.publish_point_cloud(
                    color_image, 
                    depth_image, 
                    intrinsic, 
                    color_image_rgb
                )
                
                # Track and append processing duration
                recon_duration_ms = (time.time() - recon_start) * 1000.0
                self.reconstruction_times_ms.append(recon_duration_ms)

    # TODO: Modify so it can produce a natural-color or geometry-only cloud rather than a semantic branch cloud
    def process_images(self, color_msg, depth_msg, cam_info):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgra8") # assumes incoming RGB can be decoded as BGRA
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1") # depth conversion, assumes 16-bit
        # Apply depth thresholding
        depth_threshold = 1000  # 1 meter in millimeters TODO: might be too close
        depth_image[depth_image > depth_threshold] = 0

        # Camera intrinsics
        # focal lengths
        fx = cam_info.K[0]
        fy = cam_info.K[4]
        # principal point
        cx = cam_info.K[2]
        cy = cam_info.K[5]

        width = cam_info.width
        height = cam_info.height

        color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
        color_image_o3d = o3d.geometry.Image(color_image_bgr)
        depth_image_o3d = o3d.geometry.Image(depth_image)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

        return color_image_o3d, depth_image_o3d, intrinsic, color_image_bgr

    # TODO: Disable -- preserving foliage/original geometry.
    def remove_black_colored_points(self, pcd):
        colors = np.asarray(pcd.colors)
        mask = ~np.all(colors == 0, axis=1)  
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[mask])
        pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        return pcd
    
    # TODO: Verify images are used correctly; especially in the actual fused cloud (carrying RGB instead of semantic class)
    def publish_point_cloud(self, color_image_o3d, depth_image_o3d, intrinsic, color_image_rgb):
        rgbd_image = (
            o3d.geometry.RGBDImage.create_from_color_and_depth(
                color=color_image_o3d, 
                depth=depth_image_o3d,
                depth_scale=1000.0,
                convert_rgb_to_intensity=False
            )
        )
        # Produces a point cloud in the RGB camera coordinate frame
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, 
            intrinsic
        )
        
        # Cloud is approx. in `base_link`, this TF transform is the initial registration estimate
        pcd.transform(self.tf_pc) # p_base = T_base_camera × p_camera
        
        # Initialize tfICP as Identity
        tfICP = np.eye(4)
        
        if len(self.global_pc.points) > 0:
            # For every frame after the first:
            tfICP, evaluation, fitness, rmse = self.register_pc(
                pcd, # src is new frame
                self.global_pc, # trgt is all prev accumulated frames
                3,  # begins at scale index 3, progresses toward scale 0
                np.eye(4)
            )
            # T_corrected = T_ICP × T_TF × points

            # Reject poor registration (only frames meeting configured ICP fitness and RMSE thresholds are fused)
            if evaluation == 'y': 
                # Apply ICP correction
                pcd.transform(tfICP) 
                
                # Per-frame filtering, accepted new cloud is:
                pcd = pcd.voxel_down_sample(voxel_size=0.008) # downsampled to 8 mm voxel grid
                pcd = self.outlier_removal(pcd) # radius-filtered

                # Merge
                self.global_pc += pcd # concatenates new points into the global cloud
                self.global_pc = self.global_pc.voxel_down_sample(
                    voxel_size=0.005 # Simple global downsample (not class-filtered since non-semantic)
                )
                
                self.accepted_frames += 1
                self.icp_fitnesses.append(fitness)
                self.icp_rmses.append(rmse)

            # TODO add explicit logs showing frame number, fitness, RMSE, rejection reason
            if evaluation != 'y':
                rospy.logwarn(
                    f"Rejected frame {self.total_frames}: ICP thresholds not met"
                )
                tfICP = np.eye(4)

        else: # first frame
            pcd = pcd.voxel_down_sample(voxel_size=0.01) # 10 mm; inconsistent with later frames TODO: ask Dawood
            pcd = self.outlier_removal(pcd) 
            self.global_pc += pcd

            self.accepted_frames += 1

        # Save frame data
        self.save_frame_data(
            color_image_rgb,
            depth_image_o3d,
            intrinsic,
            self.tf_pc,
            tfICP,
            accepted=(evaluation == 'y'),
            fitness=fitness,
            rmse=rmse,
        )

        # self.global_pc = self.remove_black_colored_points(self.global_pc)
        result_pc = self.global_pc #copy.deepcopy(self.global_pc)
        #result_pc = self.remove_black_colored_points(result_pc)
        msg = o3dpc_to_rospc(
            result_pc, 
            frame_id="base_link"
        )
        
        output_dir = "/datasets/Reconstruction/output/tree1/frames"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        global_cloud_path = os.path.join(
            output_dir,
            "global_point_cloud.ply"
        )

        success = o3d.io.write_point_cloud(
            global_cloud_path,
            self.global_pc
        )

        if not success:
            rospy.logerr(
                f"Failed to save point cloud to {global_cloud_path}"
            )

        # Publish the PointCloud2 in `base_link`
        self.pcd_publisher.publish(msg)
        self.capture_data = True

    def save_frame_data(self, image, depth, intrinsic, tf_pc, tf_icp):
        """Save image and metadata for the current frame."""
        output_dir = "/datasets/Reconstruction/output/tree1/frames"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # Format counter with 4 digits (e.g., 0000)
        frame_idx = self.image_counter
        frame_name = f"{frame_idx:04d}"
        self.image_counter += 1
        
        # Save Image
        img_path = os.path.join(output_dir, f"{frame_name}_image.jpg")
        cv2.imwrite(img_path, image)

        # Save Depth
        depth_path = os.path.join(output_dir, f"{frame_name}_depth.png")
        depth_numpy = np.asarray(depth)
        cv2.imwrite(depth_path, depth_numpy)
        
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
    
    # TODO: Will ened to tune this, alongside voxel size. Likely currently too aggressive for leafy trees.
    def outlier_removal(self, pcd, nb_points=8, radius=0.014):
        # A point only survives if it has enough nearby points (at least 8 neighbors within 14 mm)
        cl, ind = pcd.remove_radius_outlier(nb_points, radius)
        # Display the inlier cloud
        pcd = pcd.select_by_index(ind)
        return pcd

    # Main point cloud alignment algorithm; coarse-to-fine point-to-plane ICP
    def register_pc(self, sourceCP, targetP, scale_number, tf_init):
        """
        Calling it with `scale_number=3` starts at `SN3`.
        \nIf successfull, it recursively calls: `scale_number` -= 1 until reaching `SN0`.
        \nThus, the progression is:
        ```
        SN3 -> SN2 -> SN1 -> SN0
        coarse              fine
        ``` 
        (assuming larger indices correspond to larger voxels.)
        """
        scale_params = self.registration_params['Scales'][self.scales[scale_number]]
        
        if scale_number == 0:  # No downsampling
            sourceCPk = copy.deepcopy(sourceCP)
            targetPk = copy.deepcopy(targetP)

            # Normals used a fixed radius of 0.1 m. 
            search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            sourceCPk.estimate_normals(search_param=search_param)
            targetPk.estimate_normals(search_param=search_param)

            maximumCorrespondenceDistancek = scale_params["Maximum correspondence distance"]
            
        else:  # scale_number > 0
            voxelSizek = scale_params['Voxel size']

            sourceCPk = sourceCP.voxel_down_sample(voxel_size=voxelSizek)
            targetPk = targetP.voxel_down_sample(voxel_size=voxelSizek)

            # Normals are estimated with radius voxelSizek * 2.
            # This reduces computation and enlarges the convergence basin.
            # A coarse cloud is less detailed but more tolerant of initial misalignment.
            search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSizek * 2, max_nn=30)
            sourceCPk.estimate_normals(search_param=search_param)
            targetPk.estimate_normals(search_param=search_param)

            maximumCorrespondenceDistancek = voxelSizek

        # Extract ICP thresholds and criteria parameters
        inlierRMSEChangeThresholdk = scale_params["Inlier RMSE change threshold"]
        fitnessChangeThresholdk = scale_params["Fitness change threshold"]
        maximumIterationsk = scale_params["Maximum iterations"]
        inlierRMSEThresholdk = scale_params["Inlier RMSE threshold"]
        fitnessThresholdk = scale_params["Fitness threshold"]
        
        # Configure Open3D estimation method and convergence criteria separately to avoid long lines
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        convergence_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_rmse=inlierRMSEChangeThresholdk,
            relative_fitness=fitnessChangeThresholdk,
            max_iteration=maximumIterationsk
        )

        # Point-to-plane ICP minimizes the distance from each source point 
        # to the local tangent plane of its target correspondence.
        # Conceptually:
        # min Σ [(R p_i + t - q_i) · n_i]²
        # where:
        # - p_i is a source point.
        # - q_i is its target correspondence.
        # - n_i is the target normal.
        ICPRegistration = o3d.pipelines.registration.registration_icp(
            source=sourceCPk, 
            target=targetPk, 
            max_correspondence_distance=maximumCorrespondenceDistancek,
            init=tf_init, 
            estimation_method=estimation_method,
            criteria=convergence_criteria
        )
        
        # After registration, obtain the results:
        MfSCPktTPkcalculated = ICPRegistration.transformation
        currentInlierRMSE = ICPRegistration.inlier_rmse
        currentFitness = ICPRegistration.fitness
        
        # Fitness: fraction of source points with valid target correspondences.
        # Inlier RMSE: average correspondence error among accepted pairs.
        # Higher fitness is better. Lower RMSE is better.

        # Evaluate acceptance criteria
        criteria_met = (currentInlierRMSE <= inlierRMSEThresholdk) and (currentFitness >= fitnessThresholdk)

        # Recursive acceptance logic
        if scale_number == 0 and criteria_met:
            return MfSCPktTPkcalculated, 'y', currentFitness, currentInlierRMSE
            
        elif scale_number > 0 and criteria_met:
            # If a coarse scale passes, its transformation seeds the next finer scale:
            scale_number -= 1
            return self.register_pc(sourceCP, targetP, scale_number, MfSCPktTPkcalculated)
            
        else:
            # If any level fails (high RMSE or low fitness), the whole frame is rejected.
            return [], 'n', None, None

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
        '''
        TODO:
        As the scan grows, this can:

        Increase computation.
        Encourage correspondences to unrelated parts.
        Accumulate drift.
        Create bias toward dense older regions.

        Alternatives include:

        Registering against the previous frame.
        Registering against a local submap.
        Registering against the global map but spatially cropping around the predicted TF pose.
        Pose-graph optimization after initial pairwise registration.
        RGB-D odometry plus loop closure.
        '''

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

        # Compute timing statistics
        mean_recon = float(np.mean(self.reconstruction_times_ms)) if self.reconstruction_times_ms else 0.0
        std_recon = float(np.std(self.reconstruction_times_ms)) if self.reconstruction_times_ms else 0.0

        # Log timing data
        rospy.loginfo("================ TIMING METRICS ================")
        rospy.loginfo(f"  Reconstruction Time (ms): mean={mean_recon:.4f}, std={std_recon:.4f}")
        rospy.loginfo("================================================")

        # Write to timing_per_tree.csv
        csv_path = os.path.join(output_dir, "timing_per_tree.csv")
        file_exists = os.path.exists(csv_path)
        try:
            with open(csv_path, 'a') as f:
                if not file_exists:
                    f.write("tree_id,mean_reconstruction_time_ms,std_reconstruction_time_ms\n")
                f.write(f"{tree_id},{mean_recon:.4f},{std_recon:.4f}\n")
            rospy.loginfo(f"Appended timing data to {csv_path}")
        except Exception as e:
            rospy.logerr(f"Failed to write timing data to CSV: {e}")

if __name__ == '__main__':
    tree_point_cloud_publisher = TreePointCloudReconstructor()
    try:
        rospy.spin()
        tree_point_cloud_publisher.processing_rate.sleep()
    except rospy.ROSInterruptException:
        # cv2.destroyAllWindows()
        pass
