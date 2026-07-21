#!/usr/bin/python3
import copy
import sys

import actionlib
import numpy as np
import rospy
import torch
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util_file import (get_robot_configs_path, join_path, load_yaml, )
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from sensor_msgs.msg import JointState as JointState_msg
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryController:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node("trajectory_planner")

        timeout = rospy.Duration(5)

        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                            "wrist_2_joint", "wrist_3_joint", ]

        # All of those controllers can be used to execute joint-based trajectories.
        # The scaled versions should be preferred over the non-scaled versions.
        self.joint_trajectory_controllers = ["scaled_pos_joint_traj_controller", "scaled_vel_joint_traj_controller",
                                             "pos_joint_traj_controller", "vel_joint_traj_controller",
                                             "forward_joint_traj_controller", ]

        # All of those controllers can be used to execute Cartesian trajectories.
        # The scaled versions should be preferred over the non-scaled versions.
        self.cartesian_trajectory_controllers = ["pose_based_cartesian_traj_controller",
                                                 "joint_based_cartesian_traj_controller",
                                                 "forward_cartesian_traj_controller", ]

        self.conflicting_controllers = ["joint_group_vel_controller", "twist_controller"]

        # Services, communicate with ROS' controller_manager
        self.switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)
        
        self.target_controller = self.joint_trajectory_controllers[0] # scaled_pos_joint_traj_controller
        self.switch_controller(self.target_controller)

        try:
            self.trajectory_client.wait_for_server()
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.target_controller), FollowJointTrajectoryAction, )

        # curobo
        tensor_args = TensorDeviceType()
        robot_file = "ur5e.yml"
        world_file = "collision_table.yml" # Plans paths considering obstacles encoded in YAML
        self.manipulation_error = 0

        motion_gen_config = MotionGenConfig.load_from_robot_config(robot_file, world_file, tensor_args,
                                                                   interpolation_dt=0.01)
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup(enable_graph=False) # initializes GPU planning structures

        robot_cfg = load_yaml(join_path(get_robot_configs_path(), robot_file))["robot_cfg"]
        robot_cfg = RobotConfig.from_dict(robot_cfg, tensor_args)

        self.joint_positions = None
        self.joint_names_received = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint",
                                     "wrist_2_joint", "wrist_3_joint"]

        # ur5e joint states
        rospy.Subscriber("/joint_states", JointState_msg, self.joint_state_callback) # planner needs the real, current robot state
        self.received_joint_states = False

        # capture alert publisher
        self.alert_publisher = rospy.Publisher('capture_alert', Bool, queue_size=10)

        # The bool above is used to trigger the point cloud capture (after robot motion)
        # Not used in scanning_mode()

    # Stores the latest joint positions
    def joint_state_callback(self, data): 
        self.joint_positions = list(data.position)
        self.joint_names_received = list(data.name)
        self.received_joint_states = True

    def send_goal(self, joint_trajectory_goal):
        """send trajectory using selected action server"""
        rospy.loginfo("Executing trajectory using the {}".format(self.target_controller))
        self.trajectory_client.send_goal(joint_trajectory_goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
        self.manipulation_error = result.error_code

    # Motion planning function
    def curobo_motion_execute(self, relative_translation, relative_rotation, pure_rotation=False):
        """curobo motion planner"""
        joint_positions = copy.copy(self.joint_positions)
        joint_names = self.joint_names

        # Reorder positions using name, not just positions (safer)
        joint_positions[0], joint_positions[2] = joint_positions[2], joint_positions[0]
        start_state = JointState.from_position(
            torch.tensor(np.array(joint_positions, dtype=np.float32), device='cuda:0').view(1, -1))

        start_state_kin = self.motion_gen.compute_kinematics(start_state)
        translation = start_state_kin.ee_pos_seq.squeeze()
        rotation = start_state_kin.ee_quat_seq.squeeze()
        start_pose = Pose(translation, rotation)
        relative_translation = torch.tensor(np.array(relative_translation, np.float32), device='cuda:0')
        relative_rotation = torch.tensor(np.array(relative_rotation, np.float32), device='cuda:0')
        start_pose_copy = start_pose.clone()
        relative_pose = Pose(relative_translation, relative_rotation)
        final_pose = start_pose.multiply(relative_pose)
        if pure_rotation:
            final_pose.position = start_pose_copy.position
        result = self.motion_gen.plan_single(start_state, final_pose, MotionGenPlanConfig())
        print('-----------------')
        if result.success:
            rospy.loginfo("Trajectory path planning success")
            plan = result.optimized_plan
            dt = result.optimized_dt.item()
            # dt = dt * 2
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = joint_names
            time_counter = rospy.Duration(0)
            for position, velocity, acceleration in zip(plan.position, plan.velocity, plan.acceleration):
                point = JointTrajectoryPoint()
                point.positions = position
                point.velocities = velocity
                point.accelerations = acceleration
                time_counter += rospy.Duration(dt)
                point.time_from_start = time_counter
                goal.trajectory.points.append(point)
            self.send_goal(goal)
        else:
            rospy.loginfo("Trajectory path planning failed")

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
                self.joint_trajectory_controllers + self.cartesian_trajectory_controllers + self.conflicting_controllers)

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def go_home(self):
        """moves robot to home position"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [-0.06, -1.25, 2.3, -3.76, -1.57, -3.14]
        # point.positions = [0.11537981033325195, -2.58834232906484, 2.544412914906637, -3.029740949670309, -1.7944453398333948, -3.141567055379049]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)
        self.send_goal(goal)  # self.alert_publisher.publish(True)  # rospy.sleep(3)

    def move_tool(self, translation, rotation, pure_rotation=False):
        if self.manipulation_error == 0:
            self.curobo_motion_execute(translation, rotation, pure_rotation)
        else:
            print("Manipulation failed!")

    def scanning_mode(self):
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0, 0, 0], [1, 0.2, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0.3, 0, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0, 0, 0], [1, -0.2, 0, 0])
        # self.move_tool([0, 0.15, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0, 0.2, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0.2, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([-0.3, 0, 0], [1, 0, 0, 0])
        # self.move_tool([-0.3, 0, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0, -0.2, 0], [1, 0, 0, 0])
        # self.move_tool([0, -0.1, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0, 0, 0], [1, 0.2, 0, 0])
        # self.move_tool([0, -0.1, 0], [1, 0, 0, 0])
        # self.move_tool([0, -0.15, 0], [1, 0, 0, 0])
        # self.alert_publisher.publish(True)
        # rospy.sleep(2)
        # self.move_tool([0.3, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, -0.2, 0, 0])
        # self.alert_publisher.publish(True)

        # self.move_tool([0, 0, 0], [1, 0.2, 0, 0])
        self.move_tool([0, 0, 0], [1, -0.2, 0, 0])
        self.move_tool([0.2, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        self.move_tool([-0.1, 0, 0], [1, 0, 0, 0])
        self.move_tool([-0.1, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        self.move_tool([0.1, 0, 0], [1, 0, 0, 0])
        self.move_tool([0, 0.1, 0], [1, 0, 0, 0])
        self.move_tool([0.2, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])
        self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        self.move_tool([0.2, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, -0.1, 0, 0])
        self.move_tool([0, 0.2, 0], [1, 0, 0, 0])
        self.move_tool([0, 0.1, 0], [1, 0, 0, 0])
        self.move_tool([0.1, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        self.move_tool([-0.1, 0, 0], [1, 0, 0, 0])
        self.move_tool([-0.1, 0, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])
        # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])
        self.move_tool([0.1, 0, 0], [1, 0, 0, 0])
        self.move_tool([0, -0.2, 0], [1, 0, 0, 0])
        # self.move_tool([0, -0.1, 0], [1, 0, 0, 0])
        # self.move_tool([0, -0.1, 0], [1, 0, 0, 0])
        # self.move_tool([0, 0, 0], [1, 0.2, 0, 0])

        # self.move_tool([0, 0, 0], [1, 0.2, 0, 0])  # self.move_tool([0.2, 0.1, 0], [1, 0, 0, 0])  # self.move_tool([0, 0, 0], [1, 0, 0.1, 0])  # self.move_tool([0, 0, 0], [1, 0, -0.1, 0])  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])

        # self.move_tool([0.2, 0, 0], [1, 0, 0, 0])  # self.move_tool([0.2, 0, 0], [1, 0, 0, 0])  # self.alert_publisher.publish(True)  # rospy.sleep(2)  #  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.alert_publisher.publish(True)  # rospy.sleep(2)  #  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.move_tool([-0.2, 0, 0], [1, 0, 0, 0])  # self.alert_publisher.publish(True)  # rospy.sleep(2)  #  # self.move_tool([0, 0.2, 0], [1, 0, 0, 0])  # self.alert_publisher.publish(True)  # rospy.sleep(2)  #  # self.move_tool([0, 0.2, 0], [1, 0, 0, 0])  # self.alert_publisher.publish(True)  # rospy.sleep(2)


if __name__ == "__main__":
    trajectory_controller = TrajectoryController()
    trajectory_controller.go_home()

    while True:
        if trajectory_controller.received_joint_states:
            trajectory_controller.scanning_mode()
            trajectory_controller.go_home()
            break
