#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState


class ArmScanMotion(Node):
    def __init__(self):
        super().__init__('arm_scan_motion')

        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self._latest_joint_state = None

        self.create_subscription(
            JointState,
            '/platform/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Arm Scan Motion Node started')

    def joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def get_current_tcp_pose(self):
        if not self._fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/compute_fk service not available')
            return None

        start_time = time.time()
        while self._latest_joint_state is None:
            if time.time() - start_time > 10.0:
                self.get_logger().error('No /platform/joint_states received')
                return None
            rclpy.spin_once(self, timeout_sec=0.1)

        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['arm_0_tool0']
        req.robot_state.joint_state = self._latest_joint_state

        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error('FK service returned no response')
            return None

        if response.error_code.val != 1 or len(response.pose_stamped) == 0:
            self.get_logger().error(f'FK failed with code: {response.error_code.val}')
            return None

        pose = response.pose_stamped[0].pose

        self.get_logger().info(
            f'Current TCP pose: '
            f'x={pose.position.x:.3f}, '
            f'y={pose.position.y:.3f}, '
            f'z={pose.position.z:.3f}, '
            f'qx={pose.orientation.x:.3f}, '
            f'qy={pose.orientation.y:.3f}, '
            f'qz={pose.orientation.z:.3f}, '
            f'qw={pose.orientation.w:.3f}'
        )

        return pose

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        self.get_logger().info(f'Moving to: x={x:.3f}, y={y:.3f}, z={z:.3f}')

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server /move_action not available')
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm_0'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.03
        goal.request.max_acceleration_scaling_factor = 0.03

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = 'arm_0_tool0'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose.header
        orient_constraint.link_name = 'arm_0_tool0'
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.2
        orient_constraint.absolute_y_axis_tolerance = 0.2
        orient_constraint.absolute_z_axis_tolerance = 0.2
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to send goal')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error('No result returned')
            return False

        result = result_response.result
        if result.error_code.val == 1:
            self.get_logger().info('Motion succeeded')
            return True

        self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
        return False


def main():
    rclpy.init()
    node = ArmScanMotion()

    current_pose = node.get_current_tcp_pose()
    if current_pose is None:
        node.get_logger().error('Could not get current pose')
        node.destroy_node()
        rclpy.shutdown()
        return

    start_x = current_pose.position.x
    start_y = current_pose.position.y
    start_z = current_pose.position.z

    qx = current_pose.orientation.x
    qy = current_pose.orientation.y
    qz = current_pose.orientation.z
    qw = current_pose.orientation.w

    # 3 cm forward first, then back
    waypoints = [
        (start_x, start_y, start_z),
        (start_x + 0.03, start_y, start_z),
        (start_x, start_y, start_z),
    ]

    for i, (x, y, z) in enumerate(waypoints, start=1):
        node.get_logger().info(f'Waypoint {i}/{len(waypoints)}')
        success = node.move_to_pose(x, y, z, qx, qy, qz, qw)
        if not success:
            node.get_logger().error('Motion failed, stopping')
            break
        time.sleep(2.0)

    node.get_logger().info('Done')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
