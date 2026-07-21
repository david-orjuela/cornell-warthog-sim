#!/usr/bin/env python3
"""cuRobo raster scan executor for a UR5e under ROS 2 Humble.

The node plans absolute scan poses from the initial end-effector pose, executes
through a ROS 2 FollowJointTrajectory action, and requests exactly one RGB-D
capture after each settled pose. Motion is disabled by default.
"""

from __future__ import annotations

import math
import time
import traceback
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import torch
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Bool, UInt32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)


DEFAULT_UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class TrajectoryController(Node):
    """Plan and execute a capture-synchronized raster scan."""

    def __init__(self) -> None:
        super().__init__("trajectory_planner")
        self._declare_parameters()

        self.execute_motion = bool(self._p("execute_motion"))
        self.joint_state_topic = self._p("joint_state_topic")
        self.trajectory_action = self._p("trajectory_action")
        self.control_joint_names = list(self._p("joint_names"))
        self.joint_positions: Optional[List[float]] = None
        self.received_joint_names: List[str] = []
        self.capture_done_sequence = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_sub = self.create_subscription(
            JointStateMsg,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.capture_done_sub = self.create_subscription(
            UInt32,
            self._p("capture_done_topic"),
            self.capture_done_callback,
            qos,
        )
        self.capture_pub = self.create_publisher(
            Bool, self._p("capture_topic"), qos
        )
        self.finished_pub = self.create_publisher(
            Bool, self._p("scan_finished_topic"), qos
        )
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, self.trajectory_action
        )

        self.motion_gen: Optional[MotionGen] = None
        self.curobo_joint_names = list(DEFAULT_UR5E_JOINTS)
        self.cuda_device = torch.device("cpu")
        if self.execute_motion:
            self.motion_gen, self.curobo_joint_names, self.cuda_device = self._init_curobo()
            if len(self.curobo_joint_names) != len(self.control_joint_names):
                raise RuntimeError(
                    "cuRobo and controller joint counts differ: "
                    f"{self.curobo_joint_names} vs {self.control_joint_names}"
                )
        else:
            self.get_logger().warning(
                "Motion is DISABLED. Set execute_motion:=true only after checking "
                "the action name, joint names, TF, scan dimensions, and collision world."
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "execute_motion": False,
            "joint_state_topic": "/platform/joint_states",
            "trajectory_action": "/joint_trajectory_controller/follow_joint_trajectory",
            "joint_names": DEFAULT_UR5E_JOINTS,
            "capture_topic": "/capture_alert",
            "capture_done_topic": "/capture_done",
            "scan_finished_topic": "/scan_finished",
            "robot_file": "ur5e.yml",
            "world_file": "collision_table.yml",
            "cuda_device": 0,
            "interpolation_dt_sec": 0.02,
            "trajectory_time_scale": 1.75,
            "action_server_timeout_sec": 15.0,
            "trajectory_timeout_sec": 90.0,
            "joint_state_timeout_sec": 15.0,
            "capture_subscriber_timeout_sec": 15.0,
            "capture_timeout_sec": 10.0,
            "settle_time_sec": 1.0,
            "scan_width_m": 0.24,
            "scan_height_m": 0.32,
            "scan_columns": 3,
            "scan_rows": 4,
            "scan_axis_u_ee": [1.0, 0.0, 0.0],
            "scan_axis_v_ee": [0.0, 1.0, 0.0],
            "capture_at_start": True,
            "return_to_start": True,
            "skip_origin_grid_point": True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _init_curobo(self) -> Tuple[MotionGen, List[str], torch.device]:
        if not torch.cuda.is_available():
            raise RuntimeError(
                "cuRobo requires a working CUDA device, but torch.cuda.is_available() is false."
            )
        cuda_index = int(self._p("cuda_device"))
        torch.cuda.set_device(cuda_index)
        device = torch.device(f"cuda:{cuda_index}")
        tensor_args = TensorDeviceType(device=device)

        robot_file = self._p("robot_file")
        world_file = self._p("world_file")
        config = MotionGenConfig.load_from_robot_config(
            robot_file,
            world_file,
            tensor_args,
            interpolation_dt=float(self._p("interpolation_dt_sec")),
        )
        motion_gen = MotionGen(config)
        configured_names = list(motion_gen.joint_names)
        if not configured_names:
            raise RuntimeError("cuRobo returned an empty joint-name list")

        # Put the controller/action joint names in exactly the same logical order
        # as cuRobo's trajectory output. Prefixes are allowed, reordering is not
        # guessed from position indices.
        requested_names = list(self.control_joint_names)
        ordered_controller_names: List[str] = []
        for curobo_name in configured_names:
            matches = [
                candidate
                for candidate in requested_names
                if candidate == curobo_name or candidate.endswith(curobo_name)
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    "joint_names must contain exactly one entry matching each "
                    f"cuRobo joint. Could not uniquely match '{curobo_name}' in "
                    f"{requested_names}."
                )
            ordered_controller_names.append(matches[0])
        self.control_joint_names = ordered_controller_names

        self.get_logger().info("Warming up cuRobo motion generation...")
        motion_gen.warmup(enable_graph=False)
        return motion_gen, configured_names, device

    def joint_state_callback(self, msg: JointStateMsg) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        resolved_names: List[str] = []
        values: List[float] = []

        for logical_name, requested_name in zip(
            self.curobo_joint_names, self.control_joint_names
        ):
            if requested_name in positions_by_name:
                resolved = requested_name
            elif logical_name in positions_by_name:
                resolved = logical_name
            else:
                # Supports common prefixes such as "ur_" while rejecting
                # ambiguous suffix matches. Values stay in cuRobo joint order.
                matches = [
                    name for name in msg.name if name.endswith(logical_name)
                ]
                if len(matches) != 1:
                    self.received_joint_names = list(msg.name)
                    return
                resolved = matches[0]
            resolved_names.append(resolved)
            values.append(float(positions_by_name[resolved]))

        if resolved_names != self.control_joint_names:
            self.control_joint_names = resolved_names
            self.get_logger().info(
                f"Resolved controller joint names from JointState: {resolved_names}"
            )
        self.joint_positions = values
        self.received_joint_names = list(msg.name)

    def capture_done_callback(self, msg: UInt32) -> None:
        self.capture_done_sequence = max(self.capture_done_sequence, int(msg.data))

    def wait_for_preconditions(self) -> bool:
        deadline = time.monotonic() + float(self._p("joint_state_timeout_sec"))
        while rclpy.ok() and self.joint_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_positions is None:
            self.get_logger().error(
                f"No UR5e joint state found on {self.joint_state_topic}. "
                f"Last message joint names: {self.received_joint_names}"
            )
            return False

        if not self.trajectory_client.wait_for_server(
            timeout_sec=float(self._p("action_server_timeout_sec"))
        ):
            self.get_logger().error(
                f"No FollowJointTrajectory action server at {self.trajectory_action}."
            )
            return False

        deadline = time.monotonic() + float(
            self._p("capture_subscriber_timeout_sec")
        )
        while (
            rclpy.ok()
            and self.capture_pub.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.capture_pub.get_subscription_count() == 0:
            self.get_logger().error(
                f"No reconstruction subscriber on {self._p('capture_topic')}."
            )
            return False
        return True

    def _current_curobo_state(self) -> JointState:
        if self.joint_positions is None:
            raise RuntimeError("Joint state is not initialized")
        tensor = torch.as_tensor(
            [self.joint_positions], dtype=torch.float32, device=self.cuda_device
        )
        return JointState.from_position(tensor, joint_names=self.curobo_joint_names)

    def _current_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.motion_gen is None:
            raise RuntimeError("cuRobo is not initialized")
        state = self._current_curobo_state()
        kinematics = self.motion_gen.compute_kinematics(state)
        position_tensor = getattr(
            kinematics, "ee_position", getattr(kinematics, "ee_pos_seq", None)
        )
        quaternion_tensor = getattr(
            kinematics, "ee_quaternion", getattr(kinematics, "ee_quat_seq", None)
        )
        if position_tensor is None or quaternion_tensor is None:
            raise RuntimeError("Unsupported cuRobo kinematics result fields")
        position = position_tensor.detach().reshape(-1, 3)[0].cpu().numpy()
        quaternion_wxyz = quaternion_tensor.detach().reshape(-1, 4)[0].cpu().numpy()
        quaternion_wxyz = quaternion_wxyz / np.linalg.norm(quaternion_wxyz)
        return position.astype(np.float64), quaternion_wxyz.astype(np.float64)

    @staticmethod
    def _rotation_matrix_wxyz(quaternion: Sequence[float]) -> np.ndarray:
        w, x, y, z = [float(value) for value in quaternion]
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1.0e-12:
            raise ValueError("Quaternion has zero norm")
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _make_goal_pose(
        self,
        origin_position: np.ndarray,
        origin_quaternion_wxyz: np.ndarray,
        u_offset: float,
        v_offset: float,
    ) -> Pose:
        u_axis = np.asarray(self._p("scan_axis_u_ee"), dtype=np.float64)
        v_axis = np.asarray(self._p("scan_axis_v_ee"), dtype=np.float64)
        if u_axis.shape != (3,) or v_axis.shape != (3,):
            raise ValueError("scan_axis_u_ee and scan_axis_v_ee must each have 3 values")
        u_axis /= np.linalg.norm(u_axis)
        v_axis -= u_axis * float(np.dot(u_axis, v_axis))
        if np.linalg.norm(v_axis) < 1.0e-9:
            raise ValueError("Scan axes must not be parallel")
        v_axis /= np.linalg.norm(v_axis)

        local_offset = u_axis * u_offset + v_axis * v_offset
        base_offset = self._rotation_matrix_wxyz(origin_quaternion_wxyz) @ local_offset
        final_position = origin_position + base_offset
        position_tensor = torch.as_tensor(
            final_position, dtype=torch.float32, device=self.cuda_device
        )
        quaternion_tensor = torch.as_tensor(
            origin_quaternion_wxyz, dtype=torch.float32, device=self.cuda_device
        )
        return Pose(position_tensor, quaternion_tensor)

    def _generate_raster_offsets(self) -> List[Tuple[float, float]]:
        columns = int(self._p("scan_columns"))
        rows = int(self._p("scan_rows"))
        if columns < 1 or rows < 1:
            raise ValueError("scan_columns and scan_rows must be positive")
        width = float(self._p("scan_width_m"))
        height = float(self._p("scan_height_m"))
        u_values = np.linspace(-width / 2.0, width / 2.0, columns)
        v_values = np.linspace(-height / 2.0, height / 2.0, rows)

        offsets: List[Tuple[float, float]] = []
        for row_index, v in enumerate(v_values):
            row_u = u_values if row_index % 2 == 0 else u_values[::-1]
            for u in row_u:
                if bool(self._p("skip_origin_grid_point")) and math.hypot(float(u), float(v)) < 1.0e-9:
                    continue
                offsets.append((float(u), float(v)))
        return offsets

    def plan_to_pose(self, goal_pose: Pose) -> Optional[Tuple[JointTrajectory, float]]:
        if self.motion_gen is None:
            raise RuntimeError("cuRobo is not initialized")
        start_state = self._current_curobo_state()
        result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(enable_graph=False, max_attempts=5),
        )
        success = result.success
        if hasattr(success, "item"):
            success = bool(success.item())
        if not success:
            self.get_logger().error("cuRobo failed to find a collision-free trajectory.")
            return None

        plan = result.optimized_plan
        positions = plan.position.detach()
        if positions.ndim == 3:
            positions = positions.squeeze(0)
        dt = float(result.optimized_dt.item())
        time_scale = float(self._p("trajectory_time_scale"))
        if time_scale <= 0.0:
            raise ValueError("trajectory_time_scale must be positive")

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.control_joint_names)
        for index, joint_position in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = joint_position.cpu().tolist()
            point.time_from_start = self._duration_msg((index + 1) * dt * time_scale)
            trajectory.points.append(point)
        return trajectory, len(positions) * dt * time_scale

    @staticmethod
    def _duration_msg(seconds: float) -> DurationMsg:
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return DurationMsg(sec=sec, nanosec=nanosec)

    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=float(self._p("action_server_timeout_sec")),
        )
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Timed out while sending trajectory goal.")
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory controller rejected the goal.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=float(self._p("trajectory_timeout_sec")),
        )
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Trajectory execution timed out; cancelling goal.")
            goal_handle.cancel_goal_async()
            return False

        wrapped_result = result_future.result()
        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error(
                "Trajectory failed: "
                f"status={wrapped_result.status}, error_code={result.error_code}, "
                f"error_string='{result.error_string}'"
            )
            return False
        return True

    def request_capture(self, label: str) -> bool:
        before = self.capture_done_sequence
        msg = Bool()
        msg.data = True
        self.capture_pub.publish(msg)
        self.get_logger().info(f"Requested RGB-D capture at {label}.")

        deadline = time.monotonic() + float(self._p("capture_timeout_sec"))
        while (
            rclpy.ok()
            and self.capture_done_sequence <= before
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.capture_done_sequence <= before:
            self.get_logger().error(
                "Timed out waiting for reconstruction acknowledgement. Check RGB-D sync and TF."
            )
            return False
        return True

    def move_and_capture(self, goal_pose: Pose, label: str) -> bool:
        planned = self.plan_to_pose(goal_pose)
        if planned is None:
            return False
        trajectory, nominal_duration = planned
        self.get_logger().info(
            f"Executing {label}: {len(trajectory.points)} points, "
            f"nominal {nominal_duration:.2f} s"
        )
        if not self.execute_trajectory(trajectory):
            return False
        self._spin_sleep(float(self._p("settle_time_sec")))
        return self.request_capture(label)

    def _spin_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic()))
            )

    def run_scan(self) -> bool:
        if not self.execute_motion:
            self.get_logger().warning("Exiting without motion because execute_motion is false.")
            return False
        if not self.wait_for_preconditions():
            return False

        origin_position, origin_quaternion = self._current_ee_pose()
        self.get_logger().info(
            "Scan origin from current UR5e pose: position="
            f"{origin_position.tolist()}, quaternion_wxyz={origin_quaternion.tolist()}"
        )

        if bool(self._p("capture_at_start")):
            self._spin_sleep(float(self._p("settle_time_sec")))
            if not self.request_capture("scan origin"):
                return False

        offsets = self._generate_raster_offsets()
        for index, (u_offset, v_offset) in enumerate(offsets, start=1):
            goal_pose = self._make_goal_pose(
                origin_position, origin_quaternion, u_offset, v_offset
            )
            label = f"raster {index}/{len(offsets)} (u={u_offset:.3f}, v={v_offset:.3f})"
            if not self.move_and_capture(goal_pose, label):
                self.get_logger().error("Scan aborted after planning, execution, or capture failure.")
                return False

        if bool(self._p("return_to_start")):
            start_pose = self._make_goal_pose(
                origin_position, origin_quaternion, 0.0, 0.0
            )
            planned = self.plan_to_pose(start_pose)
            if planned is None or not self.execute_trajectory(planned[0]):
                self.get_logger().error("Scan finished, but return-to-start failed.")
                return False

        finished = Bool()
        finished.data = True
        self.finished_pub.publish(finished)
        self._spin_sleep(0.5)
        self.get_logger().info("Raster scan completed successfully.")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TrajectoryController] = None
    try:
        node = TrajectoryController()
        node.run_scan()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Trajectory planner failed: {exc}\n{traceback.format_exc()}"
            )
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
