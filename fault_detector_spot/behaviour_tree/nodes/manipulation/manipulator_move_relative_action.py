#!/usr/bin/env python3
"""
A behaviour that moves the Spot arm by a relative XYZ offset in the body frame.
"""
from bosdyn.client.frame_helpers import HAND_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import rclpy
import tf2_geometry_msgs
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.manipulator_move_command import ManipulatorMoveCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access, Status
from rclpy.duration import Duration
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from tf_transformations import quaternion_multiply


class ManipulatorMoveRelativeAction(ActionClientBehaviour):
    """
    Moves the arm by a relative offset stored in last_command.goal_pose.pose.position
    (interpreted in the body frame).  The orientation remains unchanged.
    """

    def __init__(self,
                 name: str = "ManipulatorMoveRelativeAction",
                 robot_name: str = "",
                 duration: float = 2):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration
        self.tf_listener: TFListenerWrapper = None
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def _init_client(self) -> bool:
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
            self.tf_listener = TFListenerWrapper(self.node)
            self.initialized = True
            return True
        except Exception as e:
            self.logger.error(f"[{self.name}] init failed: {e}")
            return False

    def _build_goal(self) -> RobotCommand.Goal:
        # Start the timeout timer when first entering RUNNING state
        if not hasattr(self, "_tf_wait_start_time"):
            self._tf_wait_start_time = self.node.get_clock().now()

        if not self.tf_listener._tf_buffer.can_transform(
                self.blackboard.last_command.goal_pose.header.frame_id,
                HAND_FRAME_NAME,
                rclpy.time.Time()):
            elapsed = self.node.get_clock().now() - self._tf_wait_start_time
            if elapsed > Duration(seconds=5.0):
                self.feedback_message = "TF →hand timeout (5s)"
                del self._tf_wait_start_time  # reset for next run
                return Status.FAILURE
            self.feedback_message = "Waiting for TF →hand"
            return Status.RUNNING

        # Success — transform available, clear timer
        if hasattr(self, "_tf_wait_start_time"):
            del self._tf_wait_start_time

        arm_cmd = self.get_goal_cmd_with_offset()
        goal = RobotCommand.Goal()
        convert(arm_cmd, goal.command)
        return goal

    def get_goal_cmd_with_offset(self):
        pose_with_offset = self._hand_position_with_offset_in_offset_frame()
        tf_to_body = self.tf_listener.lookup_a_tform_b(GRAV_ALIGNED_BODY_FRAME_NAME, pose_with_offset.header.frame_id,
                                                       timeout_sec=2)
        pose_in_body = tf2_geometry_msgs.do_transform_pose_stamped(pose_with_offset, tf_to_body)

        arm_cmd = RobotCommandBuilder.arm_pose_command(
            pose_in_body.pose.position.x, pose_in_body.pose.position.y, pose_in_body.pose.position.z,
            pose_in_body.pose.orientation.w, pose_in_body.pose.orientation.x, pose_in_body.pose.orientation.y,
            pose_in_body.pose.orientation.z,
            namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
            self.duration
        )
        return arm_cmd

    def _hand_position_with_offset_in_offset_frame(self):
        cmd_obj = self.blackboard.last_command
        if not isinstance(cmd_obj, ManipulatorMoveCommand):
            raise RuntimeError(f"Expected ManipulatorMoveCommand, got {type(cmd_obj).__name__}")
        tf_msg = self.tf_listener.lookup_a_tform_b(cmd_obj.goal_pose.header.frame_id, HAND_FRAME_NAME, timeout_sec=2)
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation

        dx, dy, dz = (cmd_obj.goal_pose.pose.position.x,
                      cmd_obj.goal_pose.pose.position.y,
                      cmd_obj.goal_pose.pose.position.z)
        tx, ty, tz = t.x + dx, t.y + dy, t.z + dz
        dr = cmd_obj.goal_pose.pose.orientation
        offset_quat = [dr.x, dr.y, dr.z, dr.w]
        r_quat = [r.x, r.y, r.z, r.w]
        rtx, rty, rtz, rtw = quaternion_multiply(r_quat, offset_quat)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = cmd_obj.goal_pose.header.frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()

        pose_stamped.pose.position.x = tx
        pose_stamped.pose.position.y = ty
        pose_stamped.pose.position.z = tz

        pose_stamped.pose.orientation.x = rtx
        pose_stamped.pose.orientation.y = rty
        pose_stamped.pose.orientation.z = rtz
        pose_stamped.pose.orientation.w = rtw

        return pose_stamped