#!/usr/bin/env python3
"""
A behaviour that moves the Spot arm by a relative XYZ offset in the body frame.
"""
import py_trees
from py_trees.common import Access, Status
from geometry_msgs.msg import PoseStamped
import time
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
from synchros2.tf_listener_wrapper import TFListenerWrapper
import synchros2.scope as ros_scope
import rclpy
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME

from fault_detector_spot.behaviour_tree.manipulator_move_command import ManipulatorMoveCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour


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

        if not self.tf_listener._tf_buffer.can_transform(
                GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME, rclpy.time.Time()):
            self.feedback_message = "Waiting for TF flat_body→hand"
            return Status.RUNNING

        arm_cmd = self.get_goal_cmd_with_offset()
        goal = RobotCommand.Goal()
        convert(arm_cmd, goal.command)
        return goal

    def get_goal_cmd_with_offset(self):
        cmd_obj = self.blackboard.last_command
        if not isinstance(cmd_obj, ManipulatorMoveCommand):
            raise RuntimeError(f"Expected ManipulatorMoveCommand, got {type(cmd_obj).__name__}")

        tf_msg = self.tf_listener.lookup_a_tform_b(GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME, timeout_sec=2)
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation

        dx, dy, dz = (cmd_obj.goal_pose.pose.position.x,
                      cmd_obj.goal_pose.pose.position.y,
                      cmd_obj.goal_pose.pose.position.z)

        tx, ty, tz = t.x + dx, t.y + dy, t.z + dz
        arm_cmd = RobotCommandBuilder.arm_pose_command(
            tx, ty, tz,
            r.w, r.x, r.y, r.z,
            namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
            self.duration
        )
        return arm_cmd
