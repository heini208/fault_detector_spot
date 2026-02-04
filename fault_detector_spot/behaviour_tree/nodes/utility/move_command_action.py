#!/usr/bin/env python3
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME

import rclpy
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
from fault_detector_spot.behaviour_tree.commands.move_to_tag_command import MoveToTagCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from py_trees.common import Status, Access
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with


class MoveCommandAction(ActionClientBehaviour):
    """
    Intermediate base class for actions that execute a MoveCommand.

    It overrides update() to ensure all necessary TF frames are available
    BEFORE delegating to the ActionClientBehaviour lifecycle.
    """

    def __init__(self, name: str, robot_name: str = "",
                 ):
        super().__init__(name)
        self.tf_listener: TFListenerWrapper = None
        # Ensure blackboard access is set up
        self.blackboard = self.attach_blackboard_client()
        self.robot_name = robot_name
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def _init_client(self) -> bool:
        action_ns = namespace_with(self.robot_name, "robot_command")
        self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
        self.tf_listener = TFListenerWrapper(self.node)
        self.initialized = True
        return True

    def _phase_send_goal(self) -> Status | None:
        """
        Inject TF safety checks before delegating to the standard send_goal logic.
        """
        # Only check if we haven't sent the goal yet
        if self.send_goal_future is None:
            # 1. Retrieve Command
            if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
                self.feedback_message = "No command on blackboard"
                return Status.FAILURE

            cmd = self.blackboard.last_command

            # If valid MoveCommand, perform TF checks
            if isinstance(cmd, MoveCommand):
                source_frame = cmd.offset.header.frame_id
                target_frame = cmd.target_frame
                final_frame = GRAV_ALIGNED_BODY_FRAME_NAME

                # Check 1: Offset -> Target
                if source_frame != target_frame:
                    if not self.tf_listener._tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
                        self.feedback_message = f"Waiting for TF: {source_frame} -> {target_frame}"
                        return Status.RUNNING
                # Check 2: Target -> Body
                if target_frame != final_frame:
                    if not self.tf_listener._tf_buffer.can_transform(final_frame, target_frame, rclpy.time.Time()):
                        self.feedback_message = f"Waiting for TF: {target_frame} -> {final_frame}"
                        return Status.RUNNING
            if isinstance(cmd, MoveToTagCommand):
                tag_frame = cmd.tag_pose.header.frame_id
                if tag_frame != target_frame:
                    if not self.tf_listener._tf_buffer.can_transform(target_frame, tag_frame, rclpy.time.Time()):
                        self.feedback_message = f"Waiting for TF: {tag_frame} -> {target_frame}"
                        return Status.RUNNING

        # TF is ready (or goal already sent), proceed with base logic
        return super()._phase_send_goal()