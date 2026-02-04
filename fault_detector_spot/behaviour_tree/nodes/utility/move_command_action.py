#!/usr/bin/env python3
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME

import rclpy
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from py_trees.common import Status, Access
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveCommandAction(ActionClientBehaviour):
    """
    Intermediate base class for actions that execute a MoveCommand.

    It overrides update() to ensure all necessary TF frames are available
    BEFORE delegating to the ActionClientBehaviour lifecycle.
    """

    def __init__(self, name: str):
        super().__init__(name)
        self.tf_listener: TFListenerWrapper = None
        # Ensure blackboard access is set up
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # Corrected indentation here
        if not self.tf_listener and hasattr(self, 'node'):
            self.tf_listener = TFListenerWrapper(self.node)

    def update(self) -> Status:
        """
        Check TF availability before proceeding with the action lifecycle.
        """
        # 1. Retrieve Command
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No command on blackboard"
            return Status.FAILURE

        cmd = self.blackboard.last_command

        # If it's not a MoveCommand, we can't check frames easily.
        if not isinstance(cmd, MoveCommand):
            return super().update()

        # 2. Check TF Readiness
        # We assume subclasses rely on:
        #   A) Offset Frame -> Target Frame (for command calculation)
        #   B) Target Frame -> Body Frame (for Spot API execution)

        source_frame = cmd.offset.header.frame_id
        target_frame = cmd.target_frame
        final_frame = GRAV_ALIGNED_BODY_FRAME_NAME

        try:
            # Check 1: Offset -> Target (Command Internal Logic)
            if source_frame != target_frame:
                if not self.tf_listener._tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
                    self.feedback_message = f"Waiting for TF: {source_frame} -> {target_frame}"
                    return Status.RUNNING

            # Check 2: Target -> Body (Execution Logic)
            # Most actions convert the final goal into Body frame for Spot.
            if target_frame != final_frame:
                if not self.tf_listener._tf_buffer.can_transform(final_frame, target_frame, rclpy.time.Time()):
                    self.feedback_message = f"Waiting for TF: {target_frame} -> {final_frame}"
                    return Status.RUNNING
        except Exception as e:
            # Handle potential TF errors gracefully (e.g. listener not ready)
            self.logger.warn(f"TF Check Failed: {e}")
            return Status.FAILURE

        # If TF is ready (or we are already running), proceed.
        return super().update()