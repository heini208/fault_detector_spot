#!/usr/bin/env python3
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import tf2_geometry_msgs
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
# CHANGED: Import intermediate class
from fault_detector_spot.behaviour_tree.nodes.utility.move_command_action import MoveCommandAction
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with


class ManipulatorMoveRelativeAction(MoveCommandAction):
    """
    Moves the arm based on a MoveCommand. Inherits TF safety checks from MoveCommandAction.
    """

    def __init__(self,
                 name: str = "ManipulatorMoveRelativeAction",
                 duration: float = 2):
        super().__init__(name)
        self.duration = duration


    def _build_goal(self) -> RobotCommand.Goal:
        cmd = self.blackboard.last_command
        if not isinstance(cmd, MoveCommand):
            raise RuntimeError(f"Expected MoveCommand, got {type(cmd)}")

        arm_cmd = self.get_goal_cmd()
        goal = RobotCommand.Goal()
        convert(arm_cmd, goal.command)
        return goal

    def get_goal_cmd(self):
        cmd_obj = self.blackboard.last_command
        # TF is guaranteed by update() in base class
        pose_in_target = cmd_obj.compute_goal_pose(self.tf_listener)

        tf_to_body = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            pose_in_target.header.frame_id,
            timeout_sec=2
        )
        pose_in_body = tf2_geometry_msgs.do_transform_pose_stamped(pose_in_target, tf_to_body)

        arm_cmd = RobotCommandBuilder.arm_pose_command(
            pose_in_body.pose.position.x, pose_in_body.pose.position.y, pose_in_body.pose.position.z,
            pose_in_body.pose.orientation.w, pose_in_body.pose.orientation.x, pose_in_body.pose.orientation.y,
            pose_in_body.pose.orientation.z,
            namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME),
            self.duration
        )
        return arm_cmd