#!/usr/bin/env python3
from math import sin, cos, pi

from fault_detector_spot.behaviour_tree.command_ids import CommandID
from fault_detector_spot.behaviour_tree.manipulator_move_command import ManipulatorMoveCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from builtin_interfaces.msg import Time


_YAW_90_SIN = sin(pi / 4)  # sin(45°)
_YAW_90_COS = cos(pi / 4)  # cos(45°)
_PITCH_45_SIN = sin(pi / 8)  # sin(22.5°)
_PITCH_45_COS = cos(pi / 8)  # cos(22.5°)

class ManipulatorTagCommand(ManipulatorMoveCommand):
    """
    Encapsulates a tag-based goal for the manipulator:
    - id: numeric tag identifier
    - goal_pose: PoseStamped from fiducial detection
    - offset: PoseStamped representing positional offsets
    - orientation_mode: one of 'tag_orientation', 'look_straight',
      'left', 'right', 'up', 'down'

    Provides get_offset_pose() to combine them.
    """

    def __init__(
        self,
        command_id: str,
        stamp : Time,
        goal_pose: PoseStamped,
        tag_id: int,
        offset: PoseStamped = None,
        orientation_mode: str = "tag_orientation"
    ):
        super().__init__(command_id, stamp, goal_pose)

        self.tag_id = tag_id
        if offset is None:
            self.offset = PoseStamped()
            self.offset.header = goal_pose.header
            # zero position
            self.offset.pose.position.x = 0.0
            self.offset.pose.position.y = 0.0
            self.offset.pose.position.z = 0.0
            self.offset.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            self.offset = offset
        self.orientation_mode = orientation_mode

    def get_offset_pose(self) -> PoseStamped:
        """
        Returns a new PoseStamped which is the sum of goal_pose and offset,
        with orientation adjusted according to orientation_mode.
        """
        result = PoseStamped()
        result.header = self.goal_pose.header
        result = self._add_position_offset(result)
        result = self._add_orientation_offset(result)

        return result


    def _add_position_offset(self, pose: PoseStamped) -> PoseStamped:
        """
        Adds the offset to the given pose's position.
        """
        result = PoseStamped()
        result.header = pose.header
        result.pose.position.x = (
                self.goal_pose.pose.position.x + self.offset.pose.position.x
        )
        result.pose.position.y = (
                self.goal_pose.pose.position.y + self.offset.pose.position.y
        )
        result.pose.position.z = (
                self.goal_pose.pose.position.z + self.offset.pose.position.z
        )
        result.pose.orientation = self.goal_pose.pose.orientation
        return result

    def _add_orientation_offset(self, pose: PoseStamped) -> PoseStamped:
        result = PoseStamped()
        result.header = pose.header
        result.pose.position = pose.pose.position

        if self.orientation_mode == "tag_orientation":
            result.pose.orientation = pose.pose.orientation
            return pose

        elif self.orientation_mode == "look_straight":
            result.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        elif self.orientation_mode == "left":
            result.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == "right":
            result.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=-_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == "up":
            result.pose.orientation = Quaternion(
                x=0.0,
                y=-_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )
        elif self.orientation_mode == "down":
            result.pose.orientation = Quaternion(
                x=0.0,
                y=_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )
        else:
            result.pose.orientation = pose.pose.orientation


        return result

    def get_as_manipulator_move_command_with_offset(self) -> ManipulatorMoveCommand:
        """
        Returns a ManipulatorMoveCommand with the same goal pose and offset.
        """
        return ManipulatorMoveCommand(
            command_id=self.id,
            stamp=self.stamp,
            goal_pose=self.get_offset_pose()
        )