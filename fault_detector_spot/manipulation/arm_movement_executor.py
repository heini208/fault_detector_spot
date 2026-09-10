"""Shared construction of Cartesian Spot arm movements."""

from copy import deepcopy
import math

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
import tf2_geometry_msgs

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import (
    transform_to_pose_data,
)


class ArmMovementExecutor:
    """Resolve Cartesian hand and probe targets into Spot arm goals."""

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
    ):
        if tf_listener is None:
            raise ValueError(
                "ArmMovementExecutor requires a TF listener"
            )
        self.tf_listener = tf_listener
        self.tag_state_source = tag_state_source
        self.robot_name = robot_name

    def relative(
        self,
        command,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Build a hand-relative movement in gravity-aligned body frame."""
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative arm movement requires a command with "
                "compute_goal_pose()"
            )
        target = command.compute_goal_pose(self.tf_listener)
        return self.pose(
            target,
            duration_sec,
            execution_frame=GRAV_ALIGNED_BODY_FRAME_NAME,
        )

    def probe_pose(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Move the active probe frame to an absolute target pose."""
        if not isinstance(probe_target, PoseStamped):
            raise TypeError("Probe target must be a PoseStamped")

        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Probe movement requires attachment geometry"
            )
        if sensor_id == BARE_HAND_MOTION_ID:
            return self.pose(probe_target, duration_sec)

        probe_frame = sensor_probe_frame(sensor_id)
        hand_to_probe = self.tf_listener.lookup_a_tform_b(
            HAND_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        hand_to_probe_pose = pose_data_to_pose(
            transform_to_pose_data(hand_to_probe)
        )

        hand_target = deepcopy(probe_target)
        hand_target.pose = compose_poses(
            probe_target.pose,
            inverse_pose(hand_to_probe_pose),
        )
        return self.pose(hand_target, duration_sec)

    def tag_probe(
        self,
        command,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Move the active probe to a target relative to a live tag."""
        if self.tag_state_source is None:
            raise RuntimeError(
                "Tag probe movement requires a tag state source"
            )
        if command is None or not hasattr(command, "tag_id"):
            raise TypeError(
                "Tag probe movement requires a command with tag_id"
            )
        if not callable(getattr(command, "compute_goal_pose", None)):
            raise TypeError(
                "Tag probe movement requires compute_goal_pose()"
            )

        tag_id = int(command.tag_id)
        tag = self.tag_state_source.reachable_tag(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently reachable"
            )

        command.tag_pose = deepcopy(tag.pose)
        probe_target = command.compute_goal_pose(self.tf_listener)
        return self.probe_pose(
            probe_target,
            command.motion_sensor_id,
            duration_sec,
        )

    def probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Apply a local relative transform around the current probe frame."""
        if not isinstance(offset, PoseStamped):
            raise TypeError(
                "Probe-relative offset must be a PoseStamped"
            )

        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Probe-relative movement requires attachment geometry"
            )

        probe_frame = sensor_probe_frame(sensor_id)
        offset_frame = offset.header.frame_id.strip()
        if offset_frame != probe_frame:
            raise ValueError(
                "Probe-relative offset must be expressed in the "
                f"active probe frame '{probe_frame}'"
            )

        probe_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        probe_target = tf2_geometry_msgs.do_transform_pose_stamped(
            offset,
            probe_to_execution,
        )
        return self.probe_pose(
            probe_target,
            sensor_id,
            duration_sec,
        )

    def pose(
        self,
        target: PoseStamped,
        duration_sec: float,
        execution_frame: str = "",
    ) -> RobotCommand.Goal:
        """Build an absolute Cartesian hand-pose RobotCommand goal."""
        if not isinstance(target, PoseStamped):
            raise TypeError("Arm pose target must be a PoseStamped")
        duration = float(duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError(
                "Arm movement duration must be positive and finite"
            )

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        normalized_target = target
        normalized_frame = execution_frame.strip()
        if normalized_frame and target_frame != normalized_frame:
            transform = self.tf_listener.lookup_a_tform_b(
                normalized_frame,
                target_frame,
                timeout_sec=0.0,
            )
            normalized_target = (
                tf2_geometry_msgs.do_transform_pose_stamped(
                    target,
                    transform,
                )
            )
            target_frame = normalized_frame

        pose = normalized_target.pose
        command = RobotCommandBuilder.arm_pose_command(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            namespace_with(self.robot_name, target_frame),
            duration,
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal


__all__ = ["ArmMovementExecutor"]
