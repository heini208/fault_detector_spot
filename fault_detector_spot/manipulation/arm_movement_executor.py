"""Shared construction of Cartesian Spot arm movements."""

from copy import deepcopy
import math

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
import tf2_geometry_msgs

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
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
    """Resolve controlled-frame targets into Spot hand commands."""

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        speed_policy=None,
    ):
        if tf_listener is None:
            raise ValueError(
                "ArmMovementExecutor requires a TF listener"
            )
        self.tf_listener = tf_listener
        self.tag_state_source = tag_state_source
        self.robot_name = robot_name
        self.speed_policy = (
            speed_policy
            if speed_policy is not None
            else ArmMotionSpeedPolicy()
        )

    def relative(
        self,
        command,
        speed=None,
    ) -> RobotCommand.Goal:
        """Move the hand to a target defined by a relative command."""
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative arm movement requires a command with "
                "compute_goal_pose()"
            )

        relative_target = command.compute_goal_pose(
            self.tf_listener
        )
        source_frame = relative_target.header.frame_id.strip()
        if not source_frame:
            raise ValueError(
                "Relative arm target frame must not be empty"
            )

        source_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            source_frame,
            timeout_sec=0.0,
        )
        target_hand = tf2_geometry_msgs.do_transform_pose_stamped(
            relative_target,
            source_to_execution,
        )

        if source_frame == HAND_FRAME_NAME:
            current_hand = self._pose_from_transform(
                source_to_execution,
                GRAV_ALIGNED_BODY_FRAME_NAME,
            )
        else:
            current_hand = self._current_pose(
                GRAV_ALIGNED_BODY_FRAME_NAME,
                HAND_FRAME_NAME,
            )

        return self._build_motion_goal(
            current_hand,
            target_hand,
            speed,
        )

    def pose(
        self,
        target: PoseStamped,
        execution_frame: str = "",
        speed=None,
    ) -> RobotCommand.Goal:
        """Move the hand to an absolute target pose."""
        target_hand = self._normalize_target(
            target,
            execution_frame,
        )
        current_hand = self._current_pose(
            target_hand.header.frame_id,
            HAND_FRAME_NAME,
        )
        return self._build_motion_goal(
            current_hand,
            target_hand,
            speed,
        )

    def probe_pose(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
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
            return self.pose(
                probe_target,
                speed=speed,
            )

        target_frame = probe_target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Probe target frame must not be empty")

        current_probe = self._current_pose(
            target_frame,
            sensor_probe_frame(sensor_id),
        )
        return self._build_probe_motion_goal(
            current_probe,
            probe_target,
            sensor_id,
            speed,
        )

    def tag_probe(
        self,
        command,
        speed=None,
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
        probe_target = command.compute_goal_pose(
            self.tf_listener
        )
        return self.probe_pose(
            probe_target,
            command.motion_sensor_id,
            speed=speed,
        )

    def probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> RobotCommand.Goal:
        """Move relative to the current active probe frame."""
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
        current_probe = self._pose_from_transform(
            probe_to_execution,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        target_probe = tf2_geometry_msgs.do_transform_pose_stamped(
            offset,
            probe_to_execution,
        )

        if sensor_id == BARE_HAND_MOTION_ID:
            return self._build_motion_goal(
                current_probe,
                target_probe,
                speed,
            )

        return self._build_probe_motion_goal(
            current_probe,
            target_probe,
            sensor_id,
            speed,
        )

    def _build_probe_motion_goal(
        self,
        current_probe: PoseStamped,
        target_probe: PoseStamped,
        sensor_id: str,
        speed=None,
    ) -> RobotCommand.Goal:
        duration_sec = self.speed_policy.duration_between(
            current_probe.pose,
            target_probe.pose,
            speed=speed,
        )
        hand_target = self._probe_target_to_hand_target(
            target_probe,
            sensor_id,
        )
        return self._build_pose_goal(
            hand_target,
            duration_sec,
        )

    def _probe_target_to_hand_target(
        self,
        probe_target: PoseStamped,
        sensor_id: str,
    ) -> PoseStamped:
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
        return hand_target

    def _build_motion_goal(
        self,
        current_hand: PoseStamped,
        target_hand: PoseStamped,
        speed=None,
    ) -> RobotCommand.Goal:
        duration_sec = self.speed_policy.duration_between(
            current_hand.pose,
            target_hand.pose,
            speed=speed,
        )
        return self._build_pose_goal(
            target_hand,
            duration_sec,
        )

    def _normalize_target(
        self,
        target: PoseStamped,
        execution_frame: str = "",
    ) -> PoseStamped:
        if not isinstance(target, PoseStamped):
            raise TypeError("Arm pose target must be a PoseStamped")

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        normalized_frame = execution_frame.strip()
        if not normalized_frame or target_frame == normalized_frame:
            return target

        transform = self.tf_listener.lookup_a_tform_b(
            normalized_frame,
            target_frame,
            timeout_sec=0.0,
        )
        return tf2_geometry_msgs.do_transform_pose_stamped(
            target,
            transform,
        )

    def _current_pose(
        self,
        target_frame: str,
        controlled_frame: str,
    ) -> PoseStamped:
        transform = self.tf_listener.lookup_a_tform_b(
            target_frame,
            controlled_frame,
            timeout_sec=0.0,
        )
        return self._pose_from_transform(
            transform,
            target_frame,
        )

    @staticmethod
    def _pose_from_transform(
        transform,
        frame_id: str,
    ) -> PoseStamped:
        current = PoseStamped()
        current.header.frame_id = frame_id
        current.pose = pose_data_to_pose(
            transform_to_pose_data(transform)
        )
        return current

    def _build_pose_goal(
        self,
        target: PoseStamped,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        """Translate the internal speed result to Spot's duration API."""
        duration = float(duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError(
                "Arm movement duration must be positive and finite"
            )

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        pose = target.pose
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
