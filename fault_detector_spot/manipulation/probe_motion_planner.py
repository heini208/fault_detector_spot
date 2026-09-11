"""Resolve probe targets and build low-level Cartesian motion plans."""

from copy import deepcopy
from dataclasses import dataclass
import math

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryResolver,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import (
    transform_to_pose_data,
)


@dataclass(frozen=True)
class ResolvedProbeTarget:
    """Resolved probe target with optional reusable current geometry."""

    target: PoseStamped
    sensor_id: str
    current_probe: PoseStamped | None = None


@dataclass(frozen=True)
class ProbeMotionPlan:
    """Resolved geometry for one probe motion."""

    goal: object | None
    current_hand: PoseStamped
    target_hand: PoseStamped
    direction_x: float
    direction_y: float
    direction_z: float
    linear_speed_mps: float
    direction_frame: str = ""
    motion_required: bool = True
    force_guard_enabled: bool = True


class ProbeMotionPlanner:
    """Resolve probe targets and convert them to one RobotCommand goal."""

    def __init__(
        self,
        tf_listener,
        speed_policy: ArmMotionSpeedPolicy,
        build_pose_goal,
    ):
        if tf_listener is None:
            raise RuntimeError("ProbeMotionPlanner requires a TF listener")
        if not isinstance(speed_policy, ArmMotionSpeedPolicy):
            raise TypeError(
                "ProbeMotionPlanner requires an ArmMotionSpeedPolicy"
            )
        if not callable(build_pose_goal):
            raise TypeError(
                "ProbeMotionPlanner build_pose_goal must be callable"
            )

        self.tf_listener = tf_listener
        self.speed_policy = speed_policy
        self.geometry_resolver = MovementGeometryResolver(tf_listener)
        self._build_pose_goal = build_pose_goal

    def resolve_absolute(
        self,
        target: PoseStamped,
        execution_frame: str = "",
    ):
        normalized = self.normalize_target(
            target,
            execution_frame,
        )
        return ResolvedProbeTarget(
            target=normalized,
            sensor_id=BARE_HAND_MOTION_ID,
        )

    def resolve_relative(self, command):
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative arm movement requires a command with "
                "compute_goal_pose()"
            )

        command = self.geometry_resolver.prepare_move_command(
            command,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        relative_target = command.compute_goal_pose(self.tf_listener)
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
        current_probe = None
        if source_frame == HAND_FRAME_NAME:
            current_probe = self.pose_from_transform(
                source_to_execution,
                GRAV_ALIGNED_BODY_FRAME_NAME,
            )
        return ResolvedProbeTarget(
            target=target_hand,
            sensor_id=BARE_HAND_MOTION_ID,
            current_probe=current_probe,
        )

    def resolve_tag(self, command, tag_state_source):
        if tag_state_source is None:
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
        tag = tag_state_source.reachable_tag(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently reachable"
            )

        command.tag_pose = deepcopy(tag.pose)
        command = self.geometry_resolver.prepare_move_command(
            command,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        probe_target = command.compute_goal_pose(self.tf_listener)
        return ResolvedProbeTarget(
            target=probe_target,
            sensor_id=command.motion_sensor_id,
        )

    def resolve_probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
    ):
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
        if offset.header.frame_id.strip() != probe_frame:
            raise ValueError(
                "Probe-relative offset must be expressed in the "
                f"active probe frame '{probe_frame}'"
            )

        probe_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        target_probe = tf2_geometry_msgs.do_transform_pose_stamped(
            offset,
            probe_to_execution,
        )
        current_probe = self.pose_from_transform(
            probe_to_execution,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        return ResolvedProbeTarget(
            target=target_probe,
            sensor_id=sensor_id,
            current_probe=current_probe,
        )

    def resolved_target(
        self,
        target: PoseStamped,
        motion_sensor_id: str,
    ) -> ResolvedProbeTarget:
        return ResolvedProbeTarget(
            target=deepcopy(target),
            sensor_id=str(motion_sensor_id).strip(),
        )

    @staticmethod
    def _resolved_target(value) -> ResolvedProbeTarget:
        if isinstance(value, ResolvedProbeTarget):
            return value
        if (
            isinstance(value, tuple)
            and len(value) == 2
        ):
            return ResolvedProbeTarget(
                target=value[0],
                sensor_id=value[1],
            )
        raise TypeError(
            "Probe target builder must return ResolvedProbeTarget "
            "or (PoseStamped, sensor_id)"
        )

    def build_plan(
        self,
        target_builder,
        speed=None,
    ) -> ProbeMotionPlan:
        resolved = self._resolved_target(target_builder())
        probe_target = resolved.target
        sensor_id = str(resolved.sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Guarded probe movement requires attachment geometry"
            )

        target_probe = self.normalize_target(probe_target)
        target_frame = target_probe.header.frame_id.strip()
        current_probe = resolved.current_probe
        if current_probe is None:
            current_probe = self.current_pose(
                target_frame,
                sensor_probe_frame(sensor_id),
            )
        effective_speed = self.effective_speed(speed)
        duration_sec = self.speed_policy.duration_between(
            current_probe.pose,
            target_probe.pose,
            speed=effective_speed,
        )

        if sensor_id == BARE_HAND_MOTION_ID:
            current_hand = deepcopy(current_probe)
            target_hand = deepcopy(target_probe)
        else:
            hand_to_probe_pose = self.hand_to_probe_pose(sensor_id)
            current_hand = self.probe_pose_to_hand_pose(
                current_probe,
                hand_to_probe_pose,
            )
            target_hand = self.probe_pose_to_hand_pose(
                target_probe,
                hand_to_probe_pose,
            )

        probe_rotation = self.speed_policy._rotation_angle(
            current_probe.pose,
            target_probe.pose,
        )
        start = current_hand.pose.position
        target = target_hand.pose.position
        dx = float(target.x) - float(start.x)
        dy = float(target.y) - float(start.y)
        dz = float(target.z) - float(start.z)
        hand_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        motion_required = (
            hand_distance > 1e-6
            or probe_rotation > 1e-6
        )
        goal = self._build_pose_goal(target_hand, duration_sec)
        if not motion_required:
            return ProbeMotionPlan(
                goal=goal,
                current_hand=deepcopy(current_hand),
                target_hand=deepcopy(target_hand),
                direction_x=0.0,
                direction_y=0.0,
                direction_z=0.0,
                linear_speed_mps=0.0,
                direction_frame=target_frame,
                motion_required=True,
                force_guard_enabled=False,
            )

        if hand_distance <= 1e-6:
            return ProbeMotionPlan(
                goal=goal,
                current_hand=deepcopy(current_hand),
                target_hand=deepcopy(target_hand),
                direction_x=0.0,
                direction_y=0.0,
                direction_z=0.0,
                linear_speed_mps=0.0,
                direction_frame=target_frame,
                motion_required=True,
                force_guard_enabled=False,
            )

        return ProbeMotionPlan(
            goal=goal,
            current_hand=deepcopy(current_hand),
            target_hand=deepcopy(target_hand),
            direction_x=dx / hand_distance,
            direction_y=dy / hand_distance,
            direction_z=dz / hand_distance,
            linear_speed_mps=hand_distance / duration_sec,
            direction_frame=target_frame,
            motion_required=True,
            force_guard_enabled=True,
        )

    def build_probe_goal(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ):
        if not isinstance(probe_target, PoseStamped):
            raise TypeError("Probe target must be a PoseStamped")

        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Probe movement requires attachment geometry"
            )

        target_probe = self.normalize_target(probe_target)
        target_frame = target_probe.header.frame_id.strip()
        current_probe = self.current_pose(
            target_frame,
            sensor_probe_frame(sensor_id),
        )
        duration_sec = self.speed_policy.duration_between(
            current_probe.pose,
            target_probe.pose,
            speed=speed,
        )
        if sensor_id == BARE_HAND_MOTION_ID:
            hand_target = target_probe
        else:
            hand_target = self.probe_target_to_hand_target(
                target_probe,
                sensor_id,
            )
        return self._build_pose_goal(
            hand_target,
            duration_sec,
        )

    def build_motion_goal(
        self,
        current_hand: PoseStamped,
        target_hand: PoseStamped,
        speed=None,
    ):
        duration_sec = self.speed_policy.duration_between(
            current_hand.pose,
            target_hand.pose,
            speed=speed,
        )
        return self._build_pose_goal(
            target_hand,
            duration_sec,
        )

    def current_hand_pose(
        self,
        frame_id: str = GRAV_ALIGNED_BODY_FRAME_NAME,
    ) -> PoseStamped:
        normalized_frame = str(frame_id).strip()
        if not normalized_frame:
            normalized_frame = GRAV_ALIGNED_BODY_FRAME_NAME
        return self.current_pose(
            normalized_frame,
            HAND_FRAME_NAME,
        )

    def normalize_target(
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
            return deepcopy(target)

        transform = self.tf_listener.lookup_a_tform_b(
            normalized_frame,
            target_frame,
            timeout_sec=0.0,
        )
        return tf2_geometry_msgs.do_transform_pose_stamped(
            target,
            transform,
        )

    def current_pose(
        self,
        target_frame: str,
        controlled_frame: str,
    ) -> PoseStamped:
        transform = self.tf_listener.lookup_a_tform_b(
            target_frame,
            controlled_frame,
            timeout_sec=0.0,
        )
        return self.pose_from_transform(
            transform,
            target_frame,
        )

    def hand_to_probe_pose(self, sensor_id: str):
        probe_frame = sensor_probe_frame(sensor_id)
        hand_to_probe = self.tf_listener.lookup_a_tform_b(
            HAND_FRAME_NAME,
            probe_frame,
            timeout_sec=0.0,
        )
        return pose_data_to_pose(
            transform_to_pose_data(hand_to_probe)
        )

    @staticmethod
    def probe_pose_to_hand_pose(
        probe_pose: PoseStamped,
        hand_to_probe_pose,
    ) -> PoseStamped:
        hand_pose = deepcopy(probe_pose)
        hand_pose.pose = compose_poses(
            probe_pose.pose,
            inverse_pose(hand_to_probe_pose),
        )
        return hand_pose

    def probe_target_to_hand_target(
        self,
        probe_target: PoseStamped,
        sensor_id: str,
    ) -> PoseStamped:
        return self.probe_pose_to_hand_pose(
            probe_target,
            self.hand_to_probe_pose(sensor_id),
        )

    def effective_speed(self, speed):
        if speed is None:
            return self.speed_policy.default_speed
        if not isinstance(speed, ArmMotionSpeed):
            raise TypeError(
                "Guarded probe speed must be an ArmMotionSpeed"
            )
        return speed

    @classmethod
    def relative_command_is_noop(cls, command) -> bool:
        return cls.pose_offset_is_noop(getattr(command, "offset", None))

    @staticmethod
    def pose_offset_is_noop(offset) -> bool:
        if not isinstance(offset, PoseStamped):
            return False

        position = offset.pose.position
        translation = math.sqrt(
            float(position.x) * float(position.x)
            + float(position.y) * float(position.y)
            + float(position.z) * float(position.z)
        )
        if translation > 1e-6:
            return False

        orientation = offset.pose.orientation
        values = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-12:
            return False
        x, y, z, w = (value / norm for value in values)
        return (
            abs(x) <= 1e-6
            and abs(y) <= 1e-6
            and abs(z) <= 1e-6
            and abs(abs(w) - 1.0) <= 1e-6
        )

    @staticmethod
    def pose_from_transform(
        transform,
        frame_id: str,
    ) -> PoseStamped:
        current = PoseStamped()
        current.header.frame_id = frame_id
        current.pose = pose_data_to_pose(
            transform_to_pose_data(transform)
        )
        return current


__all__ = [
    "ProbeMotionPlan",
    "ProbeMotionPlanner",
    "ResolvedProbeTarget",
]
