"""Resolve Spot base movement requests into executable movement plans."""

from copy import deepcopy
from dataclasses import dataclass

from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose_stamped

from fault_detector_spot.navigation.walking_profile import (
    WalkingProfile,
    WalkingProfiles,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryResolver,
)


@dataclass(frozen=True)
class BaseMovementPlan:
    """Resolved planar base target and walking settings."""

    target: PoseStamped
    linear_speed_mps: float
    profile: WalkingProfile


class BaseMotionPlanner:
    """Resolve live base movement geometry independently of execution."""

    def __init__(
        self,
        tf_listener,
        walking_profiles: WalkingProfiles,
    ):
        if tf_listener is None:
            raise RuntimeError("BaseMotionPlanner requires a TF listener")
        if not isinstance(walking_profiles, WalkingProfiles):
            raise TypeError(
                "BaseMotionPlanner requires WalkingProfiles"
            )

        self.tf_listener = tf_listener
        self.walking_profiles = walking_profiles
        self.geometry_resolver = MovementGeometryResolver(tf_listener)

    def resolve_relative(self, command) -> BaseMovementPlan:
        if command is None or not callable(
            getattr(command, "compute_goal_pose", None)
        ):
            raise TypeError(
                "Relative base movement requires a command with "
                "compute_goal_pose()"
            )

        command = self.geometry_resolver.prepare_move_command(
            command,
            ODOM_FRAME_NAME,
        )
        target = self.normalize_target(
            command.compute_goal_pose(self.tf_listener)
        )
        profile = self.walking_profiles.for_move(
            override=getattr(command, "walking_profile", ""),
        )
        return BaseMovementPlan(
            target=target,
            linear_speed_mps=profile.relative_speed_mps,
            profile=profile,
        )

    def resolve_tag(
        self,
        command,
        tag_state_source,
    ) -> BaseMovementPlan:
        if tag_state_source is None:
            raise RuntimeError(
                "Tag base movement requires a tag state source"
            )
        if command is None or not hasattr(command, "tag_id"):
            raise TypeError(
                "Tag base movement requires a command with tag_id"
            )
        if not callable(getattr(command, "compute_goal_pose", None)):
            raise TypeError(
                "Tag base movement requires compute_goal_pose()"
            )

        tag_id = int(command.tag_id)
        tag = tag_state_source.visible_snapshot().get(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently visible"
            )

        command.tag_pose = deepcopy(tag.pose)
        command = self.geometry_resolver.prepare_move_command(
            command,
            ODOM_FRAME_NAME,
        )
        target = self.normalize_target(
            command.compute_goal_pose(self.tf_listener)
        )
        profile = self.walking_profiles.for_move(
            True,
            getattr(command, "walking_profile", ""),
        )
        return BaseMovementPlan(
            target=target,
            linear_speed_mps=profile.tag_speed_mps,
            profile=profile,
        )

    def normalize_target(self, target: PoseStamped) -> PoseStamped:
        if not isinstance(target, PoseStamped):
            raise TypeError("Base target must be a PoseStamped")

        source_frame = target.header.frame_id.strip()
        if not source_frame:
            raise ValueError("Base target frame must not be empty")

        if source_frame == ODOM_FRAME_NAME:
            return deepcopy(target)

        transform = self.tf_listener.lookup_a_tform_b(
            ODOM_FRAME_NAME,
            source_frame,
            timeout_sec=0.0,
        )
        normalized = do_transform_pose_stamped(
            target,
            transform,
        )
        normalized.header.frame_id = ODOM_FRAME_NAME
        return normalized


__all__ = ["BaseMotionPlanner", "BaseMovementPlan"]
