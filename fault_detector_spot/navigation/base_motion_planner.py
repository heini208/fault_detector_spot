"""Resolve Spot base movement requests into executable movement plans."""

from copy import deepcopy
from dataclasses import dataclass

from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import TransformException
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with
from tf2_geometry_msgs import do_transform_pose_stamped

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import QuaternionData

from fault_detector_spot.navigation.walking_profile import (
    GAITS,
    WalkingProfile,
    WalkingProfiles,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryResolver,
    MovementGeometryUnavailable,
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

    def prepare_tag_request(self, command):
        """Freeze robot-relative offsets once, after posture readiness."""
        prepared = deepcopy(command)
        offset = getattr(prepared, "offset", None)
        if offset is None or offset.header.frame_id.strip().split("/")[-1] not in {
            "body", "flat_body", "gpe",
        }:
            return prepared
        source = offset.header.frame_id
        try:
            vector = prepared._rotate_vector_into_frame(
                [offset.pose.position.x, offset.pose.position.y,
                 offset.pose.position.z],
                source, ODOM_FRAME_NAME, self.tf_listener,
            )
            rotation = prepared._rotate_quaternion_into_frame(
                [offset.pose.orientation.x, offset.pose.orientation.y,
                 offset.pose.orientation.z, offset.pose.orientation.w],
                source, ODOM_FRAME_NAME, self.tf_listener,
            )
        except TransformException as exception:
            raise MovementGeometryUnavailable(str(exception)) from exception
        offset.header.frame_id = ODOM_FRAME_NAME
        offset.pose.position.x, offset.pose.position.y, offset.pose.position.z = map(float, vector)
        (offset.pose.orientation.x, offset.pose.orientation.y,
         offset.pose.orientation.z, offset.pose.orientation.w) = map(float, rotation)
        return prepared

    def _observation_in_odom(self, pose):
        if pose.header.frame_id.strip() == ODOM_FRAME_NAME:
            return deepcopy(pose)
        stamp = pose.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise MovementGeometryUnavailable("Tag observation has no capture timestamp")
        try:
            transform = self.tf_listener.lookup_a_tform_b(
                ODOM_FRAME_NAME,
                pose.header.frame_id,
                transform_time=Time.from_msg(stamp),
                timeout_sec=0.0,
            )
        except TransformException as exception:
            raise MovementGeometryUnavailable(str(exception)) from exception
        result = do_transform_pose_stamped(pose, transform)
        result.header.frame_id = ODOM_FRAME_NAME
        result.header.stamp = deepcopy(stamp)
        return result

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
        return self.resolve_tag_observation(command, tag)

    def resolve_tag_observation(
        self,
        command,
        observation,
    ) -> BaseMovementPlan:
        if command is None or not hasattr(command, "tag_id"):
            raise TypeError(
                "Tag base movement requires a command with tag_id"
            )
        if not callable(getattr(command, "compute_goal_pose", None)):
            raise TypeError(
                "Tag base movement requires compute_goal_pose()"
            )
        if observation is None or not hasattr(observation, "pose"):
            raise TypeError(
                "Tag base movement requires a tag observation"
            )

        tag_id = int(command.tag_id)
        observed_id = int(getattr(observation, "id", tag_id))
        if observed_id != tag_id:
            raise ValueError(
                f"Tag observation {observed_id} does not match "
                f"requested tag {tag_id}"
            )

        prepared = deepcopy(command)
        prepared.tag_pose = self._observation_in_odom(observation.pose)
        prepared = self.geometry_resolver.prepare_move_command(
            prepared,
            ODOM_FRAME_NAME,
        )
        target = self.normalize_target(
            prepared.compute_goal_pose(self.tf_listener)
        )
        profile = self.walking_profiles.for_move(
            True,
            getattr(prepared, "walking_profile", ""),
        )
        return BaseMovementPlan(
            target=target,
            linear_speed_mps=profile.tag_speed_mps,
            profile=profile,
        )

    @staticmethod
    def planar_target(plan: BaseMovementPlan):
        if not isinstance(plan, BaseMovementPlan):
            raise TypeError(
                "Planar base target requires a BaseMovementPlan"
            )

        target = plan.target
        orientation = target.pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )
        return (
            target.pose.position.x,
            target.pose.position.y,
            yaw,
        )

    def build_goal(
        self,
        plan: BaseMovementPlan,
        robot_name: str = "",
    ) -> RobotCommand.Goal:
        if not isinstance(plan, BaseMovementPlan):
            raise TypeError(
                "Base goal construction requires a BaseMovementPlan"
            )

        target = plan.target
        x, y, yaw = self.planar_target(plan)
        profile = plan.profile
        speed = float(plan.linear_speed_mps)
        velocity_limit = SE2VelocityLimit(
            max_vel=math_helpers.SE2Velocity(
                speed,
                speed,
                profile.angular_speed_rad_s,
            ).to_proto(),
            min_vel=math_helpers.SE2Velocity(
                -speed,
                -speed,
                -profile.angular_speed_rad_s,
            ).to_proto(),
        )
        params = RobotCommandBuilder.mobility_params(
            locomotion_hint=GAITS[profile.gait],
        )
        params.vel_limit.CopyFrom(velocity_limit)

        command = (
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=x,
                goal_y=y,
                goal_heading=yaw,
                frame_name=namespace_with(
                    robot_name,
                    target.header.frame_id,
                ),
                params=params,
            )
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal

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
