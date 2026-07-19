"""Resolve a fresh object pose from a base-camera fiducial."""

import math
from copy import deepcopy
from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from .models import InspectionObject
from .resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)


class LiveObjectPoseResolver:
    """Create probe-usable object poses without map state."""

    def __init__(
        self,
        execution_frame: str = "odom",
        maximum_age_sec: float = 0.25,
    ):
        """Configure the execution frame and freshness limit."""
        if not execution_frame:
            raise ValueError("Execution frame must not be empty")
        if maximum_age_sec <= 0.0:
            raise ValueError(
                "Maximum observation age must be positive"
            )
        self.execution_frame = execution_frame
        self.maximum_age_sec = maximum_age_sec

    def resolve(
        self,
        inspection_object: InspectionObject,
        marker_pose: Optional[PoseStamped],
        current_time: Time,
        observed_tag_id: Optional[int] = None,
        observation_source: str = "base",
    ) -> ResolvedObjectPose:
        """Resolve the tag-defined object frame."""
        try:
            inspection_object.validate()
        except ValueError as exception:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                str(exception),
            )

        if observation_source != "base":
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                "Local object motion accepts base-camera tags only",
            )
        if marker_pose is None:
            return self._result(
                inspection_object,
                ObjectPoseState.UNAVAILABLE,
                "Expected base-camera tag is not visible",
            )

        marker_error = self.marker_geometry_error(marker_pose)
        if marker_error:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                marker_error,
            )

        expected_tag_id = inspection_object.reference_tag.tag_id
        if observed_tag_id != expected_tag_id:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                "Observed tag does not match object tag: "
                f"{observed_tag_id} != {expected_tag_id}",
            )
        if marker_pose.header.frame_id != self.execution_frame:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                f"Marker is in frame '{marker_pose.header.frame_id}', "
                f"expected '{self.execution_frame}'",
            )

        stamp = marker_pose.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                "Marker observation has a zero timestamp",
            )

        observation_time = Time.from_msg(
            stamp,
            clock_type=current_time.clock_type,
        )
        age_sec = (
            current_time - observation_time
        ).nanoseconds / 1_000_000_000.0

        if age_sec < -0.1:
            return self._result(
                inspection_object,
                ObjectPoseState.INVALID,
                "Marker observation timestamp is in the future",
                stamp=stamp,
                age_sec=age_sec,
            )
        if age_sec > self.maximum_age_sec:
            return self._result(
                inspection_object,
                ObjectPoseState.UNAVAILABLE,
                f"Base-camera tag is stale: {age_sec:.3f} s",
                stamp=stamp,
                age_sec=age_sec,
            )

        object_pose = deepcopy(marker_pose)
        return ResolvedObjectPose(
            object_id=inspection_object.object_id,
            tag_id=expected_tag_id,
            state=ObjectPoseState.LIVE,
            selected_pose=object_pose,
            message="Using fresh base-camera marker observation",
            frame_id=self.execution_frame,
            observation_timestamp=deepcopy(stamp),
            observation_age_sec=age_sec,
            observation_source="base",
        )

    @staticmethod
    def marker_geometry_error(
        marker_pose: PoseStamped,
    ) -> Optional[str]:
        """Return an error for unsafe marker geometry."""
        pose = marker_pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return "Marker pose contains a non-finite value"

        quaternion_norm = math.sqrt(
            pose.orientation.x ** 2
            + pose.orientation.y ** 2
            + pose.orientation.z ** 2
            + pose.orientation.w ** 2
        )

        if quaternion_norm < 1e-12:
            return "Marker pose quaternion norm is zero"

        if not math.isclose(
                quaternion_norm,
                1.0,
                rel_tol=0.0,
                abs_tol=1e-3,
        ):
            return "Marker pose quaternion must be normalized"

        return None

    def unavailable(
        self,
        definition: InspectionObject,
        message: str,
        stamp=None,
        age_sec=None,
    ) -> ResolvedObjectPose:
        """Create an unavailable result."""
        return self._result(
            definition,
            ObjectPoseState.UNAVAILABLE,
            message,
            stamp=stamp,
            age_sec=age_sec,
        )

    def invalid(
        self,
        definition: InspectionObject,
        message: str,
    ) -> ResolvedObjectPose:
        """Create an invalid result."""
        return self._result(
            definition,
            ObjectPoseState.INVALID,
            message,
        )

    def _result(
        self,
        definition: InspectionObject,
        state: ObjectPoseState,
        message: str,
        stamp=None,
        age_sec=None,
    ) -> ResolvedObjectPose:
        """Create a non-live result with consistent metadata."""
        return ResolvedObjectPose(
            object_id=definition.object_id,
            tag_id=definition.reference_tag.tag_id,
            state=state,
            message=message,
            frame_id=self.execution_frame,
            observation_timestamp=deepcopy(stamp),
            observation_age_sec=age_sec,
            observation_source="base",
        )