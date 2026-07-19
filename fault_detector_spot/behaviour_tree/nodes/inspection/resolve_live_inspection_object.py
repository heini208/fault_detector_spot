"""Resolve an active object routine from base-camera tags."""

from copy import deepcopy
from pathlib import Path
from typing import Any, Optional, Union

import py_trees
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node

from fault_detector_spot.inspection.live_object_pose_resolver import (
    LiveObjectPoseResolver,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)
from ..sensing.tag_observation_time import (
    is_observation_fresh,
    observation_age_sec,
)


class ResolveLiveInspectionObject(
    py_trees.behaviour.Behaviour
):
    """Produce a tag-defined probe reference for one routine."""

    def __init__(
        self,
        object_id: str,
        routine_id: str,
        execution_frame: str = "odom",
        maximum_age_sec: float = 0.25,
        object_root: Optional[Union[str, Path]] = None,
        name: str = "ResolveLiveInspectionObject",
    ):
        """Configure the active object routine and local frame."""
        super().__init__(name)
        self.object_id = object_id
        self.routine_id = routine_id
        self.execution_frame = execution_frame
        self.maximum_age_sec = maximum_age_sec
        self.object_root = object_root
        self.node: Optional[Node] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[
            tf2_ros.TransformListener
        ] = None
        self.inspection_object: Optional[
            InspectionObject
        ] = None
        self.inspection_routine: Optional[
            InspectionRoutine
        ] = None
        self.configuration_error = ""
        self.resolver = LiveObjectPoseResolver(
            execution_frame=execution_frame,
            maximum_age_sec=maximum_age_sec,
        )
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create TF resources and load the selected routine."""
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(
                f"{self.name}: no ROS node provided"
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
        )
        self.blackboard.register_key(
            "base_tag_observations",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "live_inspection_object",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "live_inspection_object_error",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "inspection_object_definition",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "inspection_routine_definition",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.live_inspection_object = None
        self.blackboard.live_inspection_object_error = ""
        self.blackboard.inspection_object_definition = None
        self.blackboard.inspection_routine_definition = None
        self._load_configuration()

    def update(self) -> py_trees.common.Status:
        """Resolve the object frame from a fresh base tag."""
        if self.configuration_error:
            result = ResolvedObjectPose(
                object_id=self.object_id,
                tag_id=None,
                state=ObjectPoseState.INVALID,
                message=self.configuration_error,
                frame_id=self.execution_frame,
                observation_source="base",
            )
            return self._store(result)

        now = self.node.get_clock().now()
        tag_id = self.inspection_object.reference_tag.tag_id
        tag = self._get_base_tags().get(tag_id)
        if tag is None:
            result = self.resolver.resolve(
                self.inspection_object,
                marker_pose=None,
                current_time=now,
                observed_tag_id=None,
                observation_source="base",
            )
            return self._store(result)

        stamp = tag.pose.header.stamp
        age_sec = observation_age_sec(now, stamp)
        if not is_observation_fresh(
            now,
            stamp,
            self.maximum_age_sec,
        ):
            result = self.resolver.unavailable(
                self.inspection_object,
                f"Base-camera tag is stale: {age_sec:.3f} s",
                stamp=stamp,
                age_sec=age_sec,
            )
            return self._store(result)

        marker_pose = PoseStamped()
        marker_pose.header = deepcopy(tag.pose.header)
        marker_pose.pose = deepcopy(tag.pose.pose)
        marker_error = self.resolver.marker_geometry_error(
            marker_pose
        )
        if marker_error:
            result = self.resolver.invalid(
                self.inspection_object,
                marker_error,
            )
            return self._store(result)

        try:
            marker_in_execution_frame = (
                self.tf_buffer.transform(
                    marker_pose,
                    self.execution_frame,
                    timeout=Duration(seconds=0.05),
                )
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exception:
            result = self.resolver.unavailable(
                self.inspection_object,
                "Cannot transform base-camera tag to "
                f"{self.execution_frame}: {exception}",
                stamp=stamp,
                age_sec=age_sec,
            )
            return self._store(result)

        result = self.resolver.resolve(
            self.inspection_object,
            marker_pose=marker_in_execution_frame,
            current_time=now,
            observed_tag_id=int(tag.id),
            observation_source="base",
        )
        return self._store(result)

    def _load_configuration(self) -> None:
        """Load the configured object and selected routine."""
        self.configuration_error = ""
        if not self.object_id or not self.routine_id:
            self.configuration_error = (
                "Set inspection.active_object_id and "
                "inspection.active_routine_id"
            )
            return

        try:
            self.inspection_object = ObjectRepository(
                self.object_root
            ).load(self.object_id)
            self.inspection_routine = (
                self.inspection_object.get_routine(
                    self.routine_id
                )
            )
            if self.inspection_routine is None:
                raise ValueError(
                    f"Unknown routine {self.routine_id} for object "
                    f"{self.object_id}"
                )
        except (
            FileNotFoundError,
            KeyError,
            OSError,
            TypeError,
            ValueError,
        ) as exception:
            self.configuration_error = str(exception)
            return

        if self.blackboard.exists(
            "inspection_object_definition"
        ):
            self.blackboard.inspection_object_definition = (
                self.inspection_object
            )
            self.blackboard.inspection_routine_definition = (
                self.inspection_routine
            )

    def _get_base_tags(self):
        """Return Spot base-camera fiducial observations."""
        if not self.blackboard.exists(
            "base_tag_observations"
        ):
            return {}
        return self.blackboard.base_tag_observations or {}

    def _store(
        self,
        result: ResolvedObjectPose,
    ) -> py_trees.common.Status:
        """Store the result while keeping sensing non-failing."""
        self.blackboard.live_inspection_object = result
        self.blackboard.live_inspection_object_error = (
            ""
            if result.state == ObjectPoseState.LIVE
            else result.message
        )
        self.feedback_message = (
            f"Local object {result.object_id}: "
            f"{result.state.value} ({result.message})"
        )
        return py_trees.common.Status.SUCCESS
