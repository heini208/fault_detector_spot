"""Publish resolved inspection object states."""

from typing import Any, Dict, Optional

import py_trees
import rclpy
from fault_detector_msgs.msg import (
    InspectionObjectStateArray,
)
from rclpy.node import Node
from std_msgs.msg import Header

from fault_detector_spot.behaviour_tree.QOS_PROFILES import (
    LATCHED_QOS,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ResolvedObjectPose,
)
from fault_detector_spot.inspection.object_state_adapter import (
    resolved_object_poses_to_array_msg,
)


class PublishInspectionObjects(
    py_trees.behaviour.Behaviour,
):
    """Publish inspection object states from the blackboard."""

    def __init__(
        self,
        name: str = "PublishInspectionObjects",
    ):
        super().__init__(name)

        self.node: Optional[Node] = None
        self.publisher: Optional[
            rclpy.publisher.Publisher
        ] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create the publisher and register blackboard keys."""
        self.node = kwargs["node"]

        self.publisher = self.node.create_publisher(
            InspectionObjectStateArray,
            "fault_detector/state/inspection_objects",
            LATCHED_QOS,
        )

        self.blackboard.register_key(
            "inspection_objects",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "inspection_object_error",
            access=py_trees.common.Access.READ,
        )

    def update(self) -> py_trees.common.Status:
        """Publish the latest resolved object states."""
        resolved_objects = self._get_objects()
        error_message = self._get_error_message()

        header = Header()
        header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        header.frame_id = "map"

        message = resolved_object_poses_to_array_msg(
            resolved_objects.values(),
            error_message=error_message,
            header=header,
        )

        self.publisher.publish(message)

        self.feedback_message = (
            f"Published {len(message.objects)} "
            f"inspection object state(s)"
        )

        return py_trees.common.Status.SUCCESS

    def _get_objects(
        self,
    ) -> Dict[str, ResolvedObjectPose]:
        """Return resolved objects from the blackboard."""
        if not self.blackboard.exists(
            "inspection_objects"
        ):
            return {}

        return (
            self.blackboard.inspection_objects
            or {}
        )

    def _get_error_message(self) -> str:
        """Return the latest resolution error."""
        if not self.blackboard.exists(
            "inspection_object_error"
        ):
            return ""

        return (
            self.blackboard.inspection_object_error
            or ""
        )