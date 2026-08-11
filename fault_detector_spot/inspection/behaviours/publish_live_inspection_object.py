"""Publish the mapless active inspection object state."""

from typing import Any, Optional

import py_trees
import rclpy
from fault_detector_msgs.msg import LiveInspectionObjectState
from rclpy.node import Node
from std_msgs.msg import Header

from fault_detector_spot.shared.ros.qos_profiles import (
    LIVE_OBJECT_QOS,
)
from fault_detector_spot.inspection.ros.live_object_state_adapter import (
    live_object_pose_to_msg,
)
from fault_detector_spot.inspection.model.resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)


class PublishLiveInspectionObject(
    py_trees.behaviour.Behaviour
):
    """Publish the active local object on a dedicated topic."""

    def __init__(
        self,
        topic_name: str = (
            "fault_detector/state/live_inspection_object"
        ),
        name: str = "PublishLiveInspectionObject",
    ):
        """Configure the local state topic."""
        super().__init__(name)
        self.topic_name = topic_name
        self.node: Optional[Node] = None
        self.publisher: Optional[
            rclpy.publisher.Publisher
        ] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create the publisher and register the result input."""
        self.node = kwargs.get("node")

        if self.node is None:
            raise RuntimeError(
                f"{self.name}: no ROS node provided"
            )

        self.publisher = self.node.create_publisher(
            LiveInspectionObjectState,
            self.topic_name,
            LIVE_OBJECT_QOS,
        )
        self.blackboard.register_key(
            "live_inspection_object",
            access=py_trees.common.Access.READ,
        )

    def update(self) -> py_trees.common.Status:
        """Publish the most recent local resolution result."""
        result = self._get_result()
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = result.frame_id
        message = live_object_pose_to_msg(result, header)
        self.publisher.publish(message)
        self.feedback_message = (
            f"Published local object state: "
            f"{result.state.value}"
        )
        return py_trees.common.Status.SUCCESS

    def _get_result(self) -> ResolvedObjectPose:
        """Return a result even before configuration is ready."""
        if self.blackboard.exists(
            "live_inspection_object"
        ):
            result = self.blackboard.live_inspection_object

            if result is not None:
                return result

        return ResolvedObjectPose(
            object_id="",
            tag_id=None,
            state=ObjectPoseState.UNAVAILABLE,
            message="No active inspection object",
            frame_id="odom",
            observation_source="base",
        )
