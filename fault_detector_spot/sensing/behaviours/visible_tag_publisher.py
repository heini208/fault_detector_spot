from typing import Dict, Optional

import py_trees
import rclpy
from fault_detector_msgs.msg import TagElement, TagElementArray
from rclpy.node import Node

from fault_detector_spot.shared.ros.qos_profiles import TAG_STATE_QOS


class PublishReachableTags(py_trees.behaviour.Behaviour):
    """Publish the reachability set derived by the behavior tree."""

    def __init__(self, name: str = "PublishReachableTags"):
        super().__init__(name)
        self.node: Optional[Node] = None
        self.publisher = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")
        self.publisher = self.node.create_publisher(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            TAG_STATE_QOS,
        )
        self.blackboard.register_key(
            "reachable_tags",
            access=py_trees.common.Access.READ,
        )
        return True

    def update(self):
        reachable_tags = getattr(
            self.blackboard,
            "reachable_tags",
            {},
        )
        message = TagElementArray()
        message.elements = list(reachable_tags.values())
        self.publisher.publish(message)
        self.feedback_message = (
            f"Published {len(reachable_tags)} reachable tag(s)"
        )
        return py_trees.common.Status.SUCCESS


class PublishTagStates(py_trees.behaviour.Behaviour):
    """
    Publishes both visible and reachable tags from the blackboard to their respective topics:
    - fault_detector/state/visible_tags
    - fault_detector/state/reachable_tags
    Should be run at the end of the sensing sequence.
    """

    def __init__(self, name: str = "PublishTagStates"):
        super().__init__(name)
        self.node: Optional[Node] = None
        self.tag_publisher: Optional[rclpy.publisher.Publisher] = None
        self.base_tag_publisher: Optional[
            rclpy.publisher.Publisher
        ] = None
        self.reachable_tag_publisher: Optional[rclpy.publisher.Publisher] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
            self.tag_publisher = self.node.create_publisher(
                TagElementArray,
                "fault_detector/state/visible_tags",
                10
            )
            self.base_tag_publisher = self.node.create_publisher(
                TagElementArray,
                "fault_detector/state/base_tags",
                10,
            )
            self.reachable_tag_publisher = self.node.create_publisher(
                TagElementArray,
                "fault_detector/state/reachable_tags",
                10
            )

            self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
            self.blackboard.register_key("reachable_tags", access=py_trees.common.Access.READ)
            self.blackboard.register_key(
                "base_tag_observations",
                access=py_trees.common.Access.READ,
            )

            self.logger.info("PublishTagStates node initialized.")
        except KeyError as e:
            self.logger.error(f"Missing required setup argument: {e}")

    def update(self) -> py_trees.common.Status:
        visible_tags: Dict[int, TagElement] = getattr(self.blackboard, "visible_tags", {})
        reachable_tags: Dict[int, TagElement] = getattr(self.blackboard, "reachable_tags", {})
        base_tags: Dict[int, TagElement] = getattr(
            self.blackboard,
            "base_tag_observations",
            {},
        )

        visible_msg = TagElementArray()
        visible_msg.elements = list(visible_tags.values())

        reachable_msg = TagElementArray()
        reachable_msg.elements = list(reachable_tags.values())

        base_msg = TagElementArray()
        base_msg.elements = list(base_tags.values())

        if self.tag_publisher:
            self.tag_publisher.publish(visible_msg)
        if self.reachable_tag_publisher:
            self.reachable_tag_publisher.publish(reachable_msg)
        if self.base_tag_publisher:
            self.base_tag_publisher.publish(base_msg)

        self.feedback_message = (
            f"Published {len(visible_tags)} visible and {len(reachable_tags)} reachable tag(s)"
        )
        return py_trees.common.Status.SUCCESS
