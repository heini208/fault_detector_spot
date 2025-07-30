import py_trees
import rclpy
from rclpy.node import Node
from typing import Dict, Optional
from fault_detector_msgs.msg import TagElement, TagElementArray


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
            self.reachable_tag_publisher = self.node.create_publisher(
                TagElementArray,
                "fault_detector/state/reachable_tags",
                10
            )

            self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
            self.blackboard.register_key("reachable_tags", access=py_trees.common.Access.READ)

            self.logger.info("PublishTagStates node initialized.")
        except KeyError as e:
            self.logger.error(f"Missing required setup argument: {e}")

    def update(self) -> py_trees.common.Status:
        visible_tags: Dict[int, TagElement] = getattr(self.blackboard, "visible_tags", {})
        reachable_tags: Dict[int, TagElement] = getattr(self.blackboard, "reachable_tags", {})

        visible_msg = TagElementArray()
        visible_msg.elements = list(visible_tags.values())

        reachable_msg = TagElementArray()
        reachable_msg.elements = list(reachable_tags.values())

        if self.tag_publisher:
            self.tag_publisher.publish(visible_msg)
        if self.reachable_tag_publisher:
            self.reachable_tag_publisher.publish(reachable_msg)

        self.feedback_message = (
            f"Published {len(visible_tags)} visible and {len(reachable_tags)} reachable tag(s)"
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.info(f"{self.name} terminated with status {new_status}")