# fault_detector_spot/behaviour_tree/nodes/manipulator_command_subscriber.py

import py_trees
import rclpy
from fault_detector_msgs.msg import TagElement
from typing import Optional


class ManipulatorCommandSubscriber(py_trees.behaviour.Behaviour):
    """
    Subscribes to tag commands from the UI and updates the blackboard
    for use by manipulator control nodes.
    """

    def __init__(self, name: str = "ManipulatorCommandSubscriber",
                 topic: str = "fault_detector/commands/move_to_tag"):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.subscription = None
        self.topic = topic
        self.blackboard = self.attach_blackboard_client()
        self.last_command_id = None
        self.last_command_stamp = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']

            # Subscribe to UI commands
            self.subscription = self.node.create_subscription(
                TagElement,
                self.topic,
                self._command_callback,
                10
            )

            # Register blackboard keys
            self.blackboard.register_key(
                key="goal_tag_id", access=py_trees.common.Access.WRITE
            )
            self.blackboard.register_key(
                key="goal_tag_pose", access=py_trees.common.Access.WRITE
            )

            self.logger.info(f"Listening for tag commands on {self.topic}")
        except KeyError as e:
            self.logger.error(f"Could not retrieve node from kwargs: {e}")

    def _command_callback(self, msg: TagElement):
        """Process incoming command from UI"""
        if self.is_last_command(msg):
            return

        try:
            self.blackboard.goal_tag_id = msg.id
            self.blackboard.goal_tag_pose = msg.pose
            self.last_command_id = msg.id
            self.last_command_stamp = msg.pose.header.stamp

            self.logger.info(f"Received command for tag {msg.id}")
        except Exception as e:
            self.logger.error(f"Error processing command: {e}")

    def is_last_command(self, msg) -> bool:
        if self.last_command_stamp is None:
            return False
        stamp = msg.pose.header.stamp
        return stamp.sec == self.last_command_stamp.sec and \
            stamp.nanosec == self.last_command_stamp.nanosec

    def update(self) -> py_trees.common.Status:
        """Always returns SUCCESS to allow parallel execution"""
        if self.last_command_id is not None:
            self.feedback_message = f"Last command: tag {self.last_command_id}"
        else:
            self.feedback_message = "No commands received yet"

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Terminating with status {new_status}")