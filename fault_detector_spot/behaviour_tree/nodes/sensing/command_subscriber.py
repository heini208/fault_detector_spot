# fault_detector_spot/behaviour_tree/nodes/command_subscriber.py

import py_trees
import rclpy
from fault_detector_msgs.msg import TagElement, BasicCommand
from fault_detector_spot.behaviour_tree.Tag_Command import TagCommand
from std_msgs.msg import Header
from typing import Optional
from fault_detector_spot.behaviour_tree.command_ids import CommandID
from fault_detector_spot.behaviour_tree.simple_command import SimpleCommand


class CommandSubscriber(py_trees.behaviour.Behaviour):
    """
    Subscribes to UI commands
    and updates the blackboard for use by manipulator control nodes.
    """

    def __init__(self,
                 name: str = "CommandSubscriber",
                 move_topic: str = "fault_detector/commands/move_to_tag",
                 command_topic: str = "fault_detector/commands/basic_command"):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.move_to_tag_topic = move_topic
        self.command_topic        = command_topic
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
            self._create_ui_subscribers()
            self._register_blackboard_keys()
        except KeyError as e:
            self.logger.error(f"Could not retrieve node from kwargs: {e}")

    def update(self) -> py_trees.common.Status:
        """only for updating feedback message"""
        if self.blackboard.last_command is not None:
            self.feedback_message = f"Last command: tag {self.blackboard.last_command.id}"
        else:
            self.feedback_message = "No commands received yet"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Terminating with status {new_status}")

    def _create_ui_subscribers(self):
        # Move-to-tag command
        self.node.create_subscription(
            TagElement,
            self.move_to_tag_topic,
            self._tag_command_callback,
            10
        )
        self.node.create_subscription(
            BasicCommand, self.command_topic, self._basic_command_callback, 10)

    def _register_blackboard_keys(self):
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="goal_tag_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.last_command = None
        self.blackboard.goal_tag_command = None

    def _tag_command_callback(self, msg: TagElement):
        """Process incoming command from UI"""
        if self.is_last_command(msg):
            return

        try:
            tag_command = TagCommand(msg.id, msg.pose, msg.offset, msg.orientation_mode)
            self.blackboard.goal_tag_command = tag_command
            last_command = SimpleCommand( CommandID.MOVE_TO_TAG, msg.pose.header.stamp)
            self.blackboard.last_command = last_command
            self.logger.info(f"Received command for tag {msg.id}")
        except Exception as e:
            self.logger.error(f"Error processing command: {e}")

    def _basic_command_callback(self, msg: BasicCommand):
        if self.is_last_command(msg):
            return

        last_command = SimpleCommand(msg.command_id, msg.header.stamp)
        self.blackboard.last_command = last_command

        self.logger.info(f"Received {msg.command_id} command")

    def is_last_command(self, msg) -> bool:
        if self.blackboard.last_command is None:
            return False

        stamp = self._extract_timestamp(msg)
        if stamp is None:
            self.logger.warning(f"Could not extract timestamp from message of type {type(msg).__name__}")
            return False

        last_stamp = self.blackboard.last_command.stamp
        return stamp.sec == last_stamp.sec and stamp.nanosec == last_stamp.nanosec

    def _extract_timestamp(self, msg):
        """Extract timestamp from various message types"""
        if hasattr(msg, 'header'):
            return msg.header.stamp
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'header'):
            return msg.pose.header.stamp
        elif hasattr(msg, 'stamp'):
            return msg.stamp
