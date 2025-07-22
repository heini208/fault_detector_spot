# fault_detector_spot/behaviour_tree/nodes/manipulator_command_subscriber.py

import py_trees
import rclpy
from fault_detector_msgs.msg import TagElement
from std_msgs.msg import Header
from typing import Optional


class ManipulatorCommandSubscriber(py_trees.behaviour.Behaviour):
    """
    Subscribes to UI commands
    and updates the blackboard for use by manipulator control nodes.
    """

    def __init__(self,
                 name: str = "ManipulatorCommandSubscriber",
                 move_topic: str = "fault_detector/commands/move_to_tag",
                 stow_topic: str = "fault_detector/commands/stow_arm",
                 stand_topic: str = "fault_detector/commands/stand_up",
                 ready_topic: str = "fault_detector/commands/ready_arm"):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.move_to_tag_topic = move_topic
        self.stow_topic        = stow_topic
        self.stand_topic       = stand_topic
        self.ready_topic       = ready_topic
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
        if self.blackboard.last_command_id is not None:
            self.feedback_message = f"Last command: tag {self.blackboard.last_command_id}"
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
        # Stow arm / cancel
        self.node.create_subscription(
            Header,
            self.stow_topic,
            self._stow_arm_command_callback,
            10
        )
        # Stand up
        self.node.create_subscription(
            Header,
            self.stand_topic,
            self._stand_up_command_callback,
            10
        )
        # Ready arm
        self.node.create_subscription(
            Header,
            self.ready_topic,
            self._ready_arm_command_callback,
            10
        )

    def _register_blackboard_keys(self):
        self.blackboard.register_key(
            key="goal_tag_id", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="goal_tag_pose", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_command_id", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_command_stamp", access=py_trees.common.Access.WRITE
        )

        self.blackboard.last_command_id = None
        self.blackboard.last_command_stamp = None
        self.blackboard.goal_tag_id = None
        self.blackboard.goal_tag_pose = None

    def _tag_command_callback(self, msg: TagElement):
        """Process incoming command from UI"""
        if self.is_last_command(msg):
            return

        try:
            self.blackboard.goal_tag_id = msg.id
            self.blackboard.goal_tag_pose = msg.pose
            self.blackboard.last_command_id = "move_to_tag"
            self.blackboard.last_command_stamp = msg.pose.header.stamp

            self.logger.info(f"Received command for tag {msg.id}")
        except Exception as e:
            self.logger.error(f"Error processing command: {e}")

    def _stow_arm_command_callback(self, msg: Header):
        if self.is_last_command(msg):
            return
        self.blackboard.last_command_id = "stow_arm"
        self.blackboard.last_command_stamp = msg.stamp
        self.logger.info("Received stow_arm command")

    def _ready_arm_command_callback(self, msg: Header):
        if self.is_last_command(msg.stamp):
            return
        self.blackboard.last_command_id = "ready_arm"
        self.blackboard.last_command_stamp = msg.stamp
        self.logger.info("Received ready_arm command")

    def _stand_up_command_callback(self, msg: Header):
        if self.is_last_command(msg):
            return
        self.blackboard.last_command_id = "stand_up"
        self.blackboard.last_command_stamp = msg.stamp
        self.logger.info("Received stand_up command")

    def is_last_command(self, msg) -> bool:
        if self.blackboard.last_command_stamp is None:
            return False

        stamp = self._extract_timestamp(msg)
        if stamp is None:
            self.logger.warning(f"Could not extract timestamp from message of type {type(msg).__name__}")
            return False

        last_stamp = self.blackboard.last_command_stamp
        return stamp.sec == last_stamp.sec and stamp.nanosec == last_stamp.nanosec

    def _extract_timestamp(self, msg):
        """Extract timestamp from various message types"""
        if hasattr(msg, 'header'):
            return msg.header.stamp
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'header'):
            return msg.pose.header.stamp
        elif hasattr(msg, 'stamp'):
            return msg.stamp
