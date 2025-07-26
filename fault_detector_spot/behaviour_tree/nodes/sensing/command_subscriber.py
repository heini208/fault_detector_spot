import py_trees
import rclpy
from fault_detector_msgs.msg import TagElement, BasicCommand
from fault_detector_spot.behaviour_tree.Tag_Command import TagCommand
from std_msgs.msg import Header
from typing import Optional
from fault_detector_spot.behaviour_tree.command_ids import CommandID
from fault_detector_spot.behaviour_tree.simple_command import SimpleCommand
from fault_detector_spot.behaviour_tree.timer_command import TimerCommand


class CommandSubscriber(py_trees.behaviour.Behaviour):
    """
    Subscribes to UI commands and updates the blackboard with the latest received command.
    """

    def __init__(
        self,
        name: str = "CommandSubscriber",
        move_topic: str = "fault_detector/commands/move_to_tag",
        command_topic: str = "fault_detector/commands/basic_command"
    ):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.move_to_tag_topic = move_topic
        self.command_topic = command_topic
        self.blackboard = None
        self.received_command: Optional[SimpleCommand] = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
            self._create_ui_subscribers()
            self.blackboard = self.attach_blackboard_client()
            self._register_blackboard_keys()
        except KeyError as e:
            self.logger.error(f"Could not retrieve node from kwargs: {e}")

    def update(self) -> py_trees.common.Status:
        if self.received_command is not None:
            self.feedback_message = f"Last received command: {self.received_command.id}"
        else:
            self.feedback_message = "No commands received yet"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Terminating with status {new_status}")

    def _create_ui_subscribers(self):
        self.node.create_subscription(
            TagElement,
            self.move_to_tag_topic,
            self._tag_command_callback,
            10
        )
        self.node.create_subscription(
            BasicCommand,
            self.command_topic,
            self._basic_command_callback,
            10
        )

    def _register_blackboard_keys(self):
        self.blackboard.register_key(
            key="command_buffer", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="goal_tag_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="estop_flag", access=py_trees.common.Access.WRITE
        )
        self.blackboard.command_buffer = []
        self.blackboard.goal_tag_command = None
        self.blackboard.estop_flag = False

    def _tag_command_callback(self, msg: TagElement):
        if self.is_last_command(msg):
            return
        try:
            tag_command = TagCommand(msg.id, msg.pose, msg.offset, msg.orientation_mode)
            self.blackboard.goal_tag_command = tag_command
            self.received_command = SimpleCommand(CommandID.MOVE_TO_TAG, msg.pose.header.stamp)
            self.blackboard.command_buffer.append(self.received_command)
            self.logger.info(f"Received command for tag {msg.id}")

            if msg.duration != 0.0:
                timer_command = TimerCommand(CommandID.WAIT_TIME, msg.pose.header.stamp, msg.duration)
                self.blackboard.command_buffer.append(timer_command)

        except Exception as e:
            self.logger.error(f"Error processing command: {e}")

    def _basic_command_callback(self, msg: BasicCommand):
        if self.is_last_command(msg):
            return
        self.received_command = SimpleCommand(msg.command_id, msg.header.stamp)

        if self.is_estop_command(self.received_command):
            self.trigger_estop()
            return

        self.blackboard.command_buffer.append(self.received_command)
        self.logger.info(f"Received {msg.command_id} command")

    def is_estop_command(self, command) -> bool:
        return command.id == CommandID.EMERGENCY_CANCEL

    def trigger_estop(self):
        self.blackboard.command_buffer.clear()
        self.blackboard.estop_flag = True
        self.blackboard.command_buffer.append(self.received_command)
        self.blackboard.goal_tag_command = None
        self.logger.info("Emergency stop command received, clearing command buffer")

    def is_last_command(self, msg) -> bool:
        if self.received_command is None:
            return False
        stamp = self._extract_timestamp(msg)
        if stamp is None:
            self.logger.warning(
                f"Could not extract timestamp from message of type {type(msg).__name__}"
            )
            return False
        last_stamp = self.received_command.stamp
        return stamp.sec == last_stamp.sec and stamp.nanosec == last_stamp.nanosec

    def _extract_timestamp(self, msg):
        if hasattr(msg, 'header'):
            return msg.header.stamp
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'header'):
            return msg.pose.header.stamp
        elif hasattr(msg, 'stamp'):
            return msg.stamp
        return None
