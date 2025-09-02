from typing import Optional, List

import py_trees
import rclpy
from fault_detector_msgs.msg import ComplexCommand, BasicCommand
from fault_detector_spot.behaviour_tree.QOS_PROFILES import COMMAND_QOS
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import GenericCommand
from fault_detector_spot.behaviour_tree.commands.manipulator_move_command import ManipulatorMoveCommand
from fault_detector_spot.behaviour_tree.commands.manipulator_tag_command import ManipulatorTagCommand
from fault_detector_spot.behaviour_tree.commands.simple_command import SimpleCommand
from fault_detector_spot.behaviour_tree.commands.timer_command import TimerCommand


class CommandSubscriber(py_trees.behaviour.Behaviour):
    """
    Subscribes to UI commands and updates the blackboard with the latest received command.
    """

    def __init__(
            self,
            name: str = "CommandSubscriber",
            complex_command_topic: str = "fault_detector/commands/complex_command",
            command_topic: str = "fault_detector/commands/basic_command"
    ):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.complex_command_topic = complex_command_topic
        self.command_topic = command_topic
        self.blackboard = None
        self.received_command: Optional[SimpleCommand] = None
        self._combination_command_builders = {
            CommandID.SCAN_ALL_IN_RANGE: self._scan_all_in_range,
            CommandID.MOVE_ARM_TO_TAG: self._move_to_tag,
            CommandID.MOVE_ARM_TO_TAG_AND_WAIT: self._move_to_tag_and_wait,
            CommandID.MOVE_ARM_RELATIVE: self._move_arm_command_with_offset,
            CommandID.ESTOP_STATE: self._return_to_estop_state,
        }
        self.pending_msgs = []
        self.last_received_time = None
        self.process_delay_sec = 0.05

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
            self._create_ui_subscribers()
            self.blackboard = self.attach_blackboard_client()
            self._register_blackboard_keys()
        except KeyError as e:
            self.logger.error(f"Could not retrieve node from kwargs: {e}")

    def update(self) -> py_trees.common.Status:
        if not self.pending_msgs:
            self.feedback_message = "No commands received yet"
            return py_trees.common.Status.SUCCESS

            # Sort by timestamp
        self.pending_msgs.sort(key=lambda x: (x[0].sec, x[0].nanosec))

        for stamp, msg in self.pending_msgs:
            self.fire_command(msg)

        self.pending_msgs.clear()
        self.feedback_message = f"Processed {len(self.pending_msgs)} commands"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Terminating with status {new_status}")

    def _create_ui_subscribers(self):
        self.node.create_subscription(
            ComplexCommand,
            self.complex_command_topic,
            self.append_command_to_buffer,
            COMMAND_QOS
        )
        self.node.create_subscription(
            BasicCommand,
            self.command_topic,
            self.append_command_to_buffer,
            COMMAND_QOS
        )

    def _register_blackboard_keys(self):
        self.blackboard.register_key(
            key="command_buffer", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="estop_flag", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="reachable_tags", access=py_trees.common.Access.READ
        )
        self.blackboard.command_buffer = []
        self.blackboard.estop_flag = False

    def append_command_to_buffer(self, msg):
        if isinstance(msg, BasicCommand):
            if msg.command_id == CommandID.EMERGENCY_CANCEL:
                self.trigger_estop()
                self.logger.warning("ESTOP triggered imediately.")
                return
        stamp = self._extract_timestamp(msg)
        if stamp is None:
            self.logger.warning("Command without timestamp ignored.")
            return
        self.pending_msgs.append((stamp, msg))

    def fire_command(self, msg: [ComplexCommand, BasicCommand]):
        if self.is_last_command(msg):
            return
        if isinstance(msg, BasicCommand):
            self.received_command = SimpleCommand(msg.command_id, msg.header.stamp)
            self.fire_basic_command(msg)
        elif isinstance(msg, ComplexCommand):
            self.received_command = SimpleCommand(msg.command.command_id, msg.command.header.stamp)
            self.fire_complex_command_sequence(msg)
        else:
            self.logger.error(f"Unknown message type: {type(msg)}")
            return

    def fire_basic_command(self, msg: BasicCommand):
        if msg.command_id == CommandID.EMERGENCY_CANCEL:
            self.trigger_estop()
            return
        if msg.command_id in self._combination_command_builders:
            command_sequence = self._combination_command_builders.get(msg.command_id)(msg)
            self.blackboard.command_buffer.extend(command_sequence)
        else:
            self.blackboard.command_buffer.append(self.received_command)
        self.logger.info(f"Received {msg.command_id} command")

    def fire_complex_command_sequence(self, msg: ComplexCommand) -> List[SimpleCommand]:
        command_id = msg.command.command_id
        if command_id in self._combination_command_builders:
            command_sequence = self._combination_command_builders.get(command_id)(msg)
            self.blackboard.command_buffer.extend(command_sequence)
        else:
            generic_command = self.complex_message_to_generic_command(msg)
            self.blackboard.command_buffer.append(generic_command)
        self.logger.info(f"Received {command_id} command")

    def complex_message_to_generic_command(self, msg: ComplexCommand) -> GenericCommand:
        generic_command = GenericCommand(command_id=msg.command.command_id, stamp=msg.command.header.stamp)
        generic_command.duration = msg.wait_time
        if msg.tag is not None:
            generic_command.tag_id = msg.tag.id
            generic_command.goal_pose = msg.tag.pose
        else:
            generic_command.goal_pose = msg.offset
        generic_command.orientation_mode = msg.orientation_mode
        generic_command.offset = msg.offset
        generic_command.map_name = msg.map_name
        generic_command.landmark_name = msg.landmark_name
        return generic_command

    def is_estop_command(self, command) -> bool:
        return command.command_id == CommandID.EMERGENCY_CANCEL

    def trigger_estop(self):
        self.blackboard.command_buffer.clear()
        self.pending_msgs = []
        self.blackboard.estop_flag = True
        self.blackboard.command_buffer.append(
            SimpleCommand(command_id=CommandID.EMERGENCY_CANCEL, stamp=self._create_command_stamp()))
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
        if hasattr(msg, 'command'):
            return msg.command.header.stamp
        elif hasattr(msg, 'header'):
            return msg.header.stamp
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'header'):
            return msg.pose.header.stamp
        elif hasattr(msg, 'stamp'):
            return msg.stamp
        return None

    def _create_command_stamp(self):
        return self.node.get_clock().now().to_msg()

    ### Command builders for complex commands ###

    def _move_arm_command_with_offset(self, msg: ComplexCommand):
        command = ManipulatorMoveCommand(msg.command.command_id, self._create_command_stamp(), msg.offset)
        return [command]

    def _move_to_tag(self, msg: ComplexCommand) -> List[SimpleCommand]:
        command = ManipulatorTagCommand(CommandID.MOVE_ARM_TO_TAG, self._create_command_stamp(), msg.tag.pose,
                                        msg.tag.id, msg.offset, msg.orientation_mode)
        return [command]

    ### Command builders for combination commands ###

    def _move_to_tag_and_wait(self, msg: ComplexCommand) -> List[SimpleCommand]:
        command = self._move_to_tag(msg)
        if msg.wait_time <= 0.0:
            return command
        command.append(TimerCommand(CommandID.WAIT_TIME, self._create_command_stamp(), msg.wait_time))
        return command

    def _scan_all_in_range(self, msg: ComplexCommand) -> List[SimpleCommand]:
        """
        For each tag in blackboard.reachable_tags (id→TagElement),
        MOVE_ARM_TO_TAG, WAIT_TIME, STOW_ARM.
        """
        tags = self.blackboard.reachable_tags
        if not tags:
            return []

        commands: List[SimpleCommand] = []
        for tag_id, tag in sorted(tags.items()):
            msg.tag = tag
            commands.extend(self._move_to_tag_and_wait(msg))
            commands.append(SimpleCommand(
                CommandID.STOW_ARM, self._create_command_stamp()
            ))
            commands.append(TimerCommand(
                CommandID.WAIT_TIME, self._create_command_stamp(), 0.2
            ))

        return commands

    def _return_to_estop_state(self, msg: ComplexCommand) -> List[SimpleCommand]:
        commands: List[SimpleCommand] = []
        commands.append(SimpleCommand(CommandID.STOP_BASE, self._create_command_stamp()))
        commands.append(SimpleCommand(CommandID.STOW_ARM, self._create_command_stamp()))
        commands.append(SimpleCommand(CommandID.CLOSE_GRIPPER, self._create_command_stamp()))
        return commands
