from dataclasses import replace
from typing import List, Optional

import py_trees
import rclpy
from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    CommandStatus,
)

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
    SemanticTag,
)
from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.application.behaviour_tree.commands.wait_command import (
    WaitCommand,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    stamped_pose_from_message,
    stamped_pose_to_message,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import (
    ManipulatorMoveRelativeCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.navigation.commands.base_move_relative_command import (
    BaseMoveRelativeCommand,
)
from fault_detector_spot.mapping.commands.map_command import MapCommand
from fault_detector_spot.navigation.commands.waypoint_command import (
    WaypointCommand,
)
from fault_detector_spot.navigation.commands.base_to_tag_command import (
    BaseToTagCommand,
)
from fault_detector_spot.shared.ros.qos_profiles import COMMAND_REQUEST_QOS


class CommandSubscriber(py_trees.behaviour.Behaviour):
    """Translate controller-dispatched semantic requests into tree commands."""

    def __init__(
        self,
        name: str = "CommandSubscriber",
        request_topic: str = "fault_detector/_internal/commands/request",
        status_topic: str = "fault_detector/_internal/command_status",
    ):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.request_topic = request_topic
        self.status_topic = status_topic
        self.blackboard = None
        self._combination_command_builders = {
            CommandID.SCAN_ALL_IN_RANGE: self._scan_all_in_range,
            CommandID.MOVE_ARM_TO_TAG: self._move_to_tag,
            CommandID.MOVE_ARM_TO_TAG_AND_WAIT: self._move_to_tag_and_wait,
            CommandID.MOVE_ARM_RELATIVE: self._move_arm_command_with_offset,
            CommandID.MOVE_BASE_TO_TAG: self._move_base_to_tag,
            CommandID.MOVE_BASE_RELATIVE: self._move_base_with_offset,
            CommandID.ESTOP_STATE: self._return_to_estop_state,
        }
        self._single_command_builders = {
            CommandID.STOW_ARM: self._simple_command,
            CommandID.READY_ARM: self._simple_command,
            CommandID.STAND_UP: self._simple_command,
            CommandID.TOGGLE_GRIPPER: self._simple_command,
            CommandID.CLOSE_GRIPPER: self._simple_command,
            CommandID.STOP_BASE: self._simple_command,
            CommandID.STOP_MAPPING: self._simple_command,
            CommandID.WAIT_TIME: self._wait_command,
            CommandID.START_SLAM: self._map_command,
            CommandID.START_LOCALIZATION: self._map_command,
            CommandID.SWAP_MAP: self._map_command,
            CommandID.MOVE_TO_WAYPOINT: self._waypoint_command,
            CommandID.MOVE_CLOSE_TO_SURFACE: self._move_close_to_surface,
        }
        self.pending_msgs = []
        self.last_received_time = None
        self.process_delay_sec = 0.05
        self._received_request_ids = set()
        self.request_status_publisher = None
        self.request_subscription = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
            self._create_ui_subscribers()
            self.blackboard = self.attach_blackboard_client()
            self._register_blackboard_keys()
        except KeyError as exception:
            self.logger.error(
                f"Could not retrieve node from kwargs: {exception}"
            )

    def update(self) -> py_trees.common.Status:
        if not self.pending_msgs:
            self.feedback_message = "No commands received yet"
            return py_trees.common.Status.SUCCESS

        self.pending_msgs.sort(
            key=lambda item: (item[0].sec, item[0].nanosec)
        )

        processed_count = 0
        for _, request in self.pending_msgs:
            try:
                command_count = self.fire_request(request)
                processed_count += 1
                detail = (
                    f"BT buffered {command_count} command step(s) "
                    "for execution"
                )
                self._publish_request_status(
                    request,
                    CommandStatus.STATE_RUNNING,
                    detail,
                    buffered_command_count=command_count,
                )
                self.node.get_logger().info(
                    f"{detail} [{request.request_id}]"
                )
            except Exception as exception:
                self.logger.error(f"Rejected command: {exception}")
                self._publish_request_failure(request, str(exception))

        self.pending_msgs.clear()
        self.feedback_message = f"Processed {processed_count} commands"
        return py_trees.common.Status.SUCCESS

    def _create_ui_subscribers(self):
        self.request_status_publisher = self.node.create_publisher(
            CommandStatus,
            self.status_topic,
            10,
        )
        self.request_subscription = self.node.create_subscription(
            CommandRequestMessage,
            self.request_topic,
            self.append_request_to_buffer,
            COMMAND_REQUEST_QOS,
        )

    def _register_blackboard_keys(self):
        self.blackboard.register_key(
            key="command_buffer",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="estop_flag",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="reachable_tags",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.command_buffer = []
        self.blackboard.estop_flag = False

    def append_request_to_buffer(self, message):
        try:
            request = command_request_from_message(message)
            stamp = message.header.stamp
        except (TypeError, ValueError) as exception:
            detail = f"BT rejected command envelope: {exception}"
            self.logger.error(detail)
            self._publish_raw_request_failure(message, detail)
            return

        if request.request_id in self._received_request_ids:
            detail = (
                f"BT rejected duplicate request ID: {request.request_id}"
            )
            self.logger.error(detail)
            self._publish_request_failure(request, detail)
            return

        self._received_request_ids.add(request.request_id)
        command_id = request.command.command_id.value
        detail = f"BT received {command_id} request"
        self._publish_request_status(
            request,
            CommandStatus.STATE_RUNNING,
            detail,
        )
        self.node.get_logger().info(
            f"{detail} [{request.request_id}]"
        )

        if self.is_estop_command(request.command):
            self.trigger_estop(request)
            self.logger.warning("ESTOP triggered immediately.")
            return

        self.pending_msgs.append((stamp, request))

    def fire_request(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> int:
        commands = self.fire_command_sequence(request.command)
        for command in commands:
            command.request_id = request.request_id
            self._apply_request_metadata(command, request)
        if not commands:
            raise ValueError(
                "Command produced no executable behavior-tree commands"
            )
        self.blackboard.command_buffer.extend(commands)
        return len(commands)

    def fire_command_sequence(
        self,
        command: SemanticCommand,
    ) -> List[ExecutionCommand]:
        command_id = command.command_id
        if command_id == CommandID.EXECUTE_PROBE_POINT:
            raise ValueError(
                "execute_probe_point is unavailable until its action "
                "server is installed"
            )
        builder = self._combination_command_builders.get(command_id)
        if builder is not None:
            commands = builder(command)
        else:
            builder = self._single_command_builders.get(command_id)
            if builder is None:
                raise ValueError(
                    f"No execution command builder for {command_id.value}"
                )
            commands = [builder(command)]
        self.logger.info(f"Received {command_id.value} command")
        return commands

    def _simple_command(
        self,
        command: SemanticCommand,
    ) -> ExecutionCommand:
        return ExecutionCommand(
            command_id=command.command_id,
            stamp=self._create_command_stamp(),
        )

    def _wait_command(self, command: SemanticCommand) -> WaitCommand:
        return WaitCommand(
            command_id=CommandID.WAIT_TIME,
            stamp=self._create_command_stamp(),
            duration=command.wait_time,
        )

    def _map_command(self, command: SemanticCommand) -> MapCommand:
        return MapCommand(
            command_id=command.command_id,
            stamp=self._create_command_stamp(),
            map_name=command.map_name,
        )

    def _waypoint_command(
        self,
        command: SemanticCommand,
    ) -> WaypointCommand:
        return WaypointCommand(
            command_id=CommandID.MOVE_TO_WAYPOINT,
            stamp=self._create_command_stamp(),
            map_name=command.map_name,
            waypoint_name=command.waypoint_name,
        )

    def _move_close_to_surface(
        self,
        command: SemanticCommand,
    ) -> MoveCloseToSurfaceCommand:
        return MoveCloseToSurfaceCommand(
            command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
            stamp=self._create_command_stamp(),
            target_surface_distance_m=command.target_surface_distance_m,
            aligned_preapproach_distance_m=(
                command.aligned_preapproach_distance_m
            ),
        )

    def is_estop_command(self, command: SemanticCommand) -> bool:
        return command.command_id == CommandID.EMERGENCY_CANCEL

    def trigger_estop(self, request: CommandRequest[SemanticCommand]):
        self.blackboard.command_buffer.clear()
        self.pending_msgs = []
        self.blackboard.estop_flag = True
        command = ExecutionCommand(
            command_id=CommandID.EMERGENCY_CANCEL,
            stamp=self._create_command_stamp(),
            request_id=request.request_id,
        )
        self._apply_request_metadata(command, request)
        self.blackboard.command_buffer.append(command)
        detail = "BT accepted emergency stop and replaced command buffer"
        self._publish_request_status(
            request,
            CommandStatus.STATE_RUNNING,
            detail,
            buffered_command_count=1,
        )
        self.logger.info(f"{detail} [{request.request_id}]")

    @staticmethod
    def _apply_request_metadata(command, request):
        command.client_id = request.client_id
        command.context_id = request.context_id
        command.origin = request.origin
        command.recording_policy = request.recording_policy

    def _publish_request_status(
        self,
        request,
        state,
        detail,
        buffered_command_count=0,
    ):
        if self.request_status_publisher is None:
            return
        message = CommandStatus()
        message.header.stamp = self._create_command_stamp()
        message.request_id = request.request_id
        message.command_id = request.command.command_id.value
        message.state = state
        message.detail = detail
        message.buffered_command_count = int(buffered_command_count)
        self.request_status_publisher.publish(message)

    def _publish_request_failure(self, request, detail):
        self._publish_request_status(
            request,
            CommandStatus.STATE_FAILED,
            detail,
        )

    def _publish_raw_request_failure(self, message, detail):
        if self.request_status_publisher is None:
            return
        request_id = getattr(message, "request_id", "").strip()
        if not request_id:
            return
        status = CommandStatus()
        status.header.stamp = self._create_command_stamp()
        status.request_id = request_id
        payload = getattr(message, "payload", None)
        status.command_id = getattr(payload, "command_id", "")
        status.state = CommandStatus.STATE_FAILED
        status.detail = detail
        status.buffered_command_count = 0
        self.request_status_publisher.publish(status)

    def _create_command_stamp(self):
        return self.node.get_clock().now().to_msg()

    def _move_arm_command_with_offset(
        self,
        command: SemanticCommand,
    ):
        return [
            ManipulatorMoveRelativeCommand(
                command.command_id,
                self._create_command_stamp(),
                stamped_pose_to_message(command.offset),
            )
        ]

    def _move_to_tag(
        self,
        command: SemanticCommand,
    ) -> List[ExecutionCommand]:
        tag = self._required_tag(command)
        if not command.motion_sensor_id:
            raise ValueError(
                "Move-to-tag command is missing active sensor geometry"
            )
        return [
            ManipulatorToTagCommand(
                CommandID.MOVE_ARM_TO_TAG,
                self._create_command_stamp(),
                stamped_pose_to_message(tag.pose),
                tag.id,
                stamped_pose_to_message(command.offset),
                command.orientation_mode,
                motion_sensor_id=command.motion_sensor_id,
            )
        ]

    def _move_base_to_tag(self, command: SemanticCommand):
        tag = self._required_tag(command)
        return [
            BaseToTagCommand(
                command_id=CommandID.MOVE_BASE_TO_TAG,
                stamp=self._create_command_stamp(),
                tag_pose=stamped_pose_to_message(tag.pose),
                tag_id=tag.id,
                offset=stamped_pose_to_message(command.offset),
            )
        ]

    def _move_base_with_offset(self, command: SemanticCommand):
        return [
            BaseMoveRelativeCommand(
                command_id=CommandID.MOVE_BASE_RELATIVE,
                stamp=self._create_command_stamp(),
                offset=stamped_pose_to_message(command.offset),
            )
        ]

    def _move_to_tag_and_wait(
        self,
        command: SemanticCommand,
    ) -> List[ExecutionCommand]:
        commands = self._move_to_tag(command)
        if command.wait_time <= 0.0:
            return commands
        commands.append(
            WaitCommand(
                CommandID.WAIT_TIME,
                self._create_command_stamp(),
                command.wait_time,
            )
        )
        return commands

    def _scan_all_in_range(
        self,
        command: SemanticCommand,
    ) -> List[ExecutionCommand]:
        tags = self.blackboard.reachable_tags
        if not tags:
            return []

        commands: List[ExecutionCommand] = []
        for _, tag in sorted(tags.items()):
            scan_command = replace(
                command,
                tag=SemanticTag(
                    id=int(tag.id),
                    pose=stamped_pose_from_message(tag.pose),
                ),
            )
            commands.extend(self._move_to_tag_and_wait(scan_command))
            commands.append(
                ExecutionCommand(
                    CommandID.STOW_ARM,
                    self._create_command_stamp(),
                )
            )
            commands.append(
                WaitCommand(
                    CommandID.WAIT_TIME,
                    self._create_command_stamp(),
                    0.2,
                )
            )
        return commands

    def _return_to_estop_state(
        self,
        command: SemanticCommand,
    ) -> List[ExecutionCommand]:
        return [
            ExecutionCommand(
                CommandID.STOP_BASE,
                self._create_command_stamp(),
            ),
            ExecutionCommand(
                CommandID.STOW_ARM,
                self._create_command_stamp(),
            ),
            ExecutionCommand(
                CommandID.CLOSE_GRIPPER,
                self._create_command_stamp(),
            ),
        ]

    @staticmethod
    def _required_tag(command: SemanticCommand) -> SemanticTag:
        if command.tag is None:
            raise ValueError(
                f"{command.command_id.value} requires a tag target"
            )
        return command.tag
