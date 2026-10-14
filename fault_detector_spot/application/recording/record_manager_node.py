#!/usr/bin/env python3
"""Record and replay accepted semantic command requests."""

from collections import deque
from copy import deepcopy
from typing import Deque, List

import rclpy
from fault_detector_msgs.msg import (
    CommandRecordControl,
    CommandRequest as CommandRequestMessage,
    CommandStatus,
    StringArray,
)
from rclpy.node import Node
from std_msgs.msg import Bool

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.recording.recording_repository import RecordingRepository
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
    command_request_to_message,
)
from fault_detector_spot.shared.persistence.runtime_paths import default_recording_root
from fault_detector_spot.shared.ros.qos_profiles import (
    COMMAND_REQUEST_QOS,
    LATCHED_QOS,
)


class RecordManager(Node):
    """Persist accepted commands and coordinate sequential playback."""

    def __init__(self):
        super().__init__("record_manager")
        self.declare_parameter(
            "recording.root",
            str(default_recording_root()),
        )
        configured_root = str(self.get_parameter("recording.root").value).strip()
        self.repository = RecordingRepository(
            configured_root or default_recording_root()
        )
        self.recording = False
        self.recording_paused = False
        self.current_name = None
        self.temp_data: List[SemanticCommand] = []
        self._recorded_request_ids = set()
        self._recording_commands = {}
        self._recording_results = {}
        self._recording_stop_requested = False
        self._playback_commands: Deque[SemanticCommand] = deque()
        self._playback_request_id = ""
        self._playback_name = ""

        self.list_pub = self.create_publisher(
            StringArray,
            "fault_detector/recordings_list",
            LATCHED_QOS,
        )
        self.recording_state_pub = self.create_publisher(
            Bool,
            "fault_detector/recording_state",
            LATCHED_QOS,
        )
        self.playback_state_pub = self.create_publisher(
            Bool,
            "fault_detector/playback_state",
            LATCHED_QOS,
        )
        self.recording_paused_pub = self.create_publisher(
            Bool,
            "fault_detector/recording_paused",
            LATCHED_QOS,
        )
        self.recording_finishing_pub = self.create_publisher(
            Bool, "fault_detector/recording_finishing", LATCHED_QOS,
        )
        self.command_submission_pub = self.create_publisher(
            CommandRequestMessage,
            "fault_detector/_internal/commands/submit",
            COMMAND_REQUEST_QOS,
        )

        self.create_subscription(
            CommandRecordControl,
            "fault_detector/record_control",
            self.handle_control,
            10,
        )
        self.create_subscription(
            CommandRequestMessage,
            "fault_detector/_internal/commands/accepted",
            self.capture_request,
            COMMAND_REQUEST_QOS,
        )
        self.create_subscription(
            CommandStatus,
            "fault_detector/_internal/commands/status",
            self.handle_command_status,
            COMMAND_REQUEST_QOS,
        )
        self.publish_recordings_list()
        self.publish_recording_state()

    def handle_control(self, message: CommandRecordControl):
        mode = message.mode.lower()
        name = message.name.strip()
        try:
            if mode == "start":
                self.start_recording(name)
            elif mode == "stop":
                self.stop_recording()
            elif mode == "play":
                self.play_recording(name)
            elif mode == "delete":
                self.delete_recording(name)
            elif mode in {"pause", "resume"}:
                self.set_recording_paused(mode == "pause")
        except (OSError, TypeError, ValueError) as exception:
            self.get_logger().error(f"Recording {mode} request failed: {exception}")
        finally:
            if mode in {"start", "stop", "pause", "resume"}:
                self.publish_recording_state()

    def publish_recording_state(self):
        """Publish actual state, including after a rejected start or failed save."""
        self.recording_state_pub.publish(Bool(data=self.recording))
        self.recording_paused_pub.publish(Bool(data=self.recording_paused))
        self.recording_finishing_pub.publish(Bool(
            data=self._recording_stop_requested and self._has_pending_commands()
        ))

    def _has_pending_commands(self):
        return any(
            request_id not in self._recording_results
            for request_id in self._recording_commands
        )

    def set_recording_paused(self, paused: bool):
        """Exclude accepted commands without affecting their execution."""
        if not self.recording or self._recording_stop_requested:
            self.get_logger().warning("Cannot pause or resume an inactive or finishing recording.")
            return
        if self.recording_paused != paused:
            self.recording_paused = paused
            action = "Paused" if paused else "Resumed"
            self.get_logger().info(f"{action} command capture: {self.current_name}")

    def start_recording(self, name: str):
        if self.recording:
            self.get_logger().warning("Recording is already active.")
            return
        if not name:
            self.get_logger().warning("Recording name is empty, ignoring.")
            return
        self.delete_recording(name)
        self.recording = True
        self.recording_paused = False
        self.current_name = name
        self.temp_data.clear()
        self._recorded_request_ids.clear()
        self._recording_commands.clear()
        self._recording_results.clear()
        self._recording_stop_requested = False
        self.get_logger().info(f"Started recording: {name}")

    def stop_recording(self):
        if not self.recording:
            return
        self._recording_stop_requested = True
        if self._has_pending_commands():
            self.get_logger().info("Waiting for command outcomes before saving recording.")
            return
        self.repository.save(self.current_name, self.temp_data)
        self.get_logger().info(f"Saved recording: {self.current_name}")
        self.recording = False
        self.recording_paused = False
        self.current_name = None
        self._recorded_request_ids.clear()
        self._recording_commands.clear()
        self._recording_results.clear()
        self._recording_stop_requested = False
        self.publish_recordings_list()

    def capture_request(self, message: CommandRequestMessage) -> bool:
        if not self.recording or self._recording_stop_requested:
            return False
        try:
            request = command_request_from_message(message)
        except (TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Rejected accepted command request: {exception}"
            )
            return False
        if (
            request.recording_policy
            is not RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ):
            return False
        if request.request_id in self._recorded_request_ids:
            return False
        self._recorded_request_ids.add(request.request_id)
        if self.recording_paused:
            self.get_logger().info(
                f"Excluded command {request.command.command_id.value} "
                f"({request.request_id}) from {self.current_name}: recording paused"
            )
            return False
        self._recording_commands[request.request_id] = deepcopy(request.command)
        self._refresh_successful_commands()
        return True

    def _refresh_successful_commands(self):
        # Dict insertion order is acceptance order, irrespective of completion order.
        self.temp_data = [
            command for request_id, command in self._recording_commands.items()
            if self._recording_results.get(request_id) == CommandStatus.STATE_SUCCEEDED
        ]

    def handle_command_status(self, message: CommandStatus):
        self.handle_recording_status(message)
        self.handle_playback_status(message)

    def handle_recording_status(self, message: CommandStatus) -> bool:
        if not self.recording or not message.request_id or message.state not in (
            CommandStatus.STATE_SUCCEEDED,
            CommandStatus.STATE_FAILED,
            CommandStatus.STATE_CANCELLED,
        ):
            return False
        if message.request_id in self._recording_results:
            return False
        # Status and acceptance use separate topics; either callback may arrive first.
        self._recording_results[message.request_id] = message.state
        if message.state != CommandStatus.STATE_SUCCEEDED:
            self.get_logger().info(
                f"Command {message.request_id} not retained in recording: "
                f"state={message.state}; {message.detail or 'command did not succeed'}"
            )
        if message.request_id not in self._recording_commands:
            return False
        self._refresh_successful_commands()
        if self._recording_stop_requested and not self._has_pending_commands():
            # Use the same error handling as an explicit stop; unsaved data survives.
            self.handle_control(CommandRecordControl(mode="stop"))
        return True

    def play_recording(self, name: str):
        if self._playback_request_id or self._playback_commands:
            self.get_logger().warning("Playback is already active.")
            return
        try:
            commands = self.repository.load(name)
        except (OSError, TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Could not load recording {name}: {exception}"
            )
            return
        self._playback_commands = deque(commands)
        self._playback_name = name
        self.playback_state_pub.publish(Bool(data=True))
        self.get_logger().info(f"Playing back: {name}")
        self._dispatch_next_playback_command()

    def handle_playback_status(self, message: CommandStatus) -> bool:
        if (
            not self._playback_request_id
            or message.request_id != self._playback_request_id
        ):
            return False
        if message.state == CommandStatus.STATE_SUCCEEDED:
            self._playback_request_id = ""
            self._dispatch_next_playback_command()
            return True
        if message.state in (
            CommandStatus.STATE_FAILED,
            CommandStatus.STATE_CANCELLED,
        ):
            detail = message.detail or "command did not succeed"
            self._finish_playback(
                f"Playback stopped after {message.command_id}: {detail}",
                failed=True,
            )
            return True
        return message.state == CommandStatus.STATE_RUNNING

    def _dispatch_next_playback_command(self):
        if not self._playback_commands:
            self._finish_playback("Playback finished.")
            return
        command = self._playback_commands.popleft()
        request = CommandRequest.create(
            command=command,
            client_id="record_manager",
            origin=CommandOrigin.PLAYBACK,
            recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
        )
        self._playback_request_id = request.request_id
        self.command_submission_pub.publish(command_request_to_message(request))

    def _finish_playback(self, message, failed=False):
        was_active = bool(
            self._playback_name
            or self._playback_request_id
            or self._playback_commands
        )
        self._playback_commands.clear()
        self._playback_request_id = ""
        self._playback_name = ""
        if was_active:
            self.playback_state_pub.publish(Bool(data=False))
        if failed:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def delete_recording(self, name: str):
        if self.repository.delete(name):
            self.get_logger().info(f"Deleted recording: {name}")
            self.publish_recordings_list()

    def publish_recordings_list(self):
        self.list_pub.publish(StringArray(names=self.repository.list_names()))


def main(args=None):
    rclpy.init(args=args)
    node = RecordManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
