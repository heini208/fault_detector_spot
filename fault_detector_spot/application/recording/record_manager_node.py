#!/usr/bin/env python3
"""Record and replay accepted semantic command requests."""

from collections import deque
import json
import os
from typing import Deque, List

import rclpy
from fault_detector_msgs.msg import (
    CommandRecordControl,
    CommandRequest as CommandRequestMessage,
    CommandStatus,
    ComplexCommand,
    StringArray,
)
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from std_msgs.msg import Bool

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
    command_request_to_message,
)
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_recording_root,
)
from fault_detector_spot.shared.ros.qos_profiles import (
    COMMAND_REQUEST_QOS,
    LATCHED_QOS,
)


def serialize_recorded_command(command: ComplexCommand) -> dict:
    """Serialize one semantic command without transient correlation data."""
    if not isinstance(command, ComplexCommand):
        raise TypeError("Recorded command must be a ComplexCommand")
    data = message_to_ordereddict(command)
    data["command"]["request_id"] = ""
    data["command"]["header"]["stamp"] = {
        "sec": 0,
        "nanosec": 0,
    }
    return data


def deserialize_recorded_command(data: dict) -> ComplexCommand:
    """Restore one semantic command payload from recording data."""
    if not isinstance(data, dict):
        raise TypeError("Recorded command data must be an object")
    command = ComplexCommand()
    set_message_fields(command, data)
    command.command.request_id = ""
    return command


def deserialize_recording(document) -> List[ComplexCommand]:
    """Load one semantic recording document."""
    if not isinstance(document, dict):
        raise ValueError("Recording must be an object")
    entries = document.get("commands")
    if not isinstance(entries, list):
        raise ValueError("Recording commands must be a list")
    return [deserialize_recorded_command(entry) for entry in entries]


class RecordManager(Node):
    """Persist accepted commands and coordinate sequential playback."""

    def __init__(self):
        super().__init__("record_manager")
        self.declare_parameter(
            "recording.root",
            str(default_recording_root()),
        )
        configured_root = str(
            self.get_parameter("recording.root").value
        ).strip()
        self.recordings_dir = os.path.expanduser(
            configured_root or str(default_recording_root())
        )
        os.makedirs(self.recordings_dir, exist_ok=True)
        self.recording = False
        self.current_name = None
        self.temp_data: List[dict] = []
        self._recorded_request_ids = set()
        self._playback_commands: Deque[ComplexCommand] = deque()
        self._playback_request_id = ""
        self._playback_name = ""

        self.list_pub = self.create_publisher(
            StringArray,
            "fault_detector/recordings_list",
            LATCHED_QOS,
        )
        self.playback_state_pub = self.create_publisher(
            Bool,
            "fault_detector/playback_state",
            LATCHED_QOS,
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
            self.handle_playback_status,
            10,
        )
        self.publish_recordings_list()

    def handle_control(self, message: CommandRecordControl):
        mode = message.mode.lower()
        name = message.name.strip()
        if mode == "start":
            self.start_recording(name)
        elif mode == "stop":
            self.stop_recording()
        elif mode == "play":
            self.play_recording(name)
        elif mode == "delete":
            self.delete_recording(name)

    def start_recording(self, name: str):
        if not name:
            self.get_logger().warning(
                "Recording name is empty, ignoring."
            )
            return
        self.delete_recording(name)
        self.recording = True
        self.current_name = name
        self.temp_data.clear()
        self._recorded_request_ids.clear()
        self.get_logger().info(f"Started recording: {name}")

    def stop_recording(self):
        if not self.recording:
            return
        file_path = self._recording_path(self.current_name)
        with open(file_path, "w") as file:
            json.dump({"commands": self.temp_data}, file, indent=2)
        self.get_logger().info(f"Saved recording: {file_path}")
        self.recording = False
        self.current_name = None
        self._recorded_request_ids.clear()
        self.publish_recordings_list()

    def capture_request(self, message: CommandRequestMessage) -> bool:
        if not self.recording:
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
        self.temp_data.append(
            serialize_recorded_command(request.command)
        )
        return True

    def play_recording(self, name: str):
        if self._playback_request_id or self._playback_commands:
            self.get_logger().warning("Playback is already active.")
            return
        file_path = self._recording_path(name)
        if not os.path.exists(file_path):
            self.get_logger().warning(
                f"Recording not found: {name}"
            )
            return
        try:
            with open(file_path, "r") as file:
                commands = deserialize_recording(json.load(file))
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
            if message.buffered_command_count > 0:
                return True
            self._playback_request_id = ""
            self._dispatch_next_playback_command()
            return True
        if message.state in (
            CommandStatus.STATE_FAILED,
            CommandStatus.STATE_CANCELLED,
        ):
            detail = message.detail or "command did not succeed"
            self._finish_playback(
                "Playback stopped after "
                f"{message.command_id}: {detail}",
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
            recording_policy=(
                RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
            ),
        )
        self._playback_request_id = request.request_id
        self.command_submission_pub.publish(
            command_request_to_message(request)
        )

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
        file_path = self._recording_path(name)
        if os.path.exists(file_path):
            os.remove(file_path)
            self.get_logger().info(
                f"Deleted recording: {name}"
            )
            self.publish_recordings_list()

    def publish_recordings_list(self):
        files = [
            file_name[:-5]
            for file_name in os.listdir(self.recordings_dir)
            if file_name.endswith(".json")
        ]
        self.list_pub.publish(
            StringArray(names=sorted(files))
        )

    def _recording_path(self, name):
        return os.path.join(
            self.recordings_dir,
            f"{name}.json",
        )


def main(args=None):
    rclpy.init(args=args)
    node = RecordManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
