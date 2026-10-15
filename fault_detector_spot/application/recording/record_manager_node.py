#!/usr/bin/env python3
"""Record and replay accepted semantic command requests."""

from collections import deque
import json
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
from std_msgs.msg import Bool, String, UInt32
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.recording.recorded_step import (
    RecordedStep, RETRYABLE_COMMANDS, validate_retries,
)

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
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
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
        self._last_recording_request_id = ""
        self._playback_commands: Deque[RecordedStep] = deque()
        self._playback_request_id = ""
        self._playback_name = ""
        self._playback_step = None
        self._playback_attempt_command = None
        self._playback_retries_left = 0
        self._playback_acquisition_active = False
        self._retry_generation = 0
        self._retry_settings = {0: 0}
        self._retry_tokens = {}
        self._retry_consumed = set()
        self._recording_retries = {}
        self._skip_block = 0
        self._last_skip_block = 0
        self._skip_boundary = 0
        self._next_skip_block = 1
        self._recording_skip_blocks = {}
        self._playback_had_skips = False

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
        self.recording_feedback_pub = self.create_publisher(
            String, "fault_detector/recording_feedback", LATCHED_QOS,
        )
        self.recording_retries_pub = self.create_publisher(
            UInt32, "fault_detector/recording_retries", LATCHED_QOS,
        )
        self.recording_skip_pub = self.create_publisher(
            String, "fault_detector/recording_skip_state", LATCHED_QOS,
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
        self._publish_recording_feedback("", "", "idle")

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
            elif mode == "set_retries":
                self.set_recording_retries(message.retries)
            elif mode in {"skip_start", "skip_stop", "skip_continue"}:
                if name and name != self.current_name:
                    raise ValueError("Skip control belongs to a different recording")
                self.set_skip_block(mode, message.skip_block)
        except (OSError, TypeError, ValueError) as exception:
            self.get_logger().error(f"Recording {mode} request failed: {exception}")
        finally:
            if mode in {"start", "stop", "pause", "resume", "set_retries",
                        "skip_start", "skip_stop", "skip_continue"}:
                self.publish_recording_state()

    def publish_recording_state(self):
        """Publish actual state, including after a rejected start or failed save."""
        self.recording_state_pub.publish(Bool(data=self.recording))
        self.recording_paused_pub.publish(Bool(data=self.recording_paused))
        self._publish_retry_setting()
        self._publish_skip_state()
        self.recording_finishing_pub.publish(Bool(
            data=self._recording_stop_requested and self._has_pending_commands()
        ))

    def _has_pending_commands(self):
        return any(
            request_id not in self._recording_results
            for request_id in self._recording_commands
        )

    def _adjacent_skip_block(self):
        if not self.recording or self._skip_block or not self._last_skip_block:
            return 0
        outside = list(self._recording_commands)[self._skip_boundary:]
        # A pending outside command may still be recorded, splitting the old block.
        if any(self._recording_results.get(request_id) not in (
            CommandStatus.STATE_FAILED, CommandStatus.STATE_CANCELLED,
        ) for request_id in outside):
            return 0
        return self._last_skip_block

    def _publish_skip_state(self):
        self.recording_skip_pub.publish(String(data=json.dumps({
            "active_block": self._skip_block,
            "adjacent_block": self._adjacent_skip_block(),
            "next_block": self._next_skip_block,
            "recording_name": self.current_name or "",
            "boundary_pending": bool(self.recording and not self._skip_block and self._last_skip_block
                                     and any(request_id not in self._recording_results
                                             for request_id in list(self._recording_commands)[self._skip_boundary:])),
        })))

    def set_skip_block(self, mode, expected_block=0):
        if not self.recording or self._recording_stop_requested:
            raise ValueError("Skip controls require an active recording")
        if mode == "skip_stop":
            if not self._skip_block:
                return
            if expected_block and expected_block != self._skip_block:
                raise ValueError("The active skip block changed")
            self._last_skip_block = self._skip_block
            self._skip_boundary = len(self._recording_commands)
            self._skip_block = 0
        elif self._skip_block:
            raise ValueError("Skip blocks cannot be nested")
        elif mode == "skip_continue":
            if not expected_block or expected_block != self._adjacent_skip_block():
                raise ValueError("The previous skip block can no longer be continued")
            self._skip_block = expected_block
        elif mode == "skip_start":
            if self._next_skip_block > 0xFFFFFFFF:
                raise ValueError("Skip block index limit reached")
            self._skip_block = self._next_skip_block
            self._next_skip_block += 1


    def set_recording_retries(self, count):
        validate_retries(count)
        if not self.recording or self._recording_stop_requested:
            raise ValueError("Retry settings require an active recording")
        self._retry_generation += 1
        self._retry_settings[self._retry_generation] = count

    def _publish_retry_setting(self):
        count = self._retry_settings[self._retry_generation]
        if self._retry_generation in self._retry_consumed:
            count = 0
        self.recording_retries_pub.publish(UInt32(data=count))

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
        self._last_recording_request_id = ""
        self._retry_generation = 0
        self._retry_settings = {0: 0}
        self._retry_tokens.clear()
        self._retry_consumed.clear()
        self._recording_retries.clear()
        self._skip_block = 0
        self._last_skip_block = 0
        self._skip_boundary = 0
        self._next_skip_block = 1
        self._recording_skip_blocks.clear()
        self._publish_recording_feedback("", "", "idle")
        self.get_logger().info(f"Started recording: {name}")

    def stop_recording(self):
        if not self.recording:
            return
        self._recording_stop_requested = True
        if self._has_pending_commands():
            self.get_logger().info("Waiting for command outcomes before saving recording.")
            return
        steps = [
            RecordedStep(command, self._recording_retries.get(request_id, 0),
                         self._recording_skip_blocks.get(request_id, 0))
            for request_id, command in self._recording_commands.items()
            if self._recording_results.get(request_id) == CommandStatus.STATE_SUCCEEDED
        ]
        self.repository.save(self.current_name, steps)
        self.get_logger().info(f"Saved recording: {self.current_name}")
        self.recording = False
        self.recording_paused = False
        self.current_name = None
        self._recorded_request_ids.clear()
        self._recording_commands.clear()
        self._recording_results.clear()
        self._recording_stop_requested = False
        self._retry_consumed.add(self._retry_generation)
        self._skip_block = 0
        self._last_skip_block = 0
        self.publish_recordings_list()

    def capture_request(self, message: CommandRequestMessage) -> bool:
        try:
            request = command_request_from_message(message)
        except (TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Rejected accepted command request: {exception}"
            )
            return False
        if request.command.command_id in {CommandID.EMERGENCY_CANCEL, CommandID.ESTOP_STATE}:
            self._finish_playback("Playback stopped by emergency command.", failed=True)
        if request.request_id in self._recorded_request_ids:
            return False
        if self.recording:
            self._recorded_request_ids.add(request.request_id)
        self._last_recording_request_id = request.request_id
        exclusion = (
            "inactive" if not self.recording else
            "finishing" if self._recording_stop_requested else
            "excluded" if request.recording_policy is not RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE else
            "paused" if self.recording_paused else ""
        )
        if exclusion:
            self._publish_recording_feedback(
                request.request_id, request.command.command_id.value, exclusion,
            )
        if exclusion and exclusion != "paused":
            return False
        if self.recording_paused:
            self.get_logger().info(
                f"Excluded command {request.command.command_id.value} "
                f"({request.request_id}) from {self.current_name}: recording paused"
            )
            return False
        self._recording_commands[request.request_id] = deepcopy(request.command)
        self._recording_skip_blocks[request.request_id] = self._skip_block
        self._retry_tokens[request.request_id] = self._retry_generation
        self._refresh_successful_commands()
        self._publish_command_recording_outcome(request.request_id)
        return True

    def _publish_recording_feedback(self, request_id, command_id, outcome):
        """Publish confirmed capture facts; the UI owns their presentation."""
        feedback = {
            "request_id": request_id, "command_id": command_id, "outcome": outcome,
        }
        retries = self._recording_retries.get(request_id, 0)
        block = self._recording_skip_blocks.get(request_id, 0)
        if block and outcome in {"pending", "added"}:
            feedback["skip_block"] = block
        if retries:
            feedback["playback_retries"] = retries
        if outcome == "added" and request_id in self._recording_retries and not retries:
            feedback["retry_unsupported"] = True
        self.recording_feedback_pub.publish(String(data=json.dumps(feedback)))

    def _publish_command_recording_outcome(self, request_id):
        if request_id != self._last_recording_request_id:
            return
        outcome = {
            CommandStatus.STATE_SUCCEEDED: "added",
            CommandStatus.STATE_FAILED: "failed",
            CommandStatus.STATE_CANCELLED: "cancelled",
        }.get(self._recording_results.get(request_id), "pending")
        self._publish_recording_feedback(
            request_id, self._recording_commands[request_id].command_id.value, outcome,
        )

    def _refresh_successful_commands(self):
        blocked_tokens = set()
        for request_id, command in self._recording_commands.items():
            token = self._retry_tokens[request_id]
            outcome = self._recording_results.get(request_id)
            if outcome is None:
                blocked_tokens.add(token)
            elif (outcome == CommandStatus.STATE_SUCCEEDED
                  and token not in blocked_tokens and token not in self._retry_consumed
                  and self._retry_settings[token]):
                self._retry_consumed.add(token)
                if command.command_id in RETRYABLE_COMMANDS:
                    self._recording_retries[request_id] = self._retry_settings[token]
                else:
                    self._recording_retries[request_id] = 0
                    self.get_logger().warning(
                        f"Recorded {command.command_id.value} without retries: retry unsupported"
                    )
        self._publish_retry_setting()
        self._publish_skip_state()
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
        if self._last_recording_request_id in self._recording_commands:
            self._publish_command_recording_outcome(self._last_recording_request_id)
        if self._recording_stop_requested and not self._has_pending_commands():
            # Use the same error handling as an explicit stop; unsaved data survives.
            self.handle_control(CommandRecordControl(mode="stop"))
        return True

    def play_recording(self, name: str):
        if self._playback_request_id or self._playback_commands:
            self.get_logger().warning("Playback is already active.")
            return
        try:
            commands = self.repository.load_steps(name)
        except (OSError, TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Could not load recording {name}: {exception}"
            )
            return
        self._playback_commands = deque(commands)
        self._playback_name = name
        self._playback_acquisition_active = False
        self._playback_had_skips = False
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
            if self._playback_step is not None:
                command_id = self._playback_attempt_command.command_id
                if command_id is CommandID.START_SENSOR_RECORDING:
                    self._playback_acquisition_active = True
                elif command_id is CommandID.STOP_SENSOR_RECORDING:
                    self._playback_acquisition_active = False
            self._playback_request_id = ""
            self._dispatch_next_playback_command()
            return True
        if message.state in (
            CommandStatus.STATE_FAILED,
            CommandStatus.STATE_CANCELLED,
        ):
            detail = message.detail or "command did not succeed"
            replacement_valid = True
            if message.state == CommandStatus.STATE_FAILED and message.has_retry_command:
                try:
                    self._playback_attempt_command = semantic_command_from_message(message.retry_command)
                except (TypeError, ValueError) as exception:
                    replacement_valid = False
                    detail += f"; invalid retry command: {exception}"
            needs_target = self._playback_attempt_command.command_id in (
                CommandID.MOVE_ARM_RELATIVE, CommandID.MOVE_BASE_RELATIVE,
            )
            if (message.state == CommandStatus.STATE_FAILED and message.retry_safe
                    and replacement_valid and not needs_target
                    and self._playback_retries_left > 0
                    and not self._playback_acquisition_active):
                self._playback_retries_left -= 1
                self.get_logger().info(
                    f"Retrying {self._playback_attempt_command.command_id.value} after "
                    f"{message.command_id}; {self._playback_retries_left} retries remain: {detail}"
                )
                self._dispatch_playback_step()
                return True
            if (message.state == CommandStatus.STATE_FAILED
                    and (message.continuation_safe or message.retry_safe)
                    and not self._playback_acquisition_active
                    and self._playback_step.skip_block):
                block = self._playback_step.skip_block
                skipped = 0
                while self._playback_commands and self._playback_commands[0].skip_block == block:
                    self._playback_commands.popleft()
                    skipped += 1
                self._playback_had_skips = True
                self.get_logger().warning(
                    f"Skipping block {block} after {message.command_id}: {detail}; "
                    f"{skipped} remaining command(s) skipped"
                )
                self._playback_request_id = ""
                self._dispatch_next_playback_command()
                return True
            self._finish_playback(
                f"Playback stopped after {message.command_id}: {detail}",
                failed=True,
            )
            return True
        return message.state == CommandStatus.STATE_RUNNING

    def _dispatch_next_playback_command(self):
        if not self._playback_commands:
            self._finish_playback(
                "Playback finished with skips." if self._playback_had_skips else "Playback finished."
            )
            return
        self._playback_step = self._playback_commands.popleft()
        self._playback_attempt_command = self._playback_step.command
        self._playback_retries_left = self._playback_step.playback_retries
        self._dispatch_playback_step()

    def _dispatch_playback_step(self):
        request = CommandRequest.create(
            command=self._playback_attempt_command,
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
        self._playback_step = None
        self._playback_attempt_command = None
        self._playback_retries_left = 0
        self._playback_acquisition_active = False
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
