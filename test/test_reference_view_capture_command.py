"""Tests for reference-view capture command integration."""

from types import SimpleNamespace

import py_trees
import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import ComplexCommand, InspectionCommand

from fault_detector_spot.behaviour_tree import bt_runner
from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.behaviour_tree.nodes.inspection import (
    capture_inspection_object_reference_view as capture_module,
)
from fault_detector_spot.behaviour_tree.nodes.sensing.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.behaviour_tree.record_manager_node import (
    RecordManager,
)


CaptureInspectionObjectReferenceView = (
    capture_module.CaptureInspectionObjectReferenceView
)


class FakeLogger:
    """Record behavior messages."""

    def __init__(self):
        """Initialize log storage."""
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, message):
        """Record an info message."""
        self.info_messages.append(message)

    def error(self, message):
        """Record an error message."""
        self.error_messages.append(message)

    def warning(self, message):
        """Record a warning message."""
        self.warning_messages.append(message)


class FakeClock:
    """Provide a controllable capture time."""

    def __init__(self):
        """Start at the standard fresh-input test time."""
        self.nanoseconds = 10_300_000_000

    def now(self):
        """Return the current test time."""
        return SimpleNamespace(nanoseconds=self.nanoseconds)

    def advance(self, seconds):
        """Advance the test clock by a duration in seconds."""
        self.nanoseconds += int(seconds * 1_000_000_000)


class FakeNode:
    """Provide clock and logging required by the behavior."""

    def __init__(self):
        """Initialize fake node resources."""
        self.logger = FakeLogger()
        self.clock = FakeClock()

    def get_clock(self):
        """Return the controllable clock."""
        return self.clock

    def get_logger(self):
        """Return the recorded logger."""
        return self.logger


def make_command(
    object_id="motor_a",
    routine_id="magnetic_scan",
    replace_existing=False,
):
    """Create one internal reference-view capture command."""
    inspection = InspectionCommand()
    inspection.object.object_id = object_id
    inspection.routine.routine_id = routine_id
    inspection.replace_existing = replace_existing
    return GenericCommand(
        command_id=(
            CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        ),
        stamp=Time(sec=10),
        inspection=inspection,
    )


def create_writer(command):
    """Write the command consumed by the behavior."""
    writer = py_trees.blackboard.Client(name="CaptureCommandWriter")
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = command
    return writer


def configure_behavior(monkeypatch, capture_error=None):
    """Create the behavior with observable runtime resources."""
    repository = object()
    synchronizer = SimpleNamespace(image_sequence=0)
    tf_buffer = object()
    created = {}
    capture_calls = []

    monkeypatch.setattr(
        capture_module,
        "ObjectRepository",
        lambda root: created.setdefault("repository", (root, repository))[1],
    )

    def make_synchronizer(**kwargs):
        created["synchronizer"] = kwargs
        return synchronizer

    monkeypatch.setattr(
        capture_module,
        "ReferenceViewInputSynchronizer",
        make_synchronizer,
    )
    monkeypatch.setattr(
        capture_module,
        "validate_reference_view_capture_target",
        lambda *args: created.setdefault("capture_target", args),
    )
    monkeypatch.setattr(
        capture_module.tf2_ros,
        "Buffer",
        lambda: tf_buffer,
    )
    monkeypatch.setattr(
        capture_module.tf2_ros,
        "TransformListener",
        lambda buffer, node: created.setdefault(
            "listener",
            (buffer, node),
        ),
    )

    def capture(*args, **kwargs):
        capture_calls.append((args, kwargs))
        if isinstance(capture_error, list) and capture_error:
            raise capture_error.pop(0)
        if capture_error is not None and not isinstance(
            capture_error,
            list,
        ):
            raise capture_error
        view = SimpleNamespace(
            reference_dataset_path=(
                "reference_datasets/magnetic_scan/10_000000000"
            )
        )
        routine = SimpleNamespace(reference_view=view)
        return SimpleNamespace(
            get_routine=lambda routine_id: routine,
        )

    monkeypatch.setattr(
        capture_module,
        "capture_reference_view",
        capture,
    )

    behavior = CaptureInspectionObjectReferenceView(
        object_root="/tmp/objects",
        synchronization_queue_size=7,
        maximum_input_age_sec=0.4,
        maximum_timestamp_skew_sec=0.03,
        maximum_tag_timestamp_skew_sec=0.2,
        fixed_frame="odom",
        transform_timeout_sec=0.1,
        capture_timeout_sec=3.0,
        capture_max_attempts=3,
    )
    node = FakeNode()
    behavior.setup(node=node)
    return behavior, node, created, capture_calls


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_complex_message_preserves_nested_inspection_payload():
    """The nested ROS payload reaches the internal command unchanged."""
    message = ComplexCommand()
    message.command.command_id = (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
    )
    message.inspection.object.object_id = "motor_a"
    message.inspection.object.display_name = "Motor A"
    message.inspection.object.reference_tag_id = 23
    message.inspection.object.reference_tag_family = "36h11"
    message.inspection.routine.routine_id = "magnetic_scan"
    message.inspection.routine.display_name = "Magnetic scan"
    message.inspection.routine.sensor_id = "bmm150"
    message.inspection.routine.probe_frame = "sensor_tip"
    message.inspection.replace_existing = True

    command = CommandSubscriber().complex_message_to_generic_command(
        message
    )

    assert command.inspection is message.inspection
    assert command.inspection.object.object_id == "motor_a"
    assert command.inspection.object.display_name == "Motor A"
    assert command.inspection.object.reference_tag_id == 23
    assert command.inspection.object.reference_tag_family == "36h11"
    assert command.inspection.routine.routine_id == "magnetic_scan"
    assert command.inspection.routine.display_name == "Magnetic scan"
    assert command.inspection.routine.sensor_id == "bmm150"
    assert command.inspection.routine.probe_frame == "sensor_tip"
    assert command.inspection.replace_existing is True


def test_command_selector_dispatches_reference_view_capture(
    monkeypatch,
):
    """The command selector contains the capture behavior branch."""
    command_ids = []

    def make_sequence(node, command_id, constructor):
        command_ids.append(command_id)
        return py_trees.behaviours.Success(name=command_id.value)

    monkeypatch.setattr(
        bt_runner,
        "get_helper_container",
        lambda node: SimpleNamespace(slam_helper=object()),
    )
    monkeypatch.setattr(
        bt_runner,
        "make_simple_command_sequence",
        make_sequence,
    )

    bt_runner.build_command_tree(object())

    assert (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        in command_ids
    )
    assert CommandID.CREATE_INSPECTION_OBJECT in command_ids
    assert CommandID.CREATE_INSPECTION_ROUTINE in command_ids
    assert CommandID.DELETE_INSPECTION_OBJECT in command_ids
    assert CommandID.DELETE_INSPECTION_ROUTINE in command_ids


def test_behavior_creates_runtime_resources_and_captures(monkeypatch):
    """One command waits for and captures the next synchronized pair."""
    behavior, node, created, capture_calls = configure_behavior(
        monkeypatch
    )
    create_writer(make_command())

    first_status = behavior.update()
    behavior.input_synchronizer.image_sequence = 1
    status = behavior.update()

    assert first_status == py_trees.common.Status.RUNNING
    assert status == py_trees.common.Status.SUCCESS
    assert created["repository"][0] == "/tmp/objects"
    assert created["synchronizer"]["queue_size"] == 7
    assert created["synchronizer"]["camera_info_topic"] == (
        "/depth_registered/hand/camera_info"
    )
    assert created["listener"] == (behavior.tf_buffer, node)
    assert created["capture_target"][1:4] == (
        "motor_a",
        "magnetic_scan",
        False,
    )
    args, kwargs = capture_calls[0]
    assert args[0] is behavior.object_repository
    assert args[1] is behavior.input_synchronizer
    assert args[2] is behavior.tf_buffer
    assert args[3:5] == ("motor_a", "magnetic_scan")
    assert args[5].nanoseconds == 10_300_000_000
    assert kwargs == {
        "maximum_input_age_sec": 0.4,
        "maximum_timestamp_skew_sec": 0.03,
        "maximum_tag_timestamp_skew_sec": 0.2,
        "fixed_frame": "odom",
        "transform_timeout_sec": 0.1,
        "replace_existing": False,
        "minimum_image_sequence": 1,
    }
    assert "reference_datasets/magnetic_scan" in (
        behavior.feedback_message
    )
    assert len(node.logger.info_messages) == 1


def test_behavior_passes_explicit_replacement(monkeypatch):
    """The command replacement flag reaches capture unchanged."""
    behavior, _, _, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(replace_existing=True))

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior.input_synchronizer.image_sequence = 1
    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert capture_calls[0][1]["replace_existing"] is True


def test_missing_identifiers_fail_without_capture(monkeypatch):
    """Incomplete commands cannot reach sensor or storage work."""
    behavior, node, _, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(object_id=""))

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert capture_calls == []
    assert "object_id" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_missing_routine_id_fails_without_capture(monkeypatch):
    """A routine must be selected before capture starts."""
    behavior, node, _, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(routine_id=""))

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert capture_calls == []
    assert "routine_id" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_wrong_command_id_fails_without_capture(monkeypatch):
    """The behavior rejects accidental direct dispatch."""
    behavior, node, _, capture_calls = configure_behavior(monkeypatch)
    command = make_command()
    command.command_id = CommandID.STOW_ARM
    create_writer(command)

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert capture_calls == []
    assert "not a reference-view capture" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_capture_failure_becomes_behavior_failure(monkeypatch):
    """Capture errors fail the command without crashing the tree."""
    behavior, node, _, capture_calls = configure_behavior(
        monkeypatch,
        capture_error=RuntimeError("inputs unavailable"),
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior.input_synchronizer.image_sequence = 1
    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert len(capture_calls) == 1
    assert "inputs unavailable" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_waiting_for_new_images_retries_three_times_then_fails(
    monkeypatch,
):
    """One click receives three bounded nonblocking attempts."""
    behavior, node, _, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._attempt_number == 2
    assert len(node.logger.warning_messages) == 1
    assert node.logger.error_messages == []

    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._attempt_number == 3
    assert len(node.logger.warning_messages) == 2

    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.FAILURE
    assert capture_calls == []
    assert "failed after 3 attempts of 3.000 s" in (
        behavior.feedback_message
    )
    assert len(node.logger.error_messages) == 1


def test_retry_requires_an_image_from_the_new_attempt(monkeypatch):
    """A retry cannot reuse the frame rejected by the prior attempt."""
    behavior, node, _, capture_calls = configure_behavior(
        monkeypatch,
        capture_error=[
            capture_module.ReferenceViewCaptureNotReady("RGB stale")
        ],
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior.input_synchronizer.image_sequence = 1
    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._minimum_image_sequence == 2
    assert len(capture_calls) == 1

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert len(capture_calls) == 1
    behavior.input_synchronizer.image_sequence = 2
    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert len(capture_calls) == 2


def test_transient_input_failure_retries_until_success(monkeypatch):
    """A temporary sensor mismatch does not consume the command."""
    errors = [capture_module.ReferenceViewCaptureNotReady("RGB stale")]
    behavior, node, _, capture_calls = configure_behavior(
        monkeypatch,
        capture_error=errors,
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior.input_synchronizer.image_sequence = 1
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert node.logger.error_messages == []
    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert len(capture_calls) == 2


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("inf")])
def test_invalid_capture_timeout_is_rejected(timeout):
    """Capture cannot be configured with an unusable deadline."""
    with pytest.raises(ValueError, match="Capture timeout"):
        CaptureInspectionObjectReferenceView(
            capture_timeout_sec=timeout,
        )


@pytest.mark.parametrize("attempts", [0, -1, 1.5, True])
def test_invalid_capture_attempt_count_is_rejected(attempts):
    """Capture retry count must be a positive integer."""
    with pytest.raises(ValueError, match="maximum attempts"):
        CaptureInspectionObjectReferenceView(
            capture_max_attempts=attempts,
        )


@pytest.mark.parametrize(
    "command_id",
    [
        CommandID.CREATE_INSPECTION_OBJECT,
        CommandID.CREATE_INSPECTION_ROUTINE,
        CommandID.DELETE_INSPECTION_OBJECT,
        CommandID.DELETE_INSPECTION_ROUTINE,
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW,
    ],
)
def test_inspection_definition_commands_are_not_recorded(command_id):
    """Playback cannot recreate or replace inspection definitions."""
    manager = RecordManager.__new__(RecordManager)
    manager.recording = True
    manager.temp_data = []
    message = ComplexCommand()
    message.command.command_id = command_id

    manager.capture_command(message)

    assert manager.temp_data == []
