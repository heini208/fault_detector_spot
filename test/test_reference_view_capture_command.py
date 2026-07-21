"""Tests for reference-view capture command integration."""

from types import SimpleNamespace

import py_trees
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import ComplexCommand

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
        self.error_messages = []

    def info(self, message):
        """Record an info message."""
        self.info_messages.append(message)

    def error(self, message):
        """Record an error message."""
        self.error_messages.append(message)


class FakeClock:
    """Provide one fixed capture time."""

    def now(self):
        """Return the fixed time."""
        return SimpleNamespace(nanoseconds=10_300_000_000)


class FakeNode:
    """Provide clock and logging required by the behavior."""

    def __init__(self):
        """Initialize fake node resources."""
        self.logger = FakeLogger()

    def get_clock(self):
        """Return the fixed clock."""
        return FakeClock()

    def get_logger(self):
        """Return the recorded logger."""
        return self.logger


def make_command(
    object_id="motor_a",
    routine_id="magnetic_scan",
):
    """Create one internal reference-view capture command."""
    return GenericCommand(
        command_id=(
            CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        ),
        stamp=Time(sec=10),
        object_id=object_id,
        routine_id=routine_id,
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
    synchronizer = object()
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
        if capture_error is not None:
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


def test_complex_message_preserves_inspection_identifiers():
    """ROS command fields reach the internal command unchanged."""
    message = ComplexCommand()
    message.command.command_id = (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
    )
    message.object_id = "motor_a"
    message.routine_id = "magnetic_scan"

    command = CommandSubscriber().complex_message_to_generic_command(
        message
    )

    assert command.object_id == "motor_a"
    assert command.routine_id == "magnetic_scan"


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


def test_behavior_creates_runtime_resources_and_captures(monkeypatch):
    """One command invokes the complete capture transaction."""
    behavior, node, created, capture_calls = configure_behavior(
        monkeypatch
    )
    create_writer(make_command())

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert created["repository"][0] == "/tmp/objects"
    assert created["synchronizer"]["queue_size"] == 7
    assert created["listener"] == (behavior.tf_buffer, node)
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
    }
    assert "reference_datasets/magnetic_scan" in (
        behavior.feedback_message
    )
    assert len(node.logger.info_messages) == 1


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

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert len(capture_calls) == 1
    assert "inputs unavailable" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_capture_command_is_not_recorded():
    """Playback cannot overwrite a taught reference dataset."""
    manager = RecordManager.__new__(RecordManager)
    manager.recording = True
    manager.temp_data = []
    message = ComplexCommand()
    message.command.command_id = (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
    )

    manager.capture_command(message)

    assert manager.temp_data == []
