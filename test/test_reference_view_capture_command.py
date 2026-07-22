"""Tests for reference-view collection behavior integration."""

from types import SimpleNamespace

import py_trees
import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import InspectionCommand

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.behaviour_tree.nodes.inspection import (
    capture_inspection_object_reference_view as capture_module,
)


CaptureInspectionObjectReferenceView = (
    capture_module.CaptureInspectionObjectReferenceView
)


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class FakeClock:
    def __init__(self):
        self.nanoseconds = 10_300_000_000

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)

    def advance(self, seconds):
        self.nanoseconds += int(seconds * 1_000_000_000)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()
        self.clock = FakeClock()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger


class FakeSynchronizer:
    def __init__(self):
        self.image_sequence = 0
        self.ready = False
        self.begin_calls = []
        self.cancel_count = 0

    def begin_collection(
        self,
        reference_tag_id,
        minimum_image_sequence,
    ):
        self.begin_calls.append(
            (reference_tag_id, minimum_image_sequence)
        )

    def collection_ready(self):
        return self.ready

    def cancel_collection(self):
        self.cancel_count += 1


def make_command(
    object_id="motor_a",
    routine_id="magnetic_scan",
    replace_existing=False,
):
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
    writer = py_trees.blackboard.Client(name="CaptureCommandWriter")
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = command
    return writer


def configure_behavior(monkeypatch, capture_error=None):
    repository = object()
    synchronizer = FakeSynchronizer()
    tf_buffer = object()
    created = {}
    capture_calls = []

    monkeypatch.setattr(
        capture_module,
        "ObjectRepository",
        lambda root: created.setdefault(
            "repository",
            (root, repository),
        )[1],
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
        lambda *args: 23,
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
        maximum_input_age_sec=2.0,
        maximum_timestamp_skew_sec=0.03,
        maximum_tag_timestamp_skew_sec=0.2,
        collection_duration_sec=1.0,
        fixed_frame="odom",
        transform_timeout_sec=0.1,
        capture_timeout_sec=3.0,
        capture_max_attempts=3,
    )
    node = FakeNode()
    behavior.setup(node=node)
    return behavior, node, created, capture_calls, synchronizer


def setup_function():
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    py_trees.blackboard.Blackboard.clear()


def test_first_tick_starts_explicit_collection(monkeypatch):
    behavior, _, created, capture_calls, synchronizer = (
        configure_behavior(monkeypatch)
    )
    create_writer(make_command())

    status = behavior.update()

    assert status == py_trees.common.Status.RUNNING
    assert synchronizer.begin_calls == [(23, 1)]
    assert capture_calls == []
    assert created["synchronizer"]["collection_duration_sec"] == 1.0
    assert created["synchronizer"][
        "maximum_tag_timestamp_skew_sec"
    ] == 0.2


def test_behavior_waits_until_collection_is_ready(monkeypatch):
    behavior, _, _, capture_calls, synchronizer = (
        configure_behavior(monkeypatch)
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert capture_calls == []
    assert synchronizer.begin_calls == [(23, 1)]


def test_completed_collection_is_captured_and_released(monkeypatch):
    behavior, node, _, capture_calls, synchronizer = (
        configure_behavior(monkeypatch)
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer.ready = True
    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert len(capture_calls) == 1
    assert capture_calls[0][1]["minimum_image_sequence"] == 1
    assert synchronizer.cancel_count >= 2
    assert len(node.logger.info_messages) == 1


def test_invalid_completed_collection_starts_new_attempt(monkeypatch):
    errors = [
        capture_module.ReferenceViewCaptureNotReady(
            "No valid synchronized set"
        )
    ]
    behavior, node, _, capture_calls, synchronizer = (
        configure_behavior(monkeypatch, capture_error=errors)
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer.image_sequence = 1
    synchronizer.ready = True
    status = behavior.update()

    assert status == py_trees.common.Status.RUNNING
    assert behavior._attempt_number == 2
    assert synchronizer.begin_calls == [(23, 1), (23, 2)]
    assert len(capture_calls) == 1
    assert len(node.logger.warning_messages) == 1


def test_collection_timeout_retries_with_new_sequence(monkeypatch):
    behavior, node, _, capture_calls, synchronizer = (
        configure_behavior(monkeypatch)
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer.image_sequence = 1
    node.clock.advance(3.1)
    status = behavior.update()

    assert status == py_trees.common.Status.RUNNING
    assert behavior._attempt_number == 2
    assert synchronizer.begin_calls == [(23, 1), (23, 2)]
    assert capture_calls == []


def test_three_failed_collections_fail_command(monkeypatch):
    behavior, node, _, _, synchronizer = configure_behavior(monkeypatch)
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.RUNNING
    node.clock.advance(3.1)
    assert behavior.update() == py_trees.common.Status.RUNNING
    node.clock.advance(3.1)
    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert "failed after 3 attempts" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1
    assert len(synchronizer.begin_calls) == 3


def test_missing_identifiers_fail_before_collection(monkeypatch):
    behavior, _, _, capture_calls, synchronizer = (
        configure_behavior(monkeypatch)
    )
    create_writer(make_command(object_id=""))

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert capture_calls == []
    assert synchronizer.begin_calls == []


def test_wrong_command_id_fails_before_collection(monkeypatch):
    behavior, _, _, _, synchronizer = configure_behavior(monkeypatch)
    command = make_command()
    command.command_id = CommandID.STOW_ARM
    create_writer(command)

    status = behavior.update()

    assert status == py_trees.common.Status.FAILURE
    assert synchronizer.begin_calls == []


@pytest.mark.parametrize("duration", [0.0, -1.0, float("inf")])
def test_invalid_collection_duration_is_rejected(duration):
    with pytest.raises(ValueError, match="Collection duration"):
        CaptureInspectionObjectReferenceView(
            collection_duration_sec=duration,
        )


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("inf")])
def test_invalid_capture_timeout_is_rejected(timeout):
    with pytest.raises(ValueError, match="Capture timeout"):
        CaptureInspectionObjectReferenceView(
            capture_timeout_sec=timeout,
        )


@pytest.mark.parametrize("attempts", [0, -1, 1.5, True])
def test_invalid_capture_attempt_count_is_rejected(attempts):
    with pytest.raises(ValueError, match="maximum attempts"):
        CaptureInspectionObjectReferenceView(
            capture_max_attempts=attempts,
        )
