"""Tests for multi-camera reference-view behavior integration."""

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
        self.destroyed_subscriptions = []

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)


class FakeSynchronizer:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.image_sequence = 0
        self.ready = False
        self.begin_calls = []
        self.cancel_count = 0

    def begin_collection(self, reference_tag_id, minimum_image_sequence):
        self.begin_calls.append(
            (reference_tag_id, minimum_image_sequence)
        )

    def collection_ready(self):
        return self.ready

    def cancel_collection(self):
        self.cancel_count += 1


class FakeRepository:
    pass


def make_command(
    object_id="motor_a",
    routine_id="magnetic_scan",
    replace_existing=False,
    camera_ids=("hand", "", ""),
):
    inspection = InspectionCommand()
    inspection.object.object_id = object_id
    inspection.routine.routine_id = routine_id
    inspection.replace_existing = replace_existing
    inspection.reference_camera_ids = list(camera_ids)
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
    created = {"synchronizers": {}}
    capture_calls = []

    monkeypatch.setattr(
        capture_module,
        "MultiReferenceViewRepository",
        lambda root: created.setdefault("repository", FakeRepository()),
    )
    monkeypatch.setattr(
        capture_module,
        "validate_multi_reference_view_capture_target",
        lambda *args: 23,
    )

    def make_synchronizer(**kwargs):
        camera_id = kwargs["rgb_topic"].split("/")[-2]
        synchronizer = FakeSynchronizer(camera_id)
        created["synchronizers"][camera_id] = synchronizer
        return synchronizer

    monkeypatch.setattr(
        capture_module,
        "ReferenceViewInputSynchronizer",
        make_synchronizer,
    )
    monkeypatch.setattr(
        capture_module.tf2_ros,
        "Buffer",
        lambda: object(),
    )
    monkeypatch.setattr(
        capture_module.tf2_ros,
        "TransformListener",
        lambda buffer, node: object(),
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
        return object()

    monkeypatch.setattr(
        capture_module,
        "capture_reference_views",
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
    return behavior, node, created, capture_calls


def setup_function():
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    py_trees.blackboard.Blackboard.clear()


def test_first_tick_starts_selected_camera_collection(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command())

    status = behavior.update()

    assert status == py_trees.common.Status.RUNNING
    synchronizer = created["synchronizers"]["hand"]
    assert synchronizer.begin_calls == [(23, 1)]
    assert capture_calls == []


def test_three_camera_slots_start_independent_collections(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=(
        "frontleft",
        "hand",
        "back",
    )))

    assert behavior.update() == py_trees.common.Status.RUNNING

    assert set(created["synchronizers"]) == {
        "frontleft",
        "hand",
        "back",
    }
    assert all(
        synchronizer.begin_calls == [(23, 1)]
        for synchronizer in created["synchronizers"].values()
    )


def test_capture_waits_for_every_selected_camera(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=("left", "right", "")))
    assert behavior.update() == py_trees.common.Status.RUNNING

    created["synchronizers"]["left"].ready = True
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert capture_calls == []

    created["synchronizers"]["right"].ready = True
    assert behavior.update() == py_trees.common.Status.SUCCESS
    requests = capture_calls[0][0][1]
    assert [(request.slot_index, request.camera_id) for request in requests] == [
        (0, "left"),
        (1, "right"),
    ]


def test_invalid_completed_set_retries_all_cameras(monkeypatch):
    errors = [
        capture_module.ReferenceViewCaptureNotReady(
            "frontleft has no valid synchronized set"
        )
    ]
    behavior, node, created, capture_calls = configure_behavior(
        monkeypatch,
        capture_error=errors,
    )
    create_writer(make_command(camera_ids=("frontleft", "hand", "")))
    assert behavior.update() == py_trees.common.Status.RUNNING
    for synchronizer in created["synchronizers"].values():
        synchronizer.ready = True
        synchronizer.image_sequence = 1

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._attempt_number == 2
    assert all(
        synchronizer.begin_calls == [(23, 1), (23, 2)]
        for synchronizer in created["synchronizers"].values()
    )
    assert len(capture_calls) == 1
    assert len(node.logger.warning_messages) == 1


def test_duplicate_camera_fails_before_subscribing(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=("hand", "hand", "")))

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert created["synchronizers"] == {}
    assert capture_calls == []


def test_no_camera_fails_before_subscribing(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=("", "", "")))

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert created["synchronizers"] == {}


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
