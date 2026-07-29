"""Tests for shared-tag and selected-camera capture integration."""

from types import SimpleNamespace

import py_trees
import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import (
    InspectionCommand,
    TagElement,
    TagElementArray,
)

from fault_detector_spot.behaviour_tree.commands import (
    generic_complex_command,
)
from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.nodes.inspection import (
    capture_inspection_object_reference_view as capture_module,
)


GenericCommand = generic_complex_command.GenericCommand
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


class FakeFilterSubscription:
    def __init__(self):
        self.unsubscribed = False

    def unsubscribe(self):
        self.unsubscribed = True


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()
        self.clock = FakeClock()
        self.destroyed_subscriptions = []
        self.subscriptions = []

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def create_subscription(
        self,
        message_type,
        topic,
        callback,
        qos,
    ):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)


class FakeSynchronizer:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.image_sequence = 0
        self.input_ready = False
        self.ready = False
        self.begin_calls = []
        self.cancel_count = 0
        self.tag_updates = []
        self.rgb_subscription = FakeFilterSubscription()
        self.depth_subscription = FakeFilterSubscription()
        self.camera_info_subscription = object()
        self.base_tag_subscription = None

    def update_base_tag_observations(self, observations):
        values = tuple(observations)
        self.tag_updates.append(values)
        if values:
            self.input_ready = True

    def ready_for_collection(self, reference_tag_id):
        return self.input_ready

    def input_diagnostics(self, reference_tag_id):
        return (
            "camera_info=yes, rgb_depth_pairs=1, "
            f"tag_samples={sum(len(value) for value in self.tag_updates)}"
        )

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


class FakeRepository:
    pass


def make_tag(tag_id=23, sec=10, nanosec=0):
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = sec
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_tag_array(*tags):
    message = TagElementArray()
    message.elements = list(tags)
    return message


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
    writer = py_trees.blackboard.Client(
        name="CaptureCommandWriter"
    )
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = command
    return writer


def configure_behavior(monkeypatch, capture_error=None):
    created = {
        "synchronizers": {},
        "synchronizer_kwargs": {},
    }
    capture_calls = []

    monkeypatch.setattr(
        capture_module,
        "MultiReferenceViewRepository",
        lambda root: created.setdefault(
            "repository",
            FakeRepository(),
        ),
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
        created["synchronizer_kwargs"][camera_id] = kwargs
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
        if (
            capture_error is not None
            and not isinstance(capture_error, list)
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
        maximum_input_age_sec=5.0,
        maximum_timestamp_skew_sec=0.03,
        maximum_tag_timestamp_skew_sec=5.0,
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


def begin_with_retained_tag(
    behavior,
    created,
    camera_ids,
):
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )
    assert behavior.update() == py_trees.common.Status.RUNNING
    for camera_id in camera_ids:
        created["synchronizers"][camera_id].input_ready = True
    assert behavior.update() == py_trees.common.Status.RUNNING


def test_setup_keeps_one_shared_tag_subscription(monkeypatch):
    behavior, node, created, _ = configure_behavior(monkeypatch)

    assert behavior._synchronizers == {}
    assert created["synchronizers"] == {}
    assert [subscription.topic for subscription in node.subscriptions] == [
        "fault_detector/state/visible_tags"
    ]


def test_hand_capture_is_seeded_from_retained_tag_history(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING

    synchronizer = created["synchronizers"]["hand"]
    assert set(created["synchronizers"]) == {"hand"}
    assert synchronizer.tag_updates
    assert synchronizer.tag_updates[0][0].id == 23
    assert (
        created["synchronizer_kwargs"]["hand"]["base_tag_topic"]
        is None
    )
    assert capture_calls == []


def test_new_tag_messages_are_fanned_to_selected_cameras(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    selected = ("frontleft", "hand", "back")
    create_writer(make_command(camera_ids=selected))

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )

    assert all(
        created["synchronizers"][camera_id].tag_updates
        for camera_id in selected
    )


def test_warm_inputs_start_independent_selected_windows(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    selected = ("frontleft", "hand", "back")
    create_writer(make_command(camera_ids=selected))

    begin_with_retained_tag(behavior, created, selected)

    assert set(created["synchronizers"]) == set(selected)
    assert all(
        created["synchronizers"][camera_id].begin_calls
        == [(23, 1)]
        for camera_id in selected
    )


def test_capture_waits_for_every_selected_camera(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    selected = ("left", "right")
    create_writer(make_command(camera_ids=("left", "right", "")))
    begin_with_retained_tag(behavior, created, selected)

    created["synchronizers"]["left"].ready = True
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert capture_calls == []

    created["synchronizers"]["right"].ready = True
    assert behavior.update() == py_trees.common.Status.SUCCESS
    requests = capture_calls[0][0][1]
    assert [
        (request.slot_index, request.camera_id)
        for request in requests
    ] == [
        (0, "left"),
        (1, "right"),
    ]
    assert behavior._synchronizers == {}


def test_tf_retry_reuses_completed_attempt_validation_time(monkeypatch):
    error = capture_module.tf2_ros.TransformException(
        "camera frame is pending"
    )
    behavior, node, created, capture_calls = configure_behavior(
        monkeypatch,
        capture_error=[error],
    )
    create_writer(make_command())
    begin_with_retained_tag(behavior, created, ("hand",))

    created["synchronizers"]["hand"].ready = True
    first_time = node.clock.now()
    assert behavior.update() == py_trees.common.Status.RUNNING

    node.clock.advance(1.0)
    assert behavior.update() == py_trees.common.Status.SUCCESS

    assert capture_calls[0][0][6] is first_time
    assert capture_calls[1][0][6] is first_time


def test_backward_bag_time_clears_retained_history(monkeypatch):
    behavior, node, _, _ = configure_behavior(monkeypatch)
    behavior._base_tags_callback(
        make_tag_array(make_tag(sec=20))
    )
    behavior._base_tags_callback(
        make_tag_array(make_tag(sec=10))
    )

    history = behavior._base_tag_history[23]
    assert len(history) == 1
    assert history[0].pose.header.stamp.sec == 10
    assert node.logger.warning_messages


def test_warmup_timeout_reports_selected_camera(monkeypatch):
    behavior, node, created, capture_calls = configure_behavior(
        monkeypatch
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    node.clock.advance(3.1)

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "hand:" in behavior.feedback_message
    assert created["synchronizers"]["hand"].begin_calls == []
    assert capture_calls == []
    assert behavior._synchronizers == {}


def test_initialise_releases_camera_but_keeps_tag_subscription(
    monkeypatch,
):
    behavior, node, created, _ = configure_behavior(monkeypatch)
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer = created["synchronizers"]["hand"]
    shared_subscription = behavior.base_tag_subscription

    behavior.initialise()

    assert synchronizer.rgb_subscription.unsubscribed is True
    assert synchronizer.depth_subscription.unsubscribed is True
    assert len(node.destroyed_subscriptions) == 1
    assert shared_subscription not in node.destroyed_subscriptions
    assert behavior._synchronizers == {}


def test_duplicate_camera_fails_before_camera_subscriptions(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=("hand", "hand", "")))

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert created["synchronizers"] == {}
    assert capture_calls == []


def test_no_camera_fails_before_camera_subscriptions(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    create_writer(make_command(camera_ids=("", "", "")))

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert created["synchronizers"] == {}


@pytest.mark.parametrize(
    "duration",
    [0.0, -1.0, float("inf")],
)
def test_invalid_collection_duration_is_rejected(duration):
    with pytest.raises(ValueError, match="Collection duration"):
        CaptureInspectionObjectReferenceView(
            collection_duration_sec=duration,
        )


@pytest.mark.parametrize(
    "timeout",
    [0.0, -1.0, float("inf")],
)
def test_invalid_capture_timeout_is_rejected(timeout):
    with pytest.raises(ValueError, match="Capture timeout"):
        CaptureInspectionObjectReferenceView(
            capture_timeout_sec=timeout,
        )


@pytest.mark.parametrize(
    "attempts",
    [0, -1, 1.5, True],
)
def test_invalid_capture_attempt_count_is_rejected(attempts):
    with pytest.raises(ValueError, match="maximum attempts"):
        CaptureInspectionObjectReferenceView(
            capture_max_attempts=attempts,
        )
