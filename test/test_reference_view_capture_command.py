"""Tests for shared-tag and selected-camera capture integration."""

from copy import deepcopy
from types import SimpleNamespace

import py_trees
import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import (
    InspectionCommand,
    TagElement,
    TagElementArray,
)

from fault_detector_spot.application.commanding import (
    generic_complex_command,
)
from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
)
from fault_detector_spot.inspection.behaviours import (
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
        self.input_sequence = 0
        self.input_ready = False
        self.ready = False
        self.begin_calls = []
        self.cancel_count = 0
        self.rgb_subscription = FakeFilterSubscription()
        self.depth_subscription = FakeFilterSubscription()
        self.rgb_camera_info_subscription = object()
        self.depth_camera_info_subscription = object()
        self.latest_rgb_frame_id = (
            f"{camera_id}_color_image_sensor"
        )

    def ready_for_collection(self):
        return self.input_ready

    def input_diagnostics(self):
        return (
            "rgb_camera_info=yes, depth_camera_info=yes, "
            "rgb_frames=1, depth_frames=1"
        )

    def begin_collection(self, minimum_input_sequence):
        self.begin_calls.append((minimum_input_sequence,))

    def collection_ready(self):
        return self.ready

    def cancel_collection(self):
        self.cancel_count += 1


class FakeRepository:
    pass


class FakeTFBuffer:
    def __init__(self):
        self.lookup_error = None

    def transform(self, pose, target_frame, timeout=None):
        transformed = deepcopy(pose)
        transformed.header.frame_id = target_frame
        return transformed

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
        timeout=None,
    ):
        if self.lookup_error is not None:
            raise self.lookup_error
        return object()


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
        FakeTFBuffer,
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
    behavior._base_tags_callback(
        make_tag_array(make_tag(nanosec=100_000_000))
    )


def test_setup_keeps_one_shared_tag_subscription(monkeypatch):
    behavior, node, created, _ = configure_behavior(monkeypatch)

    assert behavior._synchronizers == {}
    assert created["synchronizers"] == {}
    assert [subscription.topic for subscription in node.subscriptions] == [
        "fault_detector/state/base_tags"
    ]


def test_camera_collector_does_not_receive_retained_tags(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )
    create_writer(make_command())

    assert behavior.update() == py_trees.common.Status.RUNNING

    assert set(created["synchronizers"]) == {"hand"}
    assert "base_tag_topic" not in (
        created["synchronizer_kwargs"]["hand"]
    )
    assert capture_calls == []


def test_new_tag_messages_stay_in_shared_history(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    selected = ("frontleft", "hand", "back")
    create_writer(make_command(camera_ids=selected))

    assert behavior.update() == py_trees.common.Status.RUNNING
    behavior._base_tags_callback(
        make_tag_array(make_tag())
    )

    assert behavior._base_tag_history[23][-1].id == 23
    assert behavior._visible_tag_ids == {23}


def test_empty_latest_tag_snapshot_blocks_collection(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    create_writer(make_command())
    behavior._base_tags_callback(make_tag_array(make_tag()))

    assert behavior.update() == py_trees.common.Status.RUNNING
    created["synchronizers"]["hand"].input_ready = True
    behavior._base_tags_callback(make_tag_array())

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert created["synchronizers"]["hand"].begin_calls == []
    assert "not visible" in behavior.feedback_message


def test_warm_inputs_start_independent_selected_windows(monkeypatch):
    behavior, _, created, _ = configure_behavior(monkeypatch)
    selected = ("frontleft", "hand", "back")
    create_writer(make_command(camera_ids=selected))

    begin_with_retained_tag(behavior, created, selected)

    assert set(created["synchronizers"]) == set(selected)
    assert all(
        created["synchronizers"][camera_id].begin_calls
        == [(1,)]
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

    assert (
        capture_calls[0][0][5].nanoseconds
        == first_time.nanoseconds
    )
    assert (
        capture_calls[1][0][5].nanoseconds
        == first_time.nanoseconds
    )


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
    for expected_status in (
        py_trees.common.Status.RUNNING,
        py_trees.common.Status.RUNNING,
        py_trees.common.Status.FAILURE,
    ):
        node.clock.advance(3.1)
        assert behavior.update() == expected_status
    assert "hand:" in behavior.feedback_message
    assert created["synchronizers"]["hand"].begin_calls == []
    assert capture_calls == []
    assert behavior._synchronizers == {}


def test_completed_window_accepts_fresh_pre_window_tag(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command())
    behavior._base_tags_callback(make_tag_array(make_tag()))

    assert behavior.update() == py_trees.common.Status.RUNNING
    created["synchronizers"]["hand"].input_ready = True
    assert behavior.update() == py_trees.common.Status.RUNNING
    created["synchronizers"]["hand"].ready = True

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert len(capture_calls) == 1
    reference_tags = capture_calls[0][0][7]
    assert len(reference_tags) == 1
    assert reference_tags[0].id == 23
    assert reference_tags[0].pose.header.frame_id == "odom"


def test_retry_reuses_tag_while_it_remains_fresh(monkeypatch):
    error = capture_module.ReferenceViewCaptureNotReady(
        "camera pair missing"
    )
    behavior, _, created, _ = configure_behavior(
        monkeypatch,
        capture_error=[error],
    )
    create_writer(make_command())
    behavior._base_tags_callback(make_tag_array(make_tag()))
    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer = created["synchronizers"]["hand"]
    synchronizer.input_ready = True
    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer.ready = True

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert synchronizer.begin_calls == [(1,)]

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert synchronizer.begin_calls == [(1,), (1,)]


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
    assert len(node.destroyed_subscriptions) == 2
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


def test_missing_camera_tf_blocks_collection(monkeypatch):
    behavior, _, created, capture_calls = configure_behavior(monkeypatch)
    create_writer(make_command())
    behavior._base_tags_callback(make_tag_array(make_tag()))

    assert behavior.update() == py_trees.common.Status.RUNNING
    synchronizer = created["synchronizers"]["hand"]
    synchronizer.input_ready = True
    behavior.tf_buffer.lookup_error = (
        capture_module.tf2_ros.TransformException(
            "source frame does not exist"
        )
    )

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert synchronizer.begin_calls == []
    assert "source frame does not exist" in behavior.feedback_message
    assert capture_calls == []
