"""Tests for explicit reference-view input collection."""

import math
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement, TagElementArray
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import (
    reference_view_input_synchronizer as synchronizer_module,
)


ReferenceViewInputSynchronizer = (
    synchronizer_module.ReferenceViewInputSynchronizer
)


class FakeApproximateTimeSynchronizer:
    """Record approximate synchronization configuration."""

    def __init__(self, subscribers, queue_size, slop):
        self.subscribers = subscribers
        self.queue_size = queue_size
        self.slop = slop
        self.callback = None

    def registerCallback(self, callback):
        self.callback = callback


class FakeNode:
    """Record direct ROS subscriptions."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, *args):
        self.subscriptions.append(args)
        return args


def make_tag(tag_id=7, nanosec=100_000_000):
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_tag_array(*tags):
    message = TagElementArray()
    message.elements = list(tags)
    return message


def make_image(nanosec=100_000_000):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


@pytest.fixture
def make_synchronizer(monkeypatch):
    def fake_subscriber(
        node,
        message_type,
        topic,
        qos_profile,
    ):
        return SimpleNamespace(
            node=node,
            message_type=message_type,
            topic=topic,
            qos_profile=qos_profile,
        )

    monkeypatch.setattr(
        synchronizer_module,
        "Subscriber",
        fake_subscriber,
    )
    monkeypatch.setattr(
        synchronizer_module,
        "ApproximateTimeSynchronizer",
        FakeApproximateTimeSynchronizer,
    )

    def create(
        queue_size=7,
        image_skew=0.03,
        tag_skew=0.2,
        collection_duration=1.0,
    ):
        node = FakeNode()
        synchronizer = ReferenceViewInputSynchronizer(
            node=node,
            rgb_topic="/camera/hand/image",
            depth_topic="/depth_registered/hand/image",
            camera_info_topic="/camera/hand/camera_info",
            base_tag_topic="fault_detector/state/visible_tags",
            queue_size=queue_size,
            maximum_timestamp_skew_sec=image_skew,
            maximum_tag_timestamp_skew_sec=tag_skew,
            collection_duration_sec=collection_duration,
        )
        return synchronizer, node

    return create


def test_configures_all_reference_view_inputs(make_synchronizer):
    synchronizer, node = make_synchronizer()

    assert synchronizer.rgb_subscription.topic == "/camera/hand/image"
    assert synchronizer.depth_subscription.topic == (
        "/depth_registered/hand/image"
    )
    assert node.subscriptions[0][1] == "/camera/hand/camera_info"
    assert node.subscriptions[1][1] == (
        "fault_detector/state/visible_tags"
    )
    assert synchronizer.image_synchronizer.queue_size == 7
    assert synchronizer.image_synchronizer.slop == 0.03


def test_collection_requires_explicit_begin(make_synchronizer):
    synchronizer, _ = make_synchronizer()

    with pytest.raises(RuntimeError, match="No reference-view"):
        synchronizer.best_snapshot(7, 1)


def test_second_collection_cannot_start_while_active(
    make_synchronizer,
):
    synchronizer, _ = make_synchronizer()
    synchronizer.begin_collection(7, 1)

    with pytest.raises(RuntimeError, match="already active"):
        synchronizer.begin_collection(7, 1)


def test_cancel_collection_releases_transaction(make_synchronizer):
    synchronizer, _ = make_synchronizer()
    synchronizer.begin_collection(7, 1)

    assert synchronizer.collection_active is True
    synchronizer.cancel_collection()
    assert synchronizer.collection_active is False


@pytest.mark.parametrize(
    "queue_size,image_skew,tag_skew,duration,message",
    [
        (0, 0.05, 0.25, 1.0, "queue size"),
        (10, -0.01, 0.25, 1.0, "timestamp skew"),
        (10, 0.05, -0.01, 1.0, "tag timestamp skew"),
        (10, 0.05, 0.25, 0.0, "Collection duration"),
        (10, math.inf, 0.25, 1.0, "timestamp skew"),
    ],
)
def test_invalid_configuration_is_rejected(
    make_synchronizer,
    queue_size,
    image_skew,
    tag_skew,
    duration,
    message,
):
    with pytest.raises(ValueError, match=message):
        make_synchronizer(
            queue_size=queue_size,
            image_skew=image_skew,
            tag_skew=tag_skew,
            collection_duration=duration,
        )


def test_invalid_collection_request_is_rejected(make_synchronizer):
    synchronizer, _ = make_synchronizer()

    with pytest.raises(ValueError, match="tag ID"):
        synchronizer.begin_collection(-1, 1)
    with pytest.raises(ValueError, match="image sequence"):
        synchronizer.begin_collection(7, -1)
