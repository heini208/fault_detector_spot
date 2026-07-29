"""Tests for raw reference-view input collection."""

import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


class FakeNode:
    """Record direct ROS subscriptions."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(
        self,
        message_type,
        topic,
        callback,
        qos_profile,
    ):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
        )
        self.subscriptions.append(subscription)
        return subscription


def make_synchronizer(
    queue_size=7,
    image_skew=0.03,
    collection_duration=1.0,
):
    node = FakeNode()
    synchronizer = ReferenceViewInputSynchronizer(
        node=node,
        rgb_topic="/camera/hand/image",
        depth_topic="/depth_registered/hand/image",
        rgb_camera_info_topic="/camera/hand/camera_info",
        depth_camera_info_topic=(
            "/depth_registered/hand/camera_info"
        ),
        queue_size=queue_size,
        maximum_timestamp_skew_sec=image_skew,
        collection_duration_sec=collection_duration,
    )
    return synchronizer, node


def test_configures_raw_images_and_both_calibrations():
    synchronizer, node = make_synchronizer()

    assert [subscription.topic for subscription in node.subscriptions] == [
        "/camera/hand/image",
        "/depth_registered/hand/image",
        "/camera/hand/camera_info",
        "/depth_registered/hand/camera_info",
    ]
    assert synchronizer.rgb_subscription is node.subscriptions[0]
    assert synchronizer.depth_subscription is node.subscriptions[1]


def test_collection_requires_explicit_begin():
    synchronizer, _ = make_synchronizer()

    with pytest.raises(RuntimeError, match="No reference-view"):
        synchronizer.best_snapshot(1)


def test_second_collection_cannot_start_while_active():
    synchronizer, _ = make_synchronizer()
    synchronizer.begin_collection(1)

    with pytest.raises(RuntimeError, match="already active"):
        synchronizer.begin_collection(1)


def test_cancel_collection_releases_transaction():
    synchronizer, _ = make_synchronizer()
    synchronizer.begin_collection(1)

    assert synchronizer.collection_active is True
    synchronizer.cancel_collection()
    assert synchronizer.collection_active is False


@pytest.mark.parametrize(
    "queue_size,image_skew,duration,message",
    [
        (0, 0.05, 1.0, "queue size"),
        (10, -0.01, 1.0, "timestamp skew"),
        (10, 0.05, 0.0, "Collection duration"),
        (10, math.inf, 1.0, "timestamp skew"),
    ],
)
def test_invalid_configuration_is_rejected(
    queue_size,
    image_skew,
    duration,
    message,
):
    with pytest.raises(ValueError, match=message):
        make_synchronizer(
            queue_size=queue_size,
            image_skew=image_skew,
            collection_duration=duration,
        )


def test_invalid_collection_request_is_rejected():
    synchronizer, _ = make_synchronizer()

    with pytest.raises(ValueError, match="input sequence"):
        synchronizer.begin_collection(-1)
