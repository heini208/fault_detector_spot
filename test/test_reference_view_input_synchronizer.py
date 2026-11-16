"""Tests for raw reference-view input collection."""

import math

import pytest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


class FakeSubscription:
    """Provide a weak-referenceable subscription for real callback groups."""

    def __init__(self, **attributes):
        self.__dict__.update(attributes)


class FakeNode:
    """Record direct ROS subscriptions."""

    def __init__(self):
        self.subscriptions = []
        self.default_callback_group = MutuallyExclusiveCallbackGroup()

    def create_subscription(
        self,
        message_type,
        topic,
        callback,
        qos_profile,
        *,
        callback_group=None,
    ):
        group = callback_group or self.default_callback_group
        subscription = FakeSubscription(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
            callback_group=group,
        )
        group.add_entity(subscription)
        self.subscriptions.append(subscription)
        return subscription


def make_synchronizer(
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


def test_rgb_requests_reliable_delivery_without_latched_history():
    synchronizer, node = make_synchronizer()

    rgb_qos = synchronizer.rgb_subscription.qos_profile
    assert rgb_qos.reliability == ReliabilityPolicy.RELIABLE
    assert rgb_qos.durability == DurabilityPolicy.VOLATILE
    assert rgb_qos.depth == 5
    for subscription in node.subscriptions[1:]:
        assert subscription.qos_profile == qos_profile_sensor_data


def test_camera_callbacks_are_independent_of_default_group_and_other_cameras():
    first, node = make_synchronizer()
    second = ReferenceViewInputSynchronizer(
        node, "/other/rgb", "/other/depth", "/other/rgb_info", "/other/depth_info",
    )
    unrelated = node.create_subscription(object, "/unrelated", lambda _: None, 10)
    default_group = unrelated.callback_group
    first_group = first.rgb_subscription.callback_group
    second_group = second.rgb_subscription.callback_group

    assert default_group.beginning_execution(unrelated)
    try:
        assert first_group.beginning_execution(first.depth_subscription)
        try:
            # A busy camera still serializes its own callbacks, but does not
            # prevent a different camera from collecting its RGB frame.
            assert not first_group.can_execute(first.rgb_subscription)
            assert second_group.beginning_execution(second.rgb_subscription)
            second_group.ending_execution(second.rgb_subscription)
        finally:
            first_group.ending_execution(first.depth_subscription)
        assert first_group.can_execute(first.rgb_subscription)
    finally:
        default_group.ending_execution(unrelated)


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
    "image_skew,duration,message",
    [
        (-0.01, 1.0, "timestamp skew"),
        (0.05, 0.0, "Collection duration"),
        (math.inf, 1.0, "timestamp skew"),
    ],
)
def test_invalid_configuration_is_rejected(
    image_skew,
    duration,
    message,
):
    with pytest.raises(ValueError, match=message):
        make_synchronizer(
            image_skew=image_skew,
            collection_duration=duration,
        )


def test_invalid_collection_request_is_rejected():
    synchronizer, _ = make_synchronizer()

    with pytest.raises(ValueError, match="input sequence"):
        synchronizer.begin_collection(-1)
