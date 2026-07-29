"""Tests for persistent shared tag-topic subscriptions."""

from fault_detector_spot.inspection import (
    reference_view_input_synchronizer as synchronizer_module,
)


class FakeNode:
    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = (message_type, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription


class FakeFilterSubscriber:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class FakeApproximateSynchronizer:
    def __init__(self, subscribers, queue_size, slop):
        self.subscribers = subscribers
        self.queue_size = queue_size
        self.slop = slop
        self.callback = None

    def registerCallback(self, callback):
        self.callback = callback


def test_synchronizer_subscribes_to_shared_tag_topic(monkeypatch):
    monkeypatch.setattr(
        synchronizer_module,
        "Subscriber",
        FakeFilterSubscriber,
    )
    monkeypatch.setattr(
        synchronizer_module,
        "ApproximateTimeSynchronizer",
        FakeApproximateSynchronizer,
    )
    node = FakeNode()

    synchronizer = synchronizer_module.ReferenceViewInputSynchronizer(
        node=node,
        rgb_topic="/camera/hand/image",
        depth_topic="/depth_registered/hand/image",
        camera_info_topic="/depth_registered/hand/camera_info",
        base_tag_topic="fault_detector/state/visible_tags",
    )

    assert synchronizer.base_tag_subscription is not None
    assert [subscription[1] for subscription in node.subscriptions] == [
        "/depth_registered/hand/camera_info",
        "fault_detector/state/visible_tags",
    ]
