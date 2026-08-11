"""Tests for separation of shared tags from camera collection."""

from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


class FakeNode:
    """Record direct subscriptions."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = (message_type, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription


def test_camera_collector_does_not_subscribe_to_shared_tags():
    node = FakeNode()

    ReferenceViewInputSynchronizer(
        node=node,
        rgb_topic="/camera/hand/image",
        depth_topic="/depth_registered/hand/image",
        rgb_camera_info_topic="/camera/hand/camera_info",
        depth_camera_info_topic=(
            "/depth_registered/hand/camera_info"
        ),
    )

    assert [subscription[1] for subscription in node.subscriptions] == [
        "/camera/hand/image",
        "/depth_registered/hand/image",
        "/camera/hand/camera_info",
        "/depth_registered/hand/camera_info",
    ]
