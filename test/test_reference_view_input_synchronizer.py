"""Tests for synchronized reference-view sensor inputs."""

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
        """Store synchronizer arguments."""
        self.subscribers = subscribers
        self.queue_size = queue_size
        self.slop = slop
        self.callback = None

    def registerCallback(self, callback):
        """Store the synchronized image callback."""
        self.callback = callback


class FakeNode:
    """Record direct ROS subscriptions."""

    def __init__(self):
        """Initialize recorded subscriptions."""
        self.subscriptions = []

    def create_subscription(self, *args):
        """Record and return a subscription token."""
        self.subscriptions.append(args)
        return args


def make_tag(tag_id=7, stamp_nanosec=0) -> TagElement:
    """Create one base-camera tag observation."""
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = stamp_nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_tag_array(*tags) -> TagElementArray:
    """Create one published base-tag state message."""
    message = TagElementArray()
    message.elements = list(tags)
    return message


@pytest.fixture
def make_synchronizer(monkeypatch):
    """Create a synchronizer with observable ROS resources."""
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

    def create(queue_size=7, skew=0.03):
        node = FakeNode()
        synchronizer = ReferenceViewInputSynchronizer(
            node=node,
            rgb_topic="/camera/hand/image",
            depth_topic="/depth_registered/hand/image",
            camera_info_topic="/camera/hand/camera_info",
            base_tag_topic="fault_detector/state/visible_tags",
            queue_size=queue_size,
            maximum_timestamp_skew_sec=skew,
        )
        return synchronizer, node

    return create


def test_configures_camera_and_base_tag_inputs(make_synchronizer):
    """The requested topics use the correct synchronization roles."""
    synchronizer, node = make_synchronizer()

    assert synchronizer.rgb_subscription.topic == (
        "/camera/hand/image"
    )
    assert synchronizer.depth_subscription.topic == (
        "/depth_registered/hand/image"
    )
    assert node.subscriptions[0][1] == (
        "/camera/hand/camera_info"
    )
    assert node.subscriptions[1][1] == (
        "fault_detector/state/visible_tags"
    )
    assert synchronizer.image_synchronizer.subscribers == [
        synchronizer.rgb_subscription,
        synchronizer.depth_subscription,
    ]
    assert synchronizer.image_synchronizer.queue_size == 7
    assert synchronizer.image_synchronizer.slop == 0.03


def test_snapshot_requires_camera_images_and_selected_tag(
    make_synchronizer,
):
    """A snapshot is exposed only when every required input exists."""
    synchronizer, _ = make_synchronizer()
    rgb_image = Image()
    depth_image = Image()
    camera_info = CameraInfo()

    synchronizer._synchronized_images_callback(
        rgb_image,
        depth_image,
    )
    assert synchronizer.snapshot(7) is None

    synchronizer._camera_info_callback(camera_info)
    assert synchronizer.snapshot(7) is None

    synchronizer._base_tags_callback(
        make_tag_array(make_tag(7))
    )
    assert synchronizer.snapshot(7) is not None
    assert synchronizer.snapshot(8) is None


def test_snapshot_selects_tag_and_is_isolated(make_synchronizer):
    """The selected tag and camera inputs are defensive copies."""
    synchronizer, _ = make_synchronizer()
    rgb_image = Image()
    depth_image = Image()
    camera_info = CameraInfo()
    rgb_image.width = 640
    tag = make_tag(7)

    synchronizer._camera_info_callback(camera_info)
    synchronizer._base_tags_callback(make_tag_array(tag))
    synchronizer._synchronized_images_callback(
        rgb_image,
        depth_image,
    )
    snapshot = synchronizer.snapshot(7)

    assert snapshot[0] == rgb_image
    assert snapshot[1] == depth_image
    assert snapshot[2] == camera_info
    assert snapshot[3] == tag
    snapshot[0].width = 1
    snapshot[3].id = 8
    assert synchronizer.snapshot(7)[0].width == 640
    assert synchronizer.snapshot(7)[3].id == 7


def test_snapshot_can_require_images_received_after_a_command(
    make_synchronizer,
):
    """Sequence gating excludes the pair cached before capture starts."""
    synchronizer, _ = make_synchronizer()
    synchronizer._camera_info_callback(CameraInfo())
    synchronizer._base_tags_callback(make_tag_array(make_tag(7)))

    assert synchronizer.image_sequence == 0
    synchronizer._synchronized_images_callback(Image(), Image())
    assert synchronizer.image_sequence == 1
    assert synchronizer.snapshot(7, minimum_image_sequence=2) is None

    synchronizer._synchronized_images_callback(Image(), Image())
    assert synchronizer.image_sequence == 2
    assert synchronizer.snapshot(7, minimum_image_sequence=2) is not None


def test_older_tag_update_does_not_replace_newer_observation(
    make_synchronizer,
):
    """Repeated state publication cannot move tag time backwards."""
    synchronizer, _ = make_synchronizer()
    synchronizer._camera_info_callback(CameraInfo())
    synchronizer._synchronized_images_callback(Image(), Image())
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(7, 200_000_000))
    )
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(7, 100_000_000))
    )

    assert (
        synchronizer.snapshot(7)[3].pose.header.stamp.nanosec
        == 200_000_000
    )


def test_missing_tag_is_removed_by_next_visible_state(
    make_synchronizer,
):
    """An absent tag cannot remain capturable from stale local state."""
    synchronizer, _ = make_synchronizer()
    synchronizer._camera_info_callback(CameraInfo())
    synchronizer._synchronized_images_callback(Image(), Image())
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(7, 200_000_000))
    )

    assert synchronizer.snapshot(7) is not None
    synchronizer._base_tags_callback(make_tag_array())
    assert synchronizer.snapshot(7) is None


@pytest.mark.parametrize(
    "queue_size, skew, message",
    [
        (0, 0.05, "queue size"),
        (10, -0.01, "timestamp skew"),
        (10, math.inf, "timestamp skew"),
    ],
)
def test_invalid_configuration_is_rejected(
    make_synchronizer,
    queue_size,
    skew,
    message,
):
    """Invalid timing configuration fails before subscribing."""
    with pytest.raises(ValueError, match=message):
        make_synchronizer(queue_size=queue_size, skew=skew)


def test_negative_reference_tag_id_is_rejected(make_synchronizer):
    """A snapshot cannot target an invalid tag identity."""
    synchronizer, _ = make_synchronizer()

    with pytest.raises(ValueError, match="tag ID"):
        synchronizer.snapshot(-1)


def test_negative_minimum_image_sequence_is_rejected(make_synchronizer):
    """Sequence gating cannot use an invalid lower bound."""
    synchronizer, _ = make_synchronizer()

    with pytest.raises(ValueError, match="Minimum image sequence"):
        synchronizer.snapshot(7, minimum_image_sequence=-1)
