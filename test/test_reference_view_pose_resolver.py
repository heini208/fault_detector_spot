"""Tests for capture-time reference-view pose resolution."""

import math
from copy import deepcopy

import pytest
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from fault_detector_spot.inspection.setup.reference_view_pose_resolver import (
    resolve_reference_view_pose,
)


def make_pose(
    frame_id: str,
    stamp_sec: int,
    stamp_nanosec: int,
    x: float,
    y: float,
    yaw: float,
) -> PoseStamped:
    """Create one timestamped planar pose."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = stamp_sec
    pose.header.stamp.nanosec = stamp_nanosec
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def make_rgb() -> Image:
    """Create a timestamped RGB image header."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = 200_000_000
    return image


def make_tag() -> TagElement:
    """Create a base-camera tag observation at an earlier time."""
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = 100_000_000
    tag.pose.pose.position.x = 1.0
    tag.pose.pose.orientation.w = 1.0
    return tag


class FakeTfBuffer:
    """Return configured fixed-frame poses and record requests."""

    def __init__(
        self,
        tag_in_fixed=None,
        camera_in_fixed=None,
    ):
        """Configure transformed tag and camera poses."""
        self.tag_in_fixed = tag_in_fixed or make_pose(
            "odom",
            10,
            100_000_000,
            1.0,
            2.0,
            math.pi / 2.0,
        )
        self.camera_in_fixed = camera_in_fixed or make_pose(
            "odom",
            10,
            200_000_000,
            1.0,
            3.0,
            math.pi / 2.0,
        )
        self.requests = []

    def transform(self, pose, target_frame, timeout=None):
        """Record and resolve one timestamped transform."""
        self.requests.append(
            (deepcopy(pose), target_frame, timeout)
        )
        if pose.header.frame_id == "body":
            return deepcopy(self.tag_in_fixed)
        return deepcopy(self.camera_in_fixed)


def test_resolves_camera_pose_relative_to_reference_tag():
    """The saved pose is tag-to-camera, not odom-to-camera."""
    tf_buffer = FakeTfBuffer()

    reference_view = resolve_reference_view_pose(
        tf_buffer,
        make_rgb(),
        make_tag(),
    )

    pose = reference_view.controlled_frame_pose_object
    assert pose.position.x == pytest.approx(1.0)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.position.z == pytest.approx(0.0)
    assert pose.orientation.x == pytest.approx(0.0)
    assert pose.orientation.y == pytest.approx(0.0)
    assert pose.orientation.z == pytest.approx(0.0)
    assert pose.orientation.w == pytest.approx(1.0)
    assert reference_view.controlled_frame == (
        "hand_color_image_sensor"
    )
    assert reference_view.reference_dataset_path is None


def test_uses_observation_and_rgb_timestamps_independently():
    """Each TF request retains its own non-zero timestamp."""
    tf_buffer = FakeTfBuffer()

    resolve_reference_view_pose(
        tf_buffer,
        make_rgb(),
        make_tag(),
    )

    tag_request, camera_request = tf_buffer.requests
    assert tag_request[0].header.frame_id == "body"
    assert tag_request[0].header.stamp.sec == 10
    assert tag_request[0].header.stamp.nanosec == 100_000_000
    assert camera_request[0].header.frame_id == (
        "hand_color_image_sensor"
    )
    assert camera_request[0].header.stamp.sec == 10
    assert camera_request[0].header.stamp.nanosec == 200_000_000
    assert camera_request[0].pose.position.x == 0.0
    assert camera_request[0].pose.orientation.w == 1.0
    assert tag_request[1] == "odom"
    assert camera_request[1] == "odom"


def test_resolution_does_not_modify_inputs():
    """Temporary tag and image messages remain unchanged."""
    rgb_image = make_rgb()
    tag = make_tag()
    rgb_before = deepcopy(rgb_image)
    tag_before = deepcopy(tag)

    resolve_reference_view_pose(
        FakeTfBuffer(),
        rgb_image,
        tag,
    )

    assert rgb_image == rgb_before
    assert tag == tag_before


@pytest.mark.parametrize(
    "source, message",
    [
        ("rgb_frame", "RGB image frame"),
        ("rgb_stamp", "RGB image timestamp"),
        ("tag_frame", "tag observation frame"),
        ("tag_stamp", "tag observation timestamp"),
    ],
)
def test_missing_transform_identity_is_rejected_before_tf(
    source,
    message,
):
    """Empty frames and zero times cannot become latest TF lookups."""
    rgb_image = make_rgb()
    tag = make_tag()
    if source == "rgb_frame":
        rgb_image.header.frame_id = ""
    elif source == "rgb_stamp":
        rgb_image.header.stamp.sec = 0
        rgb_image.header.stamp.nanosec = 0
    elif source == "tag_frame":
        tag.pose.header.frame_id = ""
    else:
        tag.pose.header.stamp.sec = 0
        tag.pose.header.stamp.nanosec = 0
    tf_buffer = FakeTfBuffer()

    with pytest.raises(ValueError, match=message):
        resolve_reference_view_pose(
            tf_buffer,
            rgb_image,
            tag,
        )

    assert tf_buffer.requests == []


@pytest.mark.parametrize(
    "fixed_frame, timeout, message",
    [
        ("", 0.05, "Fixed frame"),
        ("odom", -0.01, "Transform timeout"),
        ("odom", math.inf, "Transform timeout"),
    ],
)
def test_invalid_transform_configuration_is_rejected(
    fixed_frame,
    timeout,
    message,
):
    """Invalid fixed-frame resolution settings fail immediately."""
    with pytest.raises(ValueError, match=message):
        resolve_reference_view_pose(
            FakeTfBuffer(),
            make_rgb(),
            make_tag(),
            fixed_frame=fixed_frame,
            transform_timeout_sec=timeout,
        )


def test_tf_result_must_be_in_requested_fixed_frame():
    """Mismatched TF output cannot enter relative-pose math."""
    tag_in_wrong_frame = make_pose(
        "map",
        10,
        100_000_000,
        1.0,
        2.0,
        0.0,
    )

    with pytest.raises(ValueError, match="expected 'odom'"):
        resolve_reference_view_pose(
            FakeTfBuffer(tag_in_fixed=tag_in_wrong_frame),
            make_rgb(),
            make_tag(),
        )
