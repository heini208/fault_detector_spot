"""Tests for mapless base-camera object resolution."""

import math

from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from fault_detector_spot.inspection.execution.live_object_pose_resolver import (
    LiveObjectPoseResolver,
)
from fault_detector_spot.inspection.data.models import (
    InspectionObject,
    ReferenceTag,
)
from fault_detector_spot.inspection.data.resolved_object_pose import (
    ObjectPoseState,
)


def make_object() -> InspectionObject:
    """Create a valid map-independent object."""
    return InspectionObject(
        object_id="panel",
        display_name="Panel",
        reference_tag=ReferenceTag(
            tag_id=7,
            tag_family="36h11",
        ),
    )


def make_marker(
    x: float = 1.0,
    frame_id: str = "odom",
    stamp_sec: int = 10,
    stamp_nanosec: int = 100_000_000,
) -> PoseStamped:
    """Create a timestamped marker pose."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = stamp_sec
    pose.header.stamp.nanosec = stamp_nanosec
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def test_reference_tag_pose_is_the_object_pose():
    """The rigid reference tag directly defines the object frame."""
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=7,
        observation_source="base",
    )

    assert result.state == ObjectPoseState.LIVE
    assert result.is_probe_usable
    assert result.frame_id == "odom"
    assert math.isclose(
        result.selected_pose.pose.position.x,
        1.0,
    )


def test_missing_marker_is_unavailable():
    """No base observation produces no probe reference."""
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        None,
        Time(seconds=10.2),
    )

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert not result.is_probe_usable


def test_stale_marker_is_unavailable():
    """A cached but old marker is rejected for probing."""
    result = LiveObjectPoseResolver(
        maximum_age_sec=0.25
    ).resolve(
        make_object(),
        make_marker(stamp_sec=9, stamp_nanosec=0),
        Time(seconds=10.0),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert "stale" in result.message


def test_wrong_tag_is_invalid():
    """A different tag cannot define the object frame."""
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=8,
    )

    assert result.state == ObjectPoseState.INVALID


def test_hand_observation_is_invalid():
    """Hand-camera tags are not probe-motion references."""
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=7,
        observation_source="hand",
    )

    assert result.state == ObjectPoseState.INVALID
    assert "base-camera" in result.message


def test_wrong_execution_frame_is_invalid():
    """The pose must already be transformed into odom."""
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        make_marker(frame_id="body"),
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID


def test_zero_marker_quaternion_is_invalid():
    """Invalid marker orientation cannot produce arm geometry."""
    marker = make_marker()
    marker.pose.orientation.w = 0.0
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        marker,
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID


def test_non_finite_marker_position_is_invalid():
    """NaN perception data cannot enter probe geometry."""
    marker = make_marker()
    marker.pose.position.x = math.nan
    result = LiveObjectPoseResolver().resolve(
        make_object(),
        marker,
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID
    assert "non-finite" in result.message

def test_non_normalized_marker_quaternion_is_invalid():
    """A non-unit marker quaternion cannot define an object."""
    marker = make_marker()
    marker.pose.orientation.w = 2.0

    result = LiveObjectPoseResolver().resolve(
        make_object(),
        marker,
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID
    assert "normalized" in result.message