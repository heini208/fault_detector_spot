"""Tests for mapless base-camera object resolution."""

import math

from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from fault_detector_spot.inspection.live_object_pose_resolver import (
    LiveObjectPoseResolver,
)
from fault_detector_spot.inspection.models import (
    ObjectDefinition,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
)


def make_object(offset_x: float = 0.0) -> ObjectDefinition:
    """Create a valid portable object."""
    return ObjectDefinition(
        object_id="panel",
        display_name="Panel",
        tag_id=7,
        tag_family="36h11",
        marker_to_object=PoseData(
            position=Vector3Data(x=offset_x),
            orientation=QuaternionData(w=1.0),
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


def test_identity_marker_to_object_transform():
    """Identity calibration preserves the marker pose."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=7,
        observation_source="base",
    )

    assert result.state == ObjectPoseState.LIVE
    assert result.is_probe_usable
    assert result.frame_id == "odom"
    assert result.observation_source == "base"
    assert math.isclose(
        result.selected_pose.pose.position.x,
        1.0,
    )


def test_marker_to_object_transform_is_applied():
    """Object calibration is composed after the marker pose."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(offset_x=0.25),
        make_marker(x=1.0),
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.LIVE
    assert math.isclose(
        result.selected_pose.pose.position.x,
        1.25,
    )


def test_missing_marker_is_unavailable():
    """No base observation produces no probe reference."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(),
        None,
        Time(seconds=10.2),
    )

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert not result.is_probe_usable


def test_stale_marker_is_unavailable():
    """A cached but old marker is rejected for probing."""
    resolver = LiveObjectPoseResolver(maximum_age_sec=0.25)
    result = resolver.resolve(
        make_object(),
        make_marker(stamp_sec=9, stamp_nanosec=0),
        Time(seconds=10.0),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert "stale" in result.message


def test_wrong_tag_is_invalid():
    """A different tag ID cannot define the object frame."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=8,
    )

    assert result.state == ObjectPoseState.INVALID


def test_hand_observation_is_invalid():
    """Hand-camera tags are never probe-motion references."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(),
        make_marker(),
        Time(seconds=10.2),
        observed_tag_id=7,
        observation_source="hand",
    )

    assert result.state == ObjectPoseState.INVALID
    assert "base-camera" in result.message


def test_wrong_execution_frame_is_invalid():
    """The local pose must already be transformed into odom."""
    resolver = LiveObjectPoseResolver()
    result = resolver.resolve(
        make_object(),
        make_marker(frame_id="body"),
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID


def test_zero_marker_quaternion_is_invalid():
    """Invalid marker orientation cannot produce arm geometry."""
    resolver = LiveObjectPoseResolver()
    marker = make_marker()
    marker.pose.orientation.w = 0.0
    result = resolver.resolve(
        make_object(),
        marker,
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID


def test_non_finite_marker_position_is_invalid():
    """NaN perception data cannot enter probe geometry."""
    resolver = LiveObjectPoseResolver()
    marker = make_marker()
    marker.pose.position.x = math.nan
    result = resolver.resolve(
        make_object(),
        marker,
        Time(seconds=10.2),
        observed_tag_id=7,
    )

    assert result.state == ObjectPoseState.INVALID
    assert "non-finite" in result.message
