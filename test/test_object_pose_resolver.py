"""Tests for inspection object pose resolution."""

import math
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.models import (
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseResolver,
    ObjectPoseState,
)

def create_pose_stamped(
    x=0.0,
    y=0.0,
    z=0.0,
    yaw=0.0,
    frame_id="map",
):
    """Create a stamped pose for resolver tests."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def create_map_definition():
    """Create map metadata containing one inspection object."""
    landmark = LandmarkDefinition(
        name="Tag_23",
        tag_id=23,
        pose=PoseData(
            position=Vector3Data(
                x=1.0,
                y=2.0,
                z=0.0,
            ),
            orientation=QuaternionData(),
        ),
    )

    inspection_object = InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        tag_id=23,
        landmark_name="Tag_23",
        marker_to_object=PoseData(),
    )

    return MapDefinition(
        landmarks=[landmark],
        objects=[inspection_object],
    )


def assert_pose_position(
    pose,
    x,
    y,
    z,
    tolerance=1e-8,
):
    """Compare the translation of a pose."""
    assert math.isclose(
        pose.pose.position.x,
        x,
        abs_tol=tolerance,
    )
    assert math.isclose(
        pose.pose.position.y,
        y,
        abs_tol=tolerance,
    )
    assert math.isclose(
        pose.pose.position.z,
        z,
        abs_tol=tolerance,
    )


def test_remembered_pose_is_used_without_live_marker():
    """Stored landmark supplies the remembered object pose."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()

    result = resolver.resolve(
        map_definition,
        "motor_a",
    )

    assert result.state == ObjectPoseState.REMEMBERED
    assert result.is_available
    assert not result.is_live
    assert result.live_pose is None

    assert_pose_position(
        result.selected_pose,
        1.0,
        2.0,
        0.0,
    )


def test_live_marker_pose_has_priority():
    """A live observation replaces the remembered selection."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()

    live_pose = create_pose_stamped(
        x=3.0,
        y=4.0,
        z=0.2,
    )

    result = resolver.resolve(
        map_definition,
        "motor_a",
        {23: live_pose},
    )

    assert result.state == ObjectPoseState.LIVE
    assert result.is_live
    assert result.remembered_pose is not None
    assert result.live_pose is not None

    assert_pose_position(
        result.selected_pose,
        3.0,
        4.0,
        0.2,
    )


def test_marker_to_object_transform_is_applied():
    """Marker-to-object translation follows marker rotation."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()

    inspection_object = map_definition.objects[0]
    inspection_object.marker_to_object = PoseData(
        position=Vector3Data(
            x=1.0,
            y=0.0,
            z=0.2,
        ),
        orientation=QuaternionData(),
    )

    live_pose = create_pose_stamped(
        x=1.0,
        y=2.0,
        yaw=math.pi / 2.0,
    )

    result = resolver.resolve(
        map_definition,
        "motor_a",
        {23: live_pose},
    )

    assert_pose_position(
        result.selected_pose,
        1.0,
        3.0,
        0.2,
    )


def test_unknown_object_is_invalid():
    """Unknown object IDs produce an invalid result."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()

    result = resolver.resolve(
        map_definition,
        "missing",
    )

    assert result.state == ObjectPoseState.INVALID
    assert not result.is_available
    assert "Unknown object" in result.message


def test_missing_landmark_is_unavailable():
    """An object without live or stored pose is unavailable."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()
    map_definition.landmarks = []

    result = resolver.resolve(
        map_definition,
        "motor_a",
    )

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert not result.is_available


def test_live_marker_in_wrong_frame_is_invalid():
    """Live poses must already be expressed in map."""
    resolver = ObjectPoseResolver()
    map_definition = create_map_definition()

    live_pose = create_pose_stamped(
        frame_id="body",
    )

    result = resolver.resolve(
        map_definition,
        "motor_a",
        {23: live_pose},
    )

    assert result.state == ObjectPoseState.INVALID
    assert not result.is_available
    assert "expected 'map'" in result.message


def test_map_validation_rejects_tag_mismatch():
    """Object and landmark tag IDs must agree."""
    map_definition = create_map_definition()
    map_definition.objects[0].tag_id = 24

    try:
        map_definition.validate()
    except ValueError as exception:
        assert "does not match landmark tag" in str(exception)
    else:
        raise AssertionError(
            "Tag mismatch should fail validation"
        )