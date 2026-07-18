"""Tests for explicit global and local object pose sources."""

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.models import (
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    PoseData,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseResolver,
    ObjectPoseSource,
)


def make_map() -> MapDefinition:
    """Create one globally registered inspection object."""
    return MapDefinition(
        landmarks=[
            LandmarkDefinition(
                name="tag_7",
                tag_id=7,
                pose=PoseData(),
            )
        ],
        objects=[
            InspectionObject(
                object_id="panel",
                display_name="Panel",
                tag_id=7,
                landmark_name="tag_7",
            )
        ],
    )


def test_remembered_global_pose_is_not_probe_usable():
    """Remembered map geometry is explicitly navigation-only."""
    result = ObjectPoseResolver().resolve_global(
        make_map(),
        "panel",
    )

    assert result.source == ObjectPoseSource.REMEMBERED_MAP
    assert result.frame_id == "map"
    assert result.observation_source == "stored_map"
    assert not result.is_probe_usable


def test_live_global_pose_remains_distinct_from_live_local():
    """A live map pose cannot pass the local probe-use check."""
    marker = PoseStamped()
    marker.header.frame_id = "map"
    marker.header.stamp.sec = 10
    marker.pose.orientation.w = 1.0

    result = ObjectPoseResolver().resolve_global(
        make_map(),
        "panel",
        {7: marker},
    )

    assert result.source == ObjectPoseSource.LIVE_MAP
    assert result.observation_timestamp.sec == 10
    assert result.observation_source == "base"
    assert not result.is_probe_usable
