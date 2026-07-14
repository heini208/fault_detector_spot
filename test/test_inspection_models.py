"""Tests for inspection data models."""

import pytest

from fault_detector_spot.inspection.models import (
    InspectionObject,
    MapDefinition,
    PoseData,
    WaypointDefinition,
    WaypointReference,
)


def test_legacy_map_loads_without_objects():
    """Legacy maps default to map-relative waypoints."""
    data = {
        "waypoints": [
            {
                "name": "start",
                "pose": {
                    "position": {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.0,
                    },
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0,
                    },
                },
            }
        ],
        "landmarks": [],
    }

    map_definition = MapDefinition.from_dict(data)

    assert map_definition.objects == []
    assert map_definition.schema_version == 1
    assert (
        map_definition.waypoints[0].reference_type
        == WaypointReference.MAP
    )


def test_object_relative_waypoint_round_trip():
    """Object-relative waypoints retain their reference."""
    waypoint = WaypointDefinition(
        name="motor_front",
        pose=PoseData(),
        reference_type=WaypointReference.OBJECT,
        object_id="motor_a",
    )

    serialized = waypoint.to_dict()
    restored = WaypointDefinition.from_dict(serialized)

    assert restored == waypoint


def test_object_waypoint_requires_object_id():
    """Object-relative waypoints require an object ID."""
    waypoint = WaypointDefinition(
        name="invalid",
        pose=PoseData(),
        reference_type=WaypointReference.OBJECT,
    )

    with pytest.raises(ValueError):
        waypoint.validate()


def test_map_validates_object_references():
    """Map validation accepts valid object relationships."""
    data = {
        "landmarks": [
            {
                "name": "Tag_23",
                "tag_id": 23,
                "pose": {},
            }
        ],
        "waypoints": [
            {
                "name": "motor_front",
                "reference_type": "object",
                "object_id": "motor_a",
                "pose": {},
            }
        ],
        "objects": [
            {
                "id": "motor_a",
                "display_name": "Motor A",
                "tag_id": 23,
                "landmark_name": "Tag_23",
                "approach_waypoints": [
                    "motor_front"
                ],
            }
        ],
    }

    map_definition = MapDefinition.from_dict(data)
    map_definition.validate()

    inspection_object = map_definition.objects[0]

    assert isinstance(inspection_object, InspectionObject)
    assert inspection_object.tag_id == 23