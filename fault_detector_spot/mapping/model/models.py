"""Map-specific navigation and localization metadata models."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

from fault_detector_spot.inspection.model.models import PoseData, ReferenceTag


def _require_dict(data: Any, field_name: str) -> Dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"{field_name} must be an object")
    return data


def _require_list(data: Any, field_name: str) -> List[Any]:
    if not isinstance(data, list):
        raise ValueError(f"{field_name} must be a list")
    return data


def _require_text(value: str, field_name: str) -> None:
    if not value.strip():
        raise ValueError(f"{field_name} must not be empty")


@dataclass
class Waypoint:
    """Map-relative navigation waypoint."""

    waypoint_id: str
    display_name: str
    pose_map: PoseData

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Waypoint":
        data = _require_dict(data, "waypoint")
        return cls(
            waypoint_id=str(data["waypoint_id"]),
            display_name=str(data["display_name"]),
            pose_map=PoseData.from_dict(data["pose_map"]),
        )

    def validate(self) -> None:
        _require_text(self.waypoint_id, "Waypoint ID")
        _require_text(self.display_name, "Waypoint display name")
        self.pose_map.validate()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "waypoint_id": self.waypoint_id,
            "display_name": self.display_name,
            "pose_map": self.pose_map.to_dict(),
        }


@dataclass
class LocalizationLandmark:
    """Map-relative AprilTag used only for localization."""

    landmark_id: str
    display_name: str
    reference_tag: ReferenceTag
    pose_map: PoseData

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "LocalizationLandmark":
        data = _require_dict(data, "localization_landmark")
        return cls(
            landmark_id=str(data["landmark_id"]),
            display_name=str(data["display_name"]),
            reference_tag=ReferenceTag.from_dict(data["reference_tag"]),
            pose_map=PoseData.from_dict(data["pose_map"]),
        )

    def validate(self) -> None:
        _require_text(self.landmark_id, "Landmark ID")
        _require_text(self.display_name, "Landmark display name")
        self.reference_tag.validate()
        self.pose_map.validate()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "landmark_id": self.landmark_id,
            "display_name": self.display_name,
            "reference_tag": self.reference_tag.to_dict(),
            "pose_map": self.pose_map.to_dict(),
        }


@dataclass
class ObjectApproach:
    """Map waypoint associated with an inspection object."""

    approach_id: str
    display_name: str
    object_id: str
    waypoint_id: str

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ObjectApproach":
        data = _require_dict(data, "object_approach")
        return cls(
            approach_id=str(data["approach_id"]),
            display_name=str(data["display_name"]),
            object_id=str(data["object_id"]),
            waypoint_id=str(data["waypoint_id"]),
        )

    def validate(self) -> None:
        _require_text(self.approach_id, "Object approach ID")
        _require_text(self.display_name, "Object approach display name")
        _require_text(self.object_id, "Object approach object ID")
        _require_text(self.waypoint_id, "Object approach waypoint ID")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "approach_id": self.approach_id,
            "display_name": self.display_name,
            "object_id": self.object_id,
            "waypoint_id": self.waypoint_id,
        }


@dataclass
class MapDefinition:
    """Map-specific navigation and localization metadata."""

    map_id: str
    display_name: str
    waypoints: List[Waypoint] = field(default_factory=list)
    localization_landmarks: List[LocalizationLandmark] = field(default_factory=list)
    object_approaches: List[ObjectApproach] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MapDefinition":
        data = _require_dict(data, "map_definition")
        waypoints = _require_list(data["waypoints"], "waypoints")
        landmarks = _require_list(data["localization_landmarks"], "localization_landmarks")
        approaches = _require_list(data["object_approaches"], "object_approaches")
        return cls(
            map_id=str(data["map_id"]),
            display_name=str(data["display_name"]),
            waypoints=[Waypoint.from_dict(value) for value in waypoints],
            localization_landmarks=[LocalizationLandmark.from_dict(value) for value in landmarks],
            object_approaches=[ObjectApproach.from_dict(value) for value in approaches],
        )

    def validate(self) -> None:
        _require_text(self.map_id, "Map ID")
        _require_text(self.display_name, "Map display name")
        waypoint_ids: Set[str] = set()
        for waypoint in self.waypoints:
            waypoint.validate()
            if waypoint.waypoint_id in waypoint_ids:
                raise ValueError(f"Duplicate waypoint ID: {waypoint.waypoint_id}")
            waypoint_ids.add(waypoint.waypoint_id)
        landmark_ids: Set[str] = set()
        landmark_tags = set()
        for landmark in self.localization_landmarks:
            landmark.validate()
            if landmark.landmark_id in landmark_ids:
                raise ValueError(f"Duplicate landmark ID: {landmark.landmark_id}")
            tag_key = (landmark.reference_tag.tag_family, landmark.reference_tag.tag_id)
            if tag_key in landmark_tags:
                raise ValueError(
                    "Duplicate localization landmark tag: "
                    f"{tag_key[0]}:{tag_key[1]}"
                )
            landmark_ids.add(landmark.landmark_id)
            landmark_tags.add(tag_key)
        approach_ids: Set[str] = set()
        for approach in self.object_approaches:
            approach.validate()
            if approach.approach_id in approach_ids:
                raise ValueError(f"Duplicate object approach ID: {approach.approach_id}")
            if approach.waypoint_id not in waypoint_ids:
                raise ValueError(
                    f"Unknown waypoint {approach.waypoint_id} for approach {approach.approach_id}"
                )
            approach_ids.add(approach.approach_id)

    def validate_object_references(self, known_object_ids: Set[str]) -> None:
        for approach in self.object_approaches:
            if approach.object_id not in known_object_ids:
                raise ValueError(
                    f"Unknown inspection object {approach.object_id} for approach {approach.approach_id}"
                )

    def get_waypoint(self, waypoint_id: str) -> Optional[Waypoint]:
        return next((value for value in self.waypoints if value.waypoint_id == waypoint_id), None)

    def get_landmark(self, landmark_id: str) -> Optional[LocalizationLandmark]:
        return next(
            (value for value in self.localization_landmarks if value.landmark_id == landmark_id),
            None,
        )

    def get_object_approach(self, approach_id: str) -> Optional[ObjectApproach]:
        return next(
            (value for value in self.object_approaches if value.approach_id == approach_id),
            None,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "map_id": self.map_id,
            "display_name": self.display_name,
            "waypoints": [value.to_dict() for value in self.waypoints],
            "localization_landmarks": [value.to_dict() for value in self.localization_landmarks],
            "object_approaches": [value.to_dict() for value in self.object_approaches],
        }
