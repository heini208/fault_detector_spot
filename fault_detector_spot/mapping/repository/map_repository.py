"""Repository for strict map-specific navigation metadata."""

import json
from pathlib import Path
from typing import List, Optional, Union

from fault_detector_spot.mapping.model.models import (
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    Waypoint,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


class MapRepository:
    """Load and save map navigation metadata."""

    def __init__(self, maps_dir: Union[str, Path]):
        self.maps_dir = Path(maps_dir).expanduser()

    def get_map_path(self, map_id: str) -> Path:
        validate_storage_name(map_id, "map ID")
        return self.maps_dir / f"{map_id}.json"

    def exists(self, map_id: str) -> bool:
        return self.get_map_path(map_id).is_file()

    def load(self, map_id: str, validate: bool = True) -> MapDefinition:
        path = self.get_map_path(map_id)
        if not path.is_file():
            raise FileNotFoundError(f"Map metadata does not exist: {path}")
        try:
            with path.open("r", encoding="utf-8") as map_file:
                data = json.load(map_file)
        except json.JSONDecodeError as exception:
            raise ValueError(f"Invalid map JSON in {path}: {exception}") from exception
        definition = MapDefinition.from_dict(data)
        if definition.map_id != map_id:
            raise ValueError(f"Map ID mismatch in {path}: {definition.map_id}")
        if validate:
            definition.validate()
        return definition

    def save(self, map_id: str, definition: MapDefinition, validate: bool = True) -> Path:
        validate_storage_name(map_id, "map ID")
        if definition.map_id != map_id:
            raise ValueError(
                "Map ID does not match repository path: "
                f"{definition.map_id} != {map_id}"
            )
        if validate:
            definition.validate()
        path = self.get_map_path(map_id)
        content = json.dumps(definition.to_dict(), indent=2, ensure_ascii=False)
        atomic_write_text(path, content + "\n")
        return path

    def create_empty(
        self,
        map_id: str,
        display_name: Optional[str] = None,
        overwrite: bool = False,
    ) -> MapDefinition:
        path = self.get_map_path(map_id)
        if path.exists() and not overwrite:
            raise FileExistsError(f"Map metadata already exists: {path}")
        definition = MapDefinition(map_id=map_id, display_name=display_name or map_id)
        self.save(map_id, definition)
        return definition

    def list_map_ids(self) -> List[str]:
        if not self.maps_dir.is_dir():
            return []
        return sorted(path.stem for path in self.maps_dir.glob("*.json") if path.is_file())

    def get_waypoint(self, map_id: str, waypoint_id: str) -> Optional[Waypoint]:
        return self.load(map_id).get_waypoint(waypoint_id)

    def get_landmark(
        self,
        map_id: str,
        landmark_id: str,
    ) -> Optional[LocalizationLandmark]:
        return self.load(map_id).get_landmark(landmark_id)

    def get_object_approach(
        self,
        map_id: str,
        approach_id: str,
    ) -> Optional[ObjectApproach]:
        return self.load(map_id).get_object_approach(approach_id)

    def add_waypoint(self, map_id: str, waypoint: Waypoint) -> MapDefinition:
        definition = self.load(map_id)
        if definition.get_waypoint(waypoint.waypoint_id) is not None:
            raise FileExistsError(f"Waypoint already exists: {waypoint.waypoint_id}")
        definition.waypoints.append(waypoint)
        self.save(map_id, definition)
        return definition

    def add_landmark(
        self,
        map_id: str,
        landmark: LocalizationLandmark,
    ) -> MapDefinition:
        definition = self.load(map_id)
        if definition.get_landmark(landmark.landmark_id) is not None:
            raise FileExistsError(f"Landmark already exists: {landmark.landmark_id}")
        definition.localization_landmarks.append(landmark)
        self.save(map_id, definition)
        return definition

    def delete_waypoint(self, map_id: str, waypoint_id: str) -> MapDefinition:
        definition = self.load(map_id)
        if definition.get_waypoint(waypoint_id) is None:
            raise KeyError(f"Unknown waypoint: {waypoint_id}")
        definition.waypoints = [
            waypoint for waypoint in definition.waypoints
            if waypoint.waypoint_id != waypoint_id
        ]
        definition.object_approaches = [
            approach for approach in definition.object_approaches
            if approach.waypoint_id != waypoint_id
        ]
        self.save(map_id, definition)
        return definition

    def delete_landmark(self, map_id: str, landmark_id: str) -> MapDefinition:
        definition = self.load(map_id)
        if definition.get_landmark(landmark_id) is None:
            raise KeyError(f"Unknown landmark: {landmark_id}")
        definition.localization_landmarks = [
            landmark for landmark in definition.localization_landmarks
            if landmark.landmark_id != landmark_id
        ]
        self.save(map_id, definition)
        return definition
