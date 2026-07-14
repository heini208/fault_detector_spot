"""Repository for map metadata and inspection objects."""

import json
from pathlib import Path
from typing import List, Optional, Union

from fault_detector_spot.inspection.models import (
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    WaypointDefinition,
)
from fault_detector_spot.inspection.repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


class MapRepository:
    """Load and save extended map metadata."""

    def __init__(self, maps_dir: Union[str, Path]):
        self.maps_dir = Path(maps_dir).expanduser()

    def get_map_path(self, map_name: str) -> Path:
        """Return the JSON path for a map."""
        validate_storage_name(map_name, "map name")
        return self.maps_dir / f"{map_name}.json"

    def exists(self, map_name: str) -> bool:
        """Return whether map metadata exists."""
        return self.get_map_path(map_name).is_file()

    def load(
        self,
        map_name: str,
        validate: bool = True,
    ) -> MapDefinition:
        """Load map metadata."""
        path = self.get_map_path(map_name)

        if not path.is_file():
            raise FileNotFoundError(
                f"Map metadata does not exist: {path}"
            )

        try:
            with path.open("r", encoding="utf-8") as map_file:
                data = json.load(map_file)
        except json.JSONDecodeError as exception:
            raise ValueError(
                f"Invalid map JSON in {path}: {exception}"
            ) from exception

        if not isinstance(data, dict):
            raise ValueError(
                f"Map metadata root must be an object: {path}"
            )

        map_definition = MapDefinition.from_dict(data)

        if validate:
            map_definition.validate()

        return map_definition

    def save(
        self,
        map_name: str,
        map_definition: MapDefinition,
        validate: bool = True,
    ) -> Path:
        """Validate and atomically save map metadata."""
        path = self.get_map_path(map_name)

        if validate:
            map_definition.validate()

        content = json.dumps(
            map_definition.to_dict(),
            indent=2,
            ensure_ascii=False,
        )
        content += "\n"

        atomic_write_text(path, content)
        return path

    def create_empty(
        self,
        map_name: str,
        overwrite: bool = False,
    ) -> MapDefinition:
        """Create empty extended map metadata."""
        path = self.get_map_path(map_name)

        if path.exists() and not overwrite:
            raise FileExistsError(
                f"Map metadata already exists: {path}"
            )

        map_definition = MapDefinition()
        self.save(map_name, map_definition)
        return map_definition

    def list_map_names(self) -> List[str]:
        """List map names that have JSON metadata."""
        if not self.maps_dir.is_dir():
            return []

        return sorted(
            path.stem
            for path in self.maps_dir.glob("*.json")
            if path.is_file()
        )

    def get_waypoint(
        self,
        map_name: str,
        waypoint_name: str,
    ) -> Optional[WaypointDefinition]:
        """Find a waypoint by name."""
        map_definition = self.load(map_name)

        return next(
            (
                waypoint
                for waypoint in map_definition.waypoints
                if waypoint.name == waypoint_name
            ),
            None,
        )

    def get_landmark(
        self,
        map_name: str,
        landmark_name: str,
    ) -> Optional[LandmarkDefinition]:
        """Find a landmark by name."""
        map_definition = self.load(map_name)

        return next(
            (
                landmark
                for landmark in map_definition.landmarks
                if landmark.name == landmark_name
            ),
            None,
        )

    def get_object(
        self,
        map_name: str,
        object_id: str,
    ) -> Optional[InspectionObject]:
        """Find an inspection object by ID."""
        map_definition = self.load(map_name)

        return next(
            (
                inspection_object
                for inspection_object in map_definition.objects
                if inspection_object.object_id == object_id
            ),
            None,
        )