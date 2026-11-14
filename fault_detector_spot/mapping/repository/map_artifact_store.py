"""Own map database and metadata artifact deletion."""

from pathlib import Path
from typing import Iterable, Union

from fault_detector_spot.mapping.repository.map_repository import map_mutation_lock
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class MapArtifactStore:
    """Delete the complete on-disk artifact set for one map."""

    def __init__(self, maps_dir: Union[str, Path]):
        self.maps_dir = Path(maps_dir).expanduser()

    def has_database(self, map_id: str) -> bool:
        """Distinguish a saved map from an empty metadata definition."""
        validate_storage_name(map_id, "map ID")
        return (self.maps_dir / f"{map_id}.db").exists()

    def paths(self, map_id: str) -> Iterable[Path]:
        """Return existing artifacts belonging to one validated map ID."""
        validate_storage_name(map_id, "map ID")
        if not self.maps_dir.is_dir():
            return ()
        # Map IDs are literal names, including dots and glob characters.
        candidates = (
            self.maps_dir / f"{map_id}{suffix}"
            for suffix in (".json", ".db", ".db-wal", ".db-shm", ".db-journal")
        )
        return tuple(path for path in candidates if path.is_file())

    def delete(self, map_id: str) -> tuple:
        """Delete every file belonging to one map and return its paths."""
        with map_mutation_lock(self.maps_dir, map_id):
            paths = tuple(self.paths(map_id))
            if not paths:
                raise FileNotFoundError(f"Map does not exist: {map_id}")
            for path in paths:
                path.unlink()
            return paths


__all__ = ["MapArtifactStore"]
