"""Own map database and metadata artifact deletion."""

from pathlib import Path
from typing import Iterable, Union

from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class MapArtifactStore:
    """Delete the complete on-disk artifact set for one map."""

    def __init__(self, maps_dir: Union[str, Path]):
        self.maps_dir = Path(maps_dir).expanduser()

    def paths(self, map_id: str) -> Iterable[Path]:
        """Return existing artifacts belonging to one validated map ID."""
        validate_storage_name(map_id, "map ID")
        if not self.maps_dir.is_dir():
            return ()
        return tuple(
            path
            for path in self.maps_dir.glob(f"{map_id}.*")
            if path.is_file()
        )

    def delete(self, map_id: str) -> tuple:
        """Delete every file belonging to one map and return its paths."""
        paths = tuple(self.paths(map_id))
        if not paths:
            raise FileNotFoundError(f"Map does not exist: {map_id}")
        for path in paths:
            path.unlink()
        return paths


__all__ = ["MapArtifactStore"]
