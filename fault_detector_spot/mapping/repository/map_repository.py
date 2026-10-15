"""Repository for strict map-specific navigation metadata."""

import json
import fcntl
import math
import os
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
from threading import Lock, RLock
from typing import List, Optional, Union

from fault_detector_spot.mapping.model.models import (
    LocalizationLandmark,
    LandmarkRefinement,
    MapDefinition,
    ObjectApproach,
    Waypoint,
)
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


class _MapLock:
    def __init__(self):
        self.thread_lock = RLock()
        self.depth = 0
        self.file = None


_MAP_LOCKS = {}
_MAP_LOCKS_GUARD = Lock()


@contextmanager
def map_mutation_lock(maps_dir, map_id, *, blocking=True):
    """Serialize API/BT edits and artifact deletion across threads and processes.

    Keep the lock file after releasing it: unlinking permits two lock inodes to
    exist at once. It is deliberately outside the map's ``<id>.*`` artifacts.
    """
    validate_storage_name(map_id, "map ID")
    directory = Path(maps_dir).expanduser().resolve()
    lock_path = directory / f".{map_id}.lock"
    with _MAP_LOCKS_GUARD:
        state = _MAP_LOCKS.setdefault(lock_path, _MapLock())
    if not state.thread_lock.acquire(blocking=blocking):
        yield False
        return
    try:
        if state.depth == 0:
            directory.mkdir(parents=True, exist_ok=True)
            state.file = lock_path.open("a+")
            try:
                fcntl.flock(state.file.fileno(), fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
            except BlockingIOError:
                state.file.close()
                state.file = None
                yield False
                return
            except BaseException:
                state.file.close()
                state.file = None
                raise
        state.depth += 1
        try:
            yield True
        finally:
            state.depth -= 1
            if state.depth == 0:
                try:
                    fcntl.flock(state.file.fileno(), fcntl.LOCK_UN)
                finally:
                    state.file.close()
                    state.file = None
    finally:
        state.thread_lock.release()


def _revision(stat):
    return stat.st_ino, stat.st_mtime_ns, stat.st_size


class MapRepository:
    """Load and save map navigation metadata."""

    def __init__(self, maps_dir: Union[str, Path]):
        self.maps_dir = Path(maps_dir).expanduser()

    def get_map_path(self, map_id: str) -> Path:
        validate_storage_name(map_id, "map ID")
        return self.maps_dir / f"{map_id}.json"

    def exists(self, map_id: str) -> bool:
        return self.get_map_path(map_id).is_file()

    def revision(self, map_id: str) -> tuple:
        """Identify an atomic metadata replacement without rereading its JSON."""
        return _revision(self.get_map_path(map_id).stat())

    def load(self, map_id: str, validate: bool = True) -> MapDefinition:
        path = self.get_map_path(map_id)
        if not path.is_file():
            raise FileNotFoundError(f"Map metadata does not exist: {path}")
        try:
            with path.open("r", encoding="utf-8") as map_file:
                data = json.load(map_file)
                revision = _revision(os.fstat(map_file.fileno()))
        except json.JSONDecodeError as exception:
            raise ValueError(f"Invalid map JSON in {path}: {exception}") from exception
        definition = MapDefinition.from_dict(data)
        if definition.map_id != map_id:
            raise ValueError(f"Map ID mismatch in {path}: {definition.map_id}")
        if validate:
            definition.validate()
        # This snapshot token is repository bookkeeping, never persisted.
        definition._repository_revision = revision
        return definition

    def save(self, map_id: str, definition: MapDefinition, validate: bool = True,
             *, expected_revision=None) -> Path:
        with map_mutation_lock(self.maps_dir, map_id):
            return self._save_locked(map_id, definition, validate, expected_revision)

    def _save_locked(self, map_id, definition, validate, expected_revision):
        validate_storage_name(map_id, "map ID")
        if definition.map_id != map_id:
            raise ValueError(
                "Map ID does not match repository path: "
                f"{definition.map_id} != {map_id}"
            )
        if validate:
            definition.validate()
        path = self.get_map_path(map_id)
        expected_revision = (expected_revision if expected_revision is not None
                             else getattr(definition, "_repository_revision", None))
        if path.is_file():
            previous = self.load(map_id)
            if expected_revision != previous._repository_revision:
                raise ValueError("Map changed since it was loaded; reload before saving")
            if previous.localization_frame != definition.localization_frame:
                raise ValueError(
                    "Cannot change an existing map's localization frame; "
                    "create a new map instead"
                )
            for landmark in definition.localization_landmarks:
                saved = previous.get_landmark(landmark.landmark_id)
                if saved is not None and saved.refinement is not None:
                    if (landmark.refinement is None
                            or landmark.refinement.original_pose_map != saved.refinement.original_pose_map
                            or landmark.refinement.update_count < saved.refinement.update_count):
                        raise ValueError("Cannot discard or replace the original saved landmark pose")
        elif expected_revision is not None:
            raise ValueError("Map was removed since it was loaded; refusing to recreate it")
        content = json.dumps(definition.to_dict(), indent=2, ensure_ascii=False)
        atomic_write_text(path, content + "\n")
        definition._repository_revision = self.revision(map_id)
        return path

    def create_empty(
        self,
        map_id: str,
        display_name: Optional[str] = None,
        overwrite: bool = False,
    ) -> MapDefinition:
        with map_mutation_lock(self.maps_dir, map_id):
            return self._create_empty_locked(map_id, display_name, overwrite)

    def _create_empty_locked(self, map_id, display_name, overwrite):
        path = self.get_map_path(map_id)
        if path.exists() and not overwrite:
            raise FileExistsError(f"Map metadata already exists: {path}")
        # Adding metadata to an existing database must retain its scan frame.
        previous = self.load(map_id) if path.is_file() else None
        localization_frame = (
            previous.localization_frame if previous is not None
            else "base_link" if path.with_suffix(".db").exists()
            else "feet_center"
        )
        definition = MapDefinition(
            map_id=map_id,
            display_name=display_name or map_id,
            localization_frame=localization_frame,
        )
        self.save(map_id, definition, expected_revision=(
            previous._repository_revision if previous is not None else None
        ))
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
        with map_mutation_lock(self.maps_dir, map_id):
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
        with map_mutation_lock(self.maps_dir, map_id):
            definition = self.load(map_id)
            if definition.get_landmark(landmark.landmark_id) is not None:
                raise FileExistsError(f"Landmark already exists: {landmark.landmark_id}")
            definition.localization_landmarks.append(landmark)
            self.save(map_id, definition)
            return definition

    def delete_waypoint(self, map_id: str, waypoint_id: str) -> MapDefinition:
        with map_mutation_lock(self.maps_dir, map_id):
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
        with map_mutation_lock(self.maps_dir, map_id):
            definition = self.load(map_id)
            if definition.get_landmark(landmark_id) is None:
                raise KeyError(f"Unknown landmark: {landmark_id}")
            definition.localization_landmarks = [
                landmark for landmark in definition.localization_landmarks
                if landmark.landmark_id != landmark_id
            ]
            self.save(map_id, definition)
            return definition

    def refine_landmark(self, map_id: str, landmark_id: str,
                        expected_pose: PoseData, proposed_pose: PoseData, *,
                        expected_revision, refined_at: float, observation_ns: int,
                        minimum_interval_sec: float = 60.0) -> bool:
        """Commit an already-qualified proposal only if its snapshot is current.

        Return False for conflicts, removal, cooldown, replay, or no change.
        Geometry limits and independent localization evidence belong to the
        refinement policy; this boundary protects persistence and saved origin.
        """
        expected_pose.validate()
        proposed_pose.validate()
        metadata = LandmarkRefinement(deepcopy(expected_pose), 1, refined_at, observation_ns)
        metadata.validate()
        if (isinstance(minimum_interval_sec, bool)
                or not isinstance(minimum_interval_sec, (int, float))
                or not math.isfinite(minimum_interval_sec) or minimum_interval_sec < 0):
            raise ValueError("Landmark refinement interval must be finite and nonnegative")
        with map_mutation_lock(self.maps_dir, map_id, blocking=False) as acquired:
            if not acquired:
                return False
            try:
                definition = self.load(map_id)
            except FileNotFoundError:
                return False
            landmark = definition.get_landmark(landmark_id)
            if (definition._repository_revision != expected_revision or landmark is None
                    or landmark.pose_map != expected_pose or proposed_pose == expected_pose):
                return False
            if landmark.refinement is not None:
                saved = landmark.refinement
                if (refined_at - saved.last_refined_at < minimum_interval_sec
                        or observation_ns <= saved.last_observation_ns):
                    return False
                metadata.original_pose_map = deepcopy(saved.original_pose_map)
                metadata.update_count = saved.update_count + 1
            landmark.pose_map = deepcopy(proposed_pose)
            landmark.refinement = metadata
            self.save(map_id, definition)
            return True
