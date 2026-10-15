"""Refinement persistence, snapshot conflicts, and offline writer coordination."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import select
import subprocess
import sys
from threading import Event

import pytest

from fault_detector_spot.inspection.model.models import ReferenceTag
from fault_detector_spot.mapping.model.models import (
    LandmarkRefinement, LocalizationLandmark, Waypoint,
)
from fault_detector_spot.mapping.repository.map_artifact_store import MapArtifactStore
from fault_detector_spot.mapping.repository.map_repository import (
    MapRepository, map_mutation_lock,
)
from fault_detector_spot.shared.geometry.models import PoseData


def pose(x=0.0):
    value = PoseData.identity()
    value.position.x = x
    return value


@pytest.fixture
def repository(tmp_path):
    repository = MapRepository(tmp_path)
    repository.create_empty("plant")
    repository.add_landmark("plant", LocalizationLandmark(
        "Tag_7", "Marker", ReferenceTag(tag_id=7, tag_family="36h11"), pose(),
    ))
    return repository


def refine(repository, x=0.01, *, stamp=100, now=1000.0, expected=None, revision=None):
    landmark = repository.load("plant").get_landmark("Tag_7")
    return repository.refine_landmark(
        "plant", "Tag_7", expected if expected is not None else landmark.pose_map,
        pose(x), expected_revision=revision if revision is not None else repository.revision("plant"),
        refined_at=now, observation_ns=stamp,
    )


def test_unrefined_landmark_json_stays_compatible(repository):
    stored = repository.load("plant").get_landmark("Tag_7")
    encoded = stored.to_dict()
    assert "refinement" not in encoded
    assert LocalizationLandmark.from_dict(encoded) == stored
    assert stored.refinement is None


def test_first_and_subsequent_refinements_preserve_original_pose(repository):
    assert refine(repository)
    first = repository.load("plant").get_landmark("Tag_7")
    assert first.pose_map == pose(0.01)
    assert first.refinement == LandmarkRefinement(pose(), 1, 1000.0, 100)
    assert refine(repository, 0.02, stamp=101, now=1060.0)
    second = repository.load("plant").get_landmark("Tag_7")
    assert second.pose_map == pose(0.02)
    assert second.refinement == LandmarkRefinement(pose(), 2, 1060.0, 101)
    assert LocalizationLandmark.from_dict(second.to_dict()) == second
    assert "_repository_revision" not in json.loads(repository.get_map_path("plant").read_text())


@pytest.mark.parametrize("now,stamp", [(1059.99, 101), (900.0, 101), (1060.0, 100), (1060.0, 99)])
def test_cooldown_clock_regression_and_replay_do_not_write(repository, now, stamp):
    assert refine(repository)
    before = repository.get_map_path("plant").read_bytes()
    assert not refine(repository, 0.02, now=now, stamp=stamp)
    assert repository.get_map_path("plant").read_bytes() == before


def test_changed_landmark_pose_and_no_change_do_not_write(repository):
    revision = repository.revision("plant")
    assert not refine(repository, expected=pose(0.005))
    assert not refine(repository, x=0.0)
    assert repository.revision("plant") == revision


def test_concurrent_api_change_is_preserved_and_stale_proposal_rejected(repository):
    revision = repository.revision("plant")
    other = MapRepository(repository.maps_dir)
    other.add_waypoint("plant", Waypoint("door", "Door", pose(2.0)))
    assert not refine(repository, revision=revision)
    definition = repository.load("plant")
    assert definition.get_waypoint("door") is not None
    assert definition.get_landmark("Tag_7").pose_map == pose()


def test_removed_landmark_and_deleted_map_are_not_recreated(repository):
    revision = repository.revision("plant")
    repository.delete_landmark("plant", "Tag_7")
    assert not repository.refine_landmark(
        "plant", "Tag_7", pose(), pose(0.01), expected_revision=revision,
        refined_at=1000.0, observation_ns=100,
    )
    repository.get_map_path("plant").unlink()
    assert not repository.refine_landmark(
        "plant", "Tag_7", pose(), pose(0.01), expected_revision=revision,
        refined_at=1000.0, observation_ns=100,
    )
    assert not repository.exists("plant")


def test_artifact_deletion_excludes_refinement_and_prevents_recreation(repository, monkeypatch):
    store = MapArtifactStore(repository.maps_dir)
    database = repository.maps_dir / "plant.db"
    database.write_bytes(b"offline map artifact")
    revision = repository.revision("plant")
    enumerating, release = Event(), Event()
    original_paths = store.paths

    def paused_paths(map_id):
        # Pause inside deletion, after it must have acquired the shared lock.
        enumerating.set()
        assert release.wait(3.0)
        return original_paths(map_id)

    monkeypatch.setattr(store, "paths", paused_paths)

    def stale_refinement():
        return repository.refine_landmark(
            "plant", "Tag_7", pose(), pose(0.01), expected_revision=revision,
            refined_at=1000.0, observation_ns=100,
        )

    with ThreadPoolExecutor(max_workers=1) as executor:
        deleting = executor.submit(store.delete, "plant")
        try:
            assert enumerating.wait(3.0)
            assert not stale_refinement()
            assert repository.revision("plant") == revision
        finally:
            release.set()
        removed = deleting.result(timeout=3.0)

    assert set(removed) == {repository.get_map_path("plant"), database}
    assert not stale_refinement()
    assert not repository.exists("plant")
    assert not database.exists()
    assert (repository.maps_dir / ".plant.lock").is_file()


def test_stale_loaded_definition_cannot_undo_refinement(repository):
    stale = repository.load("plant")
    assert refine(repository)
    stale.display_name = "Outdated rename"
    with pytest.raises(ValueError, match="changed since"):
        repository.save("plant", stale)
    assert repository.load("plant").get_landmark("Tag_7").refinement.update_count == 1


def test_saved_original_cannot_be_replaced_or_removed(repository):
    assert refine(repository)
    for mutate in (
        lambda landmark: setattr(landmark, "refinement", None),
        lambda landmark: setattr(landmark.refinement, "original_pose_map", pose(3.0)),
    ):
        definition = repository.load("plant")
        mutate(definition.get_landmark("Tag_7"))
        with pytest.raises(ValueError, match="original saved"):
            repository.save("plant", definition)
    assert repository.load("plant").get_landmark("Tag_7").refinement.original_pose_map == pose()


def test_two_repository_instances_serialize_threaded_api_updates(repository):
    def add(index):
        other = MapRepository(repository.maps_dir)
        other.add_waypoint("plant", Waypoint(str(index), str(index), pose(index)))
    with ThreadPoolExecutor(max_workers=6) as executor:
        list(executor.map(add, range(30)))
    assert {item.waypoint_id for item in repository.load("plant").waypoints} == {
        str(index) for index in range(30)
    }


def test_refinement_does_not_wait_for_another_thread_writer(repository):
    locked, release = Event(), Event()
    def writer():
        with map_mutation_lock(repository.maps_dir, "plant"):
            locked.set()
            assert release.wait(3.0)
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(writer)
        try:
            assert locked.wait(3.0)
            assert not refine(repository)
        finally:
            release.set()
        pending.result(timeout=3.0)
    assert refine(repository)


def test_refinement_does_not_wait_for_another_process_writer(repository):
    # A plain isolated Python helper holds flock; no ROS initialization occurs.
    script = (
        "import fcntl,sys; "
        "stream=open(sys.argv[1],'a+'); "
        "fcntl.flock(stream.fileno(),fcntl.LOCK_EX); "
        "print('locked',flush=True); sys.stdin.read(1)"
    )
    process = subprocess.Popen(
        [sys.executable, "-c", script, str(repository.maps_dir / ".plant.lock")],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert select.select([process.stdout], [], [], 3.0)[0]
        assert process.stdout.readline().strip() == "locked"
        assert not refine(repository)
    finally:
        process.communicate("x", timeout=3.0)
    assert process.returncode == 0
    assert refine(repository)


@pytest.mark.parametrize("field,value", [
    ("update_count", 0), ("update_count", True), ("update_count", 1.5),
    ("last_refined_at", float("nan")), ("last_refined_at", 0),
    ("last_refined_at", "100"), ("last_observation_ns", 0),
    ("last_observation_ns", 1.5), ("last_observation_ns", True),
])
def test_malformed_refinement_metadata_is_rejected(field, value):
    metadata = LandmarkRefinement(pose(), 1, 1000.0, 100)
    data = deepcopy(metadata.to_dict())
    data[field] = value
    with pytest.raises(ValueError):
        LandmarkRefinement.from_dict(data).validate()
