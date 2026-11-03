"""Map deletion must preserve artifacts belonging to other maps."""

import pytest

from fault_detector_spot.mapping.repository.map_artifact_store import MapArtifactStore


@pytest.mark.parametrize("map_id, other_map_id", [
    ("plant", "plant.floor2"),
    ("plant.floor2", "plant"),
    ("*", "other"),
    ("plant?", "plant1"),
    ("plant[1]", "plant1"),
])
def test_delete_removes_only_exact_map_artifacts(tmp_path, map_id, other_map_id):
    suffixes = (".json", ".db", ".db-wal", ".db-shm", ".db-journal")
    selected = {tmp_path / f"{map_id}{suffix}" for suffix in suffixes}
    preserved = {tmp_path / f"{other_map_id}{suffix}" for suffix in suffixes}
    preserved.update({
        tmp_path / f"{map_id}.db.bak",
        tmp_path / f"{map_id}.notes",
        tmp_path / f".{map_id}.lock",
    })
    for path in selected | preserved:
        path.write_bytes(b"saved artifact")
    store = MapArtifactStore(tmp_path)

    assert set(store.paths(map_id)) == selected
    assert set(store.delete(map_id)) == selected

    assert all(not path.exists() for path in selected)
    assert all(path.read_bytes() == b"saved artifact" for path in preserved)


@pytest.mark.parametrize("map_id, other_map_id", [
    ("plant", "plant.floor2"),
    ("*", "other"),
    ("plant[1]", "plant1"),
])
def test_missing_map_does_not_delete_other_maps(tmp_path, map_id, other_map_id):
    other_database = tmp_path / f"{other_map_id}.db"
    other_database.write_bytes(b"other map")
    store = MapArtifactStore(tmp_path)

    with pytest.raises(FileNotFoundError, match="Map does not exist"):
        store.delete(map_id)

    assert other_database.read_bytes() == b"other map"
