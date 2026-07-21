"""Tests for atomic reference-view dataset storage."""

import json
from array import array
from dataclasses import replace

import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import (
    reference_dataset_repository as repository_module,
)
from fault_detector_spot.inspection.models import (
    PoseData,
    ReferenceView,
)


Repository = repository_module.ReferenceDatasetRepository


def make_view() -> ReferenceView:
    """Create a resolved view without a dataset path."""
    return ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
    )


def make_image(encoding, step, data, nanosec) -> Image:
    """Create one captured image."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    image.height = 2
    image.width = 2
    image.encoding = encoding
    image.step = step
    image.data = array("B", data)
    return image


def make_inputs():
    """Create RGB, registered depth, and CameraInfo."""
    rgb = make_image("rgb8", 6, range(12), 200_000_000)
    depth = make_image(
        "16UC1",
        4,
        [232, 3, 208, 7, 184, 11, 160, 15],
        210_000_000,
    )
    info = CameraInfo()
    info.header.frame_id = "hand_color_image_sensor"
    info.header.stamp.sec = 5
    info.height = 2
    info.width = 2
    info.k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    return rgb, depth, info


def save(repository, view=None):
    """Save one standard dataset."""
    return repository.save(
        "panel",
        "phase4",
        view or make_view(),
        *make_inputs(),
    )


def serialized(message) -> bytes:
    """Return exact CDR bytes for comparison."""
    return bytes(serialize_message(message))


def test_dataset_round_trip_and_relative_path(tmp_path):
    """One save preserves all inputs and returns a relative path."""
    repository = Repository(tmp_path)
    view = make_view()
    inputs = make_inputs()

    stored_view = repository.save(
        "panel",
        "phase4",
        view,
        *inputs,
    )
    restored = repository.load("panel", "phase4", stored_view)

    assert view.reference_dataset_path is None
    assert stored_view.reference_dataset_path == (
        "reference_datasets/phase4/10_200000000"
    )
    assert [serialized(value) for value in restored] == [
        serialized(value) for value in inputs
    ]

    dataset_path = (
        repository.object_repository.get_object_dir("panel")
        / stored_view.reference_dataset_path
    )
    metadata = json.loads(
        (dataset_path / "metadata.json").read_text()
    )
    assert metadata["object_id"] == "panel"
    assert metadata["routine_id"] == "phase4"
    assert metadata["capture_timestamp"] == {
        "sec": 10,
        "nanosec": 200_000_000,
    }


def test_failed_write_leaves_no_dataset(tmp_path, monkeypatch):
    """Failure before publication removes the staging directory."""
    repository = Repository(tmp_path)
    calls = 0

    def fail_second_write(path, content):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("write failed")
        path.write_bytes(content)

    monkeypatch.setattr(
        repository_module,
        "_write_bytes",
        fail_second_write,
    )

    with pytest.raises(OSError, match="write failed"):
        save(repository)

    routine_dir = (
        tmp_path / "panel" / "reference_datasets" / "phase4"
    )
    assert list(routine_dir.iterdir()) == []


def test_existing_dataset_is_immutable(tmp_path):
    """The same capture timestamp cannot overwrite a dataset."""
    repository = Repository(tmp_path)
    stored_view = save(repository)
    before = repository.load("panel", "phase4", stored_view)

    with pytest.raises(FileExistsError, match="already exists"):
        save(repository)

    after = repository.load("panel", "phase4", stored_view)
    assert serialized(after[0]) == serialized(before[0])


@pytest.mark.parametrize(
    "dataset_path",
    [
        None,
        "../outside",
        "reference_datasets/other/10_200000000",
    ],
)
def test_load_rejects_missing_or_unowned_paths(
    tmp_path,
    dataset_path,
):
    """A view can load only data owned by its routine."""
    view = replace(
        make_view(),
        reference_dataset_path=dataset_path,
    )

    with pytest.raises(ValueError):
        Repository(tmp_path).load("panel", "phase4", view)


def test_save_rejects_zero_rgb_timestamp(tmp_path):
    """A capture cannot use latest-time fallback semantics."""
    rgb, depth, info = make_inputs()
    rgb.header.stamp.sec = 0
    rgb.header.stamp.nanosec = 0

    with pytest.raises(ValueError, match="must not be zero"):
        Repository(tmp_path).save(
            "panel",
            "phase4",
            make_view(),
            rgb,
            depth,
            info,
        )
