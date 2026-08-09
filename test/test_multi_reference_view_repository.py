"""Tests for atomic camera-specific reference-view persistence."""

import json
from dataclasses import replace

import pytest
from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.inspection.multi_reference_view_repository import (
    CapturedReferenceView,
    MultiReferenceViewRepository,
)


def make_object():
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(tag_id=23, tag_family="36h11"),
        routines=[InspectionRoutine(
            routine_id="magnetic_scan",
            display_name="Magnetic scan",
            sensor_id="bmm150",
        )],
    )


def make_capture(slot_index, camera_id, nanosec):
    rgb = Image()
    rgb.header.frame_id = f"{camera_id}_image_sensor"
    rgb.header.stamp.sec = 10
    rgb.header.stamp.nanosec = nanosec
    rgb.width = 2
    rgb.height = 1
    rgb.encoding = "rgb8"
    rgb.step = 6
    rgb.data = bytes([255, 0, 0, 0, 255, 0])

    depth = Image()
    depth.header = rgb.header
    depth.width = 2
    depth.height = 1
    depth.encoding = "16UC1"
    depth.step = 4
    depth.data = bytes([232, 3, 232, 3])

    camera_info = CameraInfo()
    camera_info.header = rgb.header
    camera_info.width = 2
    camera_info.height = 1
    camera_info.k = [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0]
    tag = TagElement()
    tag.id = 23
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = 50
    tag.pose.pose.orientation.w = 1.0

    return CapturedReferenceView(
        slot_index=slot_index,
        camera_id=camera_id,
        reference_view=ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame=rgb.header.frame_id,
        ),
        rgb_image=rgb,
        depth_image=depth,
        rgb_camera_info=camera_info,
        depth_camera_info=camera_info,
        reference_tag=tag,
        fixed_frame="odom",
    )


def test_save_and_reload_three_reference_views(tmp_path):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [
            make_capture(0, "frontleft", 100),
            make_capture(1, "hand", 200),
            make_capture(2, "back", 300),
        ],
    )

    loaded = repository.load_reference_views(
        "motor_a",
        "magnetic_scan",
    )

    assert [(item.slot_index, item.camera_id) for item in loaded] == [
        (0, "frontleft"),
        (1, "hand"),
        (2, "back"),
    ]
    definition = repository.object_repository.load("motor_a")
    routine = definition.get_routine("magnetic_scan")
    assert routine.reference_views[0].controlled_frame == (
        "frontleft_image_sensor"
    )
    assert [
        (view.view_id, view.camera_id, view.slot_index)
        for view in routine.reference_views
    ] == [
        ("slot1_frontleft", "frontleft", 0),
        ("slot2_hand", "hand", 1),
        ("slot3_back", "back", 2),
    ]
    assert all(item.reference_view.reference_dataset_path for item in loaded)
    routine_root = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    assert not (routine_root / "reference_views.yaml").exists()


def test_replacement_removes_old_camera_directories(tmp_path):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(0, "hand", 100)],
    )
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(1, "right", 200)],
    )

    loaded = repository.load_reference_views(
        "motor_a",
        "magnetic_scan",
    )
    assert [(item.slot_index, item.camera_id) for item in loaded] == [
        (1, "right")
    ]
    routine_root = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    dataset_directories = [
        child.name for child in routine_root.iterdir() if child.is_dir()
    ]
    assert len(dataset_directories) == 1
    view_directories = [
        child.name
        for child in (routine_root / dataset_directories[0]).iterdir()
        if child.is_dir()
    ]
    assert view_directories == ["slot2_right"]


def test_recapture_preserves_probe_geometry_and_clears_provenance(
    tmp_path,
):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(0, "hand", 100)],
    )
    definition = repository.object_repository.load("motor_a")
    routine = definition.get_routine("magnetic_scan")
    safe_approach_pose = PoseData.identity()
    safe_approach_pose.position.x = 0.30
    probe_pose = PoseData.identity()
    probe_pose.position.x = 0.02
    original_probe = ProbePoint(
        probe_point_id="point_a",
        display_name="Point A",
        safe_approach_pose_object=safe_approach_pose,
        probe_pose_object=probe_pose,
        target_surface_distance_m=0.01,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        measurement_duration_sec=1.0,
        aligned_preapproach_distance_m=0.08,
        reference_pixel=ImagePoint(u=1, v=0),
        reference_view_id="slot1_hand",
        sensor_path="magnetic/field",
    )
    routine.probe_points.append(original_probe)
    repository.object_repository.save(definition)

    stored = repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(0, "hand", 200)],
    )

    stored_probe = stored.get_routine(
        "magnetic_scan"
    ).probe_points[0]
    assert stored_probe == replace(
        original_probe,
        reference_pixel=None,
        reference_view_id=None,
    )


def test_failed_object_update_preserves_previous_complete_set(
    tmp_path,
    monkeypatch,
):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(0, "hand", 100)],
    )
    previous = repository.object_repository.load("motor_a")

    def fail_save(*args, **kwargs):
        raise OSError("object write failed")

    monkeypatch.setattr(
        repository.object_repository,
        "save",
        fail_save,
    )
    with pytest.raises(OSError, match="object write failed"):
        repository.save_reference_views(
            "motor_a",
            "magnetic_scan",
            [make_capture(1, "right", 200)],
        )

    restored = MultiReferenceViewRepository(
        tmp_path
    ).object_repository.load("motor_a")
    assert restored == previous
    routine_root = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    capture_sets = [
        child.name for child in routine_root.iterdir()
        if child.is_dir() and not child.name.startswith(".")
    ]
    assert len(capture_sets) == 1
    assert "000000100" in capture_sets[0]


def test_reload_rejects_calibration_metadata_mismatch(tmp_path):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    definition = repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [make_capture(0, "hand", 100)],
    )
    view = definition.get_routine(
        "magnetic_scan"
    ).reference_views[0]
    metadata_path = (
        tmp_path
        / "motor_a"
        / view.reference_dataset_path
        / "metadata.json"
    )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["rgb_depth_overlap_region"]["x"] += 1
    metadata_path.write_text(
        json.dumps(metadata),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="overlap does not match"):
        repository.load_reference_views(
            "motor_a",
            "magnetic_scan",
        )


def test_save_rejects_different_tag_observations_between_cameras(
    tmp_path,
):
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_object())
    first = make_capture(0, "frontleft", 100)
    second = make_capture(1, "hand", 200)
    second.reference_tag.pose.header.stamp.nanosec = 51

    with pytest.raises(ValueError, match="shared tag observation"):
        repository.save_reference_views(
            "motor_a",
            "magnetic_scan",
            [first, second],
        )
