"""Tests for atomic camera-specific reference-view persistence."""

from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
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
            probe_frame="sensor_tip",
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

    return CapturedReferenceView(
        slot_index=slot_index,
        camera_id=camera_id,
        reference_view=ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame=rgb.header.frame_id,
        ),
        rgb_image=rgb,
        depth_image=depth,
        camera_info=camera_info,
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
    assert routine.reference_view.controlled_frame == (
        "frontleft_image_sensor"
    )
    assert all(item.reference_view.reference_dataset_path for item in loaded)


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
    assert "right" in dataset_directories[0]
