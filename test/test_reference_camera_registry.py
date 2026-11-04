"""Tests for the complete reference-camera registry."""

from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERA_BY_ID,
    all_reference_camera_slots,
)


def test_registry_matches_spot_camera_topic_families():
    assert set(REFERENCE_CAMERA_BY_ID) == {
        "frontleft",
        "frontright",
        "left",
        "right",
        "back",
        "hand",
    }
    for camera_id, camera in REFERENCE_CAMERA_BY_ID.items():
        assert camera.rgb_topic == f"/camera/{camera_id}/image"
        assert camera.depth_topic == (
            f"/depth_registered/{camera_id}/image"
        )
        assert camera.rgb_camera_info_topic == (
            f"/camera/{camera_id}/camera_info"
        )
        assert camera.depth_camera_info_topic == (
            f"/depth_registered/{camera_id}/camera_info"
        )


def test_complete_capture_uses_all_six_cameras_in_registry_order():
    assert all_reference_camera_slots() == (
        (0, "frontleft"),
        (1, "frontright"),
        (2, "left"),
        (3, "right"),
        (4, "back"),
        (5, "hand"),
    )
