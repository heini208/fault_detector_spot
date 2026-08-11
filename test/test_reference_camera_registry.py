"""Tests for selectable reference-camera slots."""

import pytest

from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERA_BY_ID,
    validate_reference_camera_slots,
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


def test_slots_preserve_empty_positions():
    assert validate_reference_camera_slots(
        ["", "hand", "back"]
    ) == ((1, "hand"), (2, "back"))


def test_slots_reject_duplicates_and_empty_selection():
    with pytest.raises(ValueError, match="more than once"):
        validate_reference_camera_slots(["hand", "hand", ""])
    with pytest.raises(ValueError, match="at least one"):
        validate_reference_camera_slots(["", "", ""])
