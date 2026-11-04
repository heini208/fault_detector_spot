"""Regression guards for transformed hand-camera clearance."""

from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.inspection.sensing.live_surface_plane import (
    MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M,
)

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ProbeSetupMotionStateSource,
)


def test_hand_camera_clearance_keeps_empirical_safety_floor():
    assert MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M == pytest.approx(0.230)
    assert MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M == (
        MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M
    )


@pytest.mark.parametrize("camera_offset_x, minimum_probe_distance", [
    (-0.10, 0.13),
    (0.10, 0.33),
    (-0.30, 0.0),
])
def test_minimum_probe_distance_uses_probe_to_camera_transform(
    camera_offset_x, minimum_probe_distance,
):
    source = ProbeSetupMotionStateSource.__new__(ProbeSetupMotionStateSource)
    source._lock = RLock()
    source._hand_depth_camera_info = SimpleNamespace(
        header=SimpleNamespace(frame_id="hand_depth"),
    )
    transform = PoseData.identity()
    transform.position.x = camera_offset_x
    source._lookup_pose = Mock(return_value=transform)

    minimum = source.minimum_aligned_probe_distance_m("probe")

    assert minimum == pytest.approx(minimum_probe_distance)
    source._lookup_pose.assert_called_once_with("probe_probe", "hand_depth")


@pytest.mark.parametrize("clearance_m", [0.200, 0.229, 0.230, 0.250])
def test_reached_alignment_has_live_camera_clearance_gate(monkeypatch, clearance_m):
    source = ProbeSetupMotionStateSource.__new__(
        ProbeSetupMotionStateSource
    )
    monkeypatch.setattr(
        source,
        "current_hand_camera_surface_clearance_m",
        lambda: clearance_m,
    )

    if clearance_m < 0.230:
        with pytest.raises(ValueError, match="required at least 0.230 m"):
            source.require_hand_camera_clearance()
    else:
        assert source.require_hand_camera_clearance() == pytest.approx(clearance_m)
