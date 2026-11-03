"""Regression guards for transformed hand-camera clearance."""

from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.shared.geometry.models import PoseData

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ProbeSetupMotionStateSource,
)


def test_hand_camera_clearance_keeps_empirical_safety_floor():
    assert MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M >= 0.230


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

    minimum = source.minimum_aligned_probe_distance_m("probe", 0.23)

    assert minimum == pytest.approx(minimum_probe_distance)
    source._lookup_pose.assert_called_once_with("probe_probe", "hand_depth")


def test_reached_alignment_has_live_camera_clearance_gate(monkeypatch):
    source = ProbeSetupMotionStateSource.__new__(
        ProbeSetupMotionStateSource
    )
    monkeypatch.setattr(
        source,
        "current_hand_camera_surface_clearance_m",
        lambda: MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M - 0.01,
    )

    with pytest.raises(ValueError, match="hand ToF near field"):
        source.require_hand_camera_clearance()

    safe_clearance_m = MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M + 0.01
    monkeypatch.setattr(
        source,
        "current_hand_camera_surface_clearance_m",
        lambda: safe_clearance_m,
    )

    assert source.require_hand_camera_clearance() == pytest.approx(
        safe_clearance_m
    )
