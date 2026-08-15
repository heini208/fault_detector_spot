"""Regression guards for transformed hand-camera clearance."""

import inspect

import pytest

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ProbeSetupMotionStateSource,
)


def test_hand_camera_clearance_keeps_empirical_safety_floor():
    assert MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M >= 0.230


def test_minimum_probe_distance_uses_probe_to_camera_transform():
    source = inspect.getsource(
        ProbeSetupMotionStateSource.minimum_aligned_probe_distance_m
    )

    assert "sensor_probe_frame" in source
    assert "camera_frame" in source
    assert "probe_to_camera.position.x" in source


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
