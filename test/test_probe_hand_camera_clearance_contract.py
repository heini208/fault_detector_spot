"""Regression guards for transformed hand-camera clearance."""

import inspect

import pytest

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M,
    ProbeSetupMotionStateSource,
)


def test_hand_camera_clearance_defaults_to_260_mm():
    assert MINIMUM_HAND_CAMERA_SURFACE_CLEARANCE_M == 0.260


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
        lambda: 0.200,
    )

    with pytest.raises(ValueError, match="hand ToF near field"):
        source.require_hand_camera_clearance()

    monkeypatch.setattr(
        source,
        "current_hand_camera_surface_clearance_m",
        lambda: 0.270,
    )

    assert source.require_hand_camera_clearance() == pytest.approx(0.270)


def test_depth_failure_explains_how_to_recover(monkeypatch):
    source = ProbeSetupMotionStateSource.__new__(
        ProbeSetupMotionStateSource
    )

    def unavailable():
        raise ValueError("No valid registered hand depth")

    monkeypatch.setattr(
        source,
        "current_hand_camera_surface_clearance_m",
        unavailable,
    )

    with pytest.raises(
        ValueError,
        match="increase the aligned pre-approach distance",
    ):
        source.require_hand_camera_clearance()
