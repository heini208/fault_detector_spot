"""Regression guards for hand-depth near-field diagnostics."""

import inspect

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    ProbeSetupMotionStateSource,
)


def test_orientation_failure_reports_possible_tof_near_field():
    source = inspect.getsource(
        ProbeSetupMotionStateSource.live_hand_surface_orientation
    )

    assert "gripper ToF" in source
    assert "pre-approach distance" in source


def test_surface_distance_failure_reports_possible_tof_near_field():
    source = inspect.getsource(
        ProbeSetupMotionStateSource.surface_distance_samples
    )

    assert "gripper ToF" in source
    assert "probe distance" in source
