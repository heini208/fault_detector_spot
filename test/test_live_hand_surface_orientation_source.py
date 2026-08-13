"""Regression guards for live hand-center surface calculation."""

import inspect

from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    ProbeSetupMotionStateSource,
)


def test_live_surface_orientation_uses_current_hand_depth_center():
    source = inspect.getsource(
        ProbeSetupMotionStateSource.live_hand_surface_orientation
    )

    assert "int(depth_image.width) // 2" in source
    assert "int(depth_image.height) // 2" in source
    assert "estimate_reference_surface_normal" in source
    assert "surface_aligned_probe_orientation" in source
