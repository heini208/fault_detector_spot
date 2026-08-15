"""Regression guards for manual alignment camera-clearance recovery."""

import inspect

from fault_detector_spot.ui.inspection.probe_refinement_dialog import (
    ProbeRefinementDialog,
)


def test_alignment_back_away_uses_sensor_frame_and_negative_probe_x():
    source = inspect.getsource(
        ProbeRefinementDialog._handle_back_away_from_surface
    )

    assert 'findData("sensor")' in source
    assert 'handle_refine_pose("alignment", "back")' in source


def test_alignment_page_exposes_back_away_button():
    source = inspect.getsource(ProbeRefinementDialog._make_alignment_page)

    assert "Back Away from Surface" in source
    assert "Camera clearance recovery" in source
