"""Regression guards for the alignment-stage orientation workflow."""

import inspect

from fault_detector_spot.ui.inspection.controls import InspectionControls


def test_alignment_controls_offer_tag_and_surface_modes():
    source = inspect.getsource(InspectionControls._create_reference_widgets)

    assert "ALIGNMENT_ORIENTATION_TAG" in source
    assert "ALIGNMENT_ORIENTATION_CALCULATED_SURFACE" in source
    assert "Calculate Hand-Facing Surface" in source
    assert "Orient to Calculated Surface" in source


def test_surface_orientation_is_requested_from_backend():
    source = inspect.getsource(
        InspectionControls.handle_calculate_hand_surface_orientation
    )

    assert "calculate_surface_orientation" in source


def test_alignment_motion_carries_mode_and_orientation_only_flag():
    source = inspect.getsource(InspectionControls._send_alignment_motion)

    assert "alignment_orientation_mode" in source
    assert "orientation_only" in source
    assert "calculated_surface_orientation_object" in source
