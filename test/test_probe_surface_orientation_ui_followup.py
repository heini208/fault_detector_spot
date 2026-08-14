"""Regression guards for live surface-orientation refinement UI."""

import inspect

from fault_detector_spot.ui.inspection.probe_refinement_dialog import (
    ProbeRefinementDialog,
)


def test_alignment_page_exposes_center_window_radius():
    source = inspect.getsource(ProbeRefinementDialog._make_alignment_page)

    assert "Live hand surface fit" in source
    assert "surface_window_radius_field" in source
    assert "Center window radius:" in source


def test_calculation_applies_visible_window_to_application_api():
    source = inspect.getsource(
        ProbeRefinementDialog._handle_calculate_surface_orientation
    )

    assert "HAND_SURFACE_WINDOW_PARAMETER" in source
    assert "SetParameters.Request" in source
    assert "call_async" in source
    assert "surface_window_radius_field.value()" in source


def test_failed_recalculation_keeps_last_valid_orientation():
    source = inspect.getsource(
        ProbeRefinementDialog._handle_surface_orientation_rejected
    )

    assert "_calculated_surface_probe_orientation is None" in source
    assert "keeping last valid orientation" in source
    assert "_clear_live_surface_orientation" not in source


def test_surface_window_transport_tolerates_missing_ui_node():
    init_source = inspect.getsource(ProbeRefinementDialog.__init__)
    request_source = inspect.getsource(
        ProbeRefinementDialog._handle_calculate_surface_orientation
    )

    assert "if controls.node is not None" in init_source
    assert "/application_api/set_parameters" in init_source
    assert "client is None or not client.service_is_ready()" in request_source
    assert "AsyncParameterClient" not in init_source


def test_rejected_orientation_signal_is_optional_for_test_clients():
    source = inspect.getsource(ProbeRefinementDialog.__init__)

    assert 'getattr(' in source
    assert '"surface_orientation_rejected"' in source
    assert 'if rejected_signal is not None' in source
