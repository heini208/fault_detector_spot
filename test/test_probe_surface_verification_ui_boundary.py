"""Tests for the UI boundary around standalone close-surface movement."""

import inspect

from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.inspection.controls import InspectionControls
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_surface_distance_button_delegates_to_ui_operation_boundary():
    source = inspect.getsource(
        InspectionControls.handle_test_surface_distance
    )

    assert "execute_probe_surface_verification" in source
    assert "measure_probe_surface_distance" not in source
    assert "aggregate_surface_distance_samples" not in source


def test_ui_submits_generic_close_surface_operation_without_setup_client():
    source = inspect.getsource(
        Fault_Detector_UI.execute_probe_surface_verification
    )

    assert "INTENT_MOVE_CLOSE_TO_SURFACE" in source
    assert "target_surface_distance_m" in source
    assert "execute_operation" in source
    assert "probe_setup_client" not in source
    assert "context_id" not in source


def test_finalization_uses_generic_command_result_not_old_verification_state():
    source = inspect.getsource(
        FinalizingInspectionControls._update_save_probe_point_state
    )

    assert "_surface_result_current" in source
    assert "SURFACE_VERIFICATION_CONVERGED" not in source
