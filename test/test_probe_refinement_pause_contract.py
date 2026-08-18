"""Regression guards for pausing and resuming the refinement popup."""

import inspect

from fault_detector_spot.ui.inspection.controls import InspectionControls
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_close_pauses_without_ending_server_refinement():
    source = inspect.getsource(
        InspectionControls.request_close_refinement_workflow
    )

    assert "pause_refinement_dialog" in source
    assert "OPERATION_END_REFINEMENT" not in source


def test_start_button_resumes_existing_refinement():
    source = inspect.getsource(
        InspectionControls.handle_start_probe_refinement
    )

    assert "_refinement_presentation is not None" in source
    assert "resume_refinement_dialog" in source


def test_pause_keeps_authoritative_presentation():
    source = inspect.getsource(InspectionControls.pause_refinement_dialog)

    assert "self._refinement_presentation = None" not in source
    assert "inspection_workspace_splitter.setEnabled(True)" in source


def test_emergency_stop_allows_close_without_retraction():
    stop_source = inspect.getsource(
        FinalizingInspectionControls.handle_refinement_emergency_stop
    )
    close_source = inspect.getsource(
        FinalizingInspectionControls.request_close_refinement_workflow
    )

    assert "_refinement_emergency_stop_requested = True" in stop_source
    assert "super().handle_refinement_emergency_stop()" in stop_source
    assert "_refinement_emergency_stop_requested" in close_source
    assert "super().request_close_refinement_workflow()" in close_source
    assert "return True" in close_source


def test_resuming_refinement_restores_normal_close_guard():
    source = inspect.getsource(
        FinalizingInspectionControls.resume_refinement_dialog
    )

    assert "_refinement_emergency_stop_requested = False" in source
