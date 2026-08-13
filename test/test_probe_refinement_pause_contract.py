"""Regression guards for pausing and resuming the refinement popup."""

import inspect

from fault_detector_spot.ui.inspection.controls import InspectionControls


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
