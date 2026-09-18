import inspect

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_normal_close_does_not_end_refinement():
    source = inspect.getsource(
        FinalizingInspectionControls.request_close_refinement_workflow
    )
    assert "OPERATION_END_REFINEMENT" not in source
    assert "Resume Probe Point Setup" in source


def test_abort_remains_the_only_ui_path_that_ends_refinement():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_abort_probe_refinement
    )
    assert "OPERATION_END_REFINEMENT" in source
