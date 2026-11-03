import inspect

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_abort_remains_the_only_ui_path_that_ends_refinement():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_abort_probe_refinement
    )
    assert "OPERATION_END_REFINEMENT" in source
