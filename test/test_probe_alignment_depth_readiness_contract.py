"""Regression guards for aligned pre-approach depth readiness."""

import inspect

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_live_depth_clearance_is_an_alignment_approval_gate():
    approval_source = inspect.getsource(ProbeRefinementController.approve)
    motion_source = inspect.getsource(
        ProbeRefinementController.handle_terminal_status
    )

    assert "_require_live_alignment_camera_clearance()" in approval_source
    assert "_require_live_alignment_camera_clearance()" not in motion_source


def test_alignment_ui_reports_depth_readiness_separately():
    source = inspect.getsource(
        FinalizingInspectionControls._refresh_alignment_depth_status
    )

    assert "registered depth verified" in source
    assert "registered depth NOT verified" in source
    assert "depth check pending approval" in source
