"""Pausing the live refinement workflow preserves server-owned progress."""

from unittest.mock import Mock

import pytest

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from test_probe_point_guided_workflow import FakeUI, application
from test_probe_safe_approach_navigation import safe_state


@pytest.mark.parametrize("emergency_stop", [False, True])
def test_pause_and_resume_preserve_refinement_without_server_requests(
    application, emergency_stop,
):
    ui = FakeUI()
    ui.handle_emergency_stop = Mock()
    controls = FinalizingInspectionControls(ui)
    controls.apply_setup_state(safe_state())
    assert controls.handle_start_probe_refinement()
    dialog = controls.refinement_dialog
    stage_page = dialog.workflow_stack.currentIndex()
    presentation = controls._refinement_presentation

    if emergency_stop:
        dialog.emergency_stop_button.click()
        ui.handle_emergency_stop.assert_called_once_with()

    assert dialog.close()
    assert not dialog.isVisible()
    assert controls.inspection_workspace_tabs.isEnabled()
    assert controls._refinement_presentation is presentation
    assert controls.start_probe_refinement_button.text() == "Resume Probe Point Setup"
    if emergency_stop:
        assert "paused after software stop" in controls.refinement_summary_status_label.text()

    assert controls.handle_start_probe_refinement()
    assert dialog.isVisible()
    assert dialog.workflow_stack.currentIndex() == stage_page
    assert controls._refinement_presentation is presentation
    assert not controls.inspection_workspace_tabs.isEnabled()
    assert dialog.close()
    assert "paused after software stop" not in controls.refinement_summary_status_label.text()
    assert ui.requests == []
