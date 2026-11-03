"""Boundary tests for server-owned probe finalization UI routing."""

import inspect

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient


def test_probe_setup_client_exposes_finalization_action():
    source = inspect.getsource(ProbeSetupClient.finalize_refinement)

    assert "FinalizeProbeRefinement.Goal" in source
    assert "MODE_SAVE_AND_RETRACT" in source
    assert "MODE_RETRACT_WITHOUT_SAVING" in source


def test_retract_without_saving_delegates_to_application_boundary():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_retract_without_saving
    )

    assert "execute_probe_refinement_finalization" in source
    assert "save_requested=False" in source
    assert "_send_refinement_motion" not in source
