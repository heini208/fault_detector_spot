"""Boundary tests for server-owned probe finalization UI routing."""

import inspect

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient
from fault_detector_spot.ui.ros.probe_setup_state_adapter import _refinement


def test_probe_setup_client_exposes_finalization_action():
    source = inspect.getsource(ProbeSetupClient.finalize_refinement)

    assert "FinalizeProbeRefinement.Goal" in source
    assert "MODE_SAVE_AND_RETRACT" in source
    assert "MODE_RETRACT_WITHOUT_SAVING" in source


def test_approve_and_retract_delegates_to_application_boundary():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_approve_and_retract
    )

    assert "execute_probe_refinement_finalization" in source
    assert "save_requested=True" in source
    assert "_send_refinement_motion" not in source


def test_retract_without_saving_delegates_to_application_boundary():
    source = inspect.getsource(
        FinalizingInspectionControls.handle_retract_without_saving
    )

    assert "execute_probe_refinement_finalization" in source
    assert "save_requested=False" in source
    assert "_send_refinement_motion" not in source


def test_transport_adapter_maps_authoritative_recovery_state():
    source = inspect.getsource(_refinement)

    assert "state.refinement_recovery_required" in source
    assert "state.refinement_recovery_message" in source


def test_finalization_controls_do_not_replace_base_controls_file():
    source = inspect.getsource(FinalizingInspectionControls)

    assert "class FinalizingInspectionControls(InspectionControls)" in source
