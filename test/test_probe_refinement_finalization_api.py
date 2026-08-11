"""Tests for final probe refinement API ownership and terminal state."""

import inspect

import pytest
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState

from fault_detector_spot.inspection.setup.probe_refinement_finalization import (
    FinalizationPhase,
)
from fault_detector_spot.inspection.setup.probe_refinement_finalization_api import (
    ProbeRefinementFinalizationApi,
)
from fault_detector_spot.inspection.setup.probe_setup_api import ProbeSetupApi


def test_sync_probe_setup_api_does_not_expose_final_probe_persistence():
    source = inspect.getsource(ProbeSetupApi._transaction_handlers)

    assert "OPERATION_APPROVE_PROBE_POSE" not in source
    assert "OPERATION_SAVE_PROBE_POINT" not in source


def test_probe_setup_operation_validation_uses_actual_handlers():
    api = ProbeSetupApi.__new__(ProbeSetupApi)
    api._handlers = {
        ProbeSetupIntent.OPERATION_REFRESH: object(),
    }

    api._validate_operation(ProbeSetupIntent.OPERATION_OPEN)
    api._validate_operation(ProbeSetupIntent.OPERATION_REFRESH)

    with pytest.raises(ValueError):
        api._validate_operation(
            ProbeSetupIntent.OPERATION_APPROVE_PROBE_POSE
        )
    with pytest.raises(ValueError):
        api._validate_operation(
            ProbeSetupIntent.OPERATION_SAVE_PROBE_POINT
        )


def test_finalization_terminal_state_matches_action_outcome():
    terminal = ProbeRefinementFinalizationApi._terminal_state_code

    assert (
        terminal(FinalizationPhase.COMPLETE, False)
        == ProbeSetupState.STATE_SUCCEEDED
    )
    assert (
        terminal(FinalizationPhase.RECOVERY_REQUIRED, False)
        == ProbeSetupState.STATE_FAILED
    )
    assert (
        terminal(FinalizationPhase.RECOVERY_REQUIRED, True)
        == ProbeSetupState.STATE_CANCELLED
    )


def test_recovery_feedback_is_not_reported_as_running():
    source = inspect.getsource(
        ProbeRefinementFinalizationApi._publish_feedback
    )

    assert "FinalizationPhase.RECOVERY_REQUIRED" in source
    assert "_terminal_state_code" in source


def test_cancelled_fallback_state_is_cancelled():
    source = inspect.getsource(ProbeRefinementFinalizationApi._abort)

    assert "ProbeSetupState.STATE_CANCELLED" in source
    assert "goal_handle.canceled()" in source
