"""Regression coverage for aborting pending probe-point creation."""

import inspect

import pytest

from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from fault_detector_spot.ui.inspection.probe_refinement_dialog import (
    ProbeRefinementDialog,
)

from test_probe_setup_coordinator import (
    coordinator,
    create_selected_routine,
)


def _begin_refinement(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    return probe, command_controller, probe.begin_refinement(
        state.context
    )


def test_abort_clears_recovery_without_resuming(tmp_path):
    probe, _, state = _begin_refinement(tmp_path)
    draft = probe._drafts[state.context.context_id]
    draft.refinement.require_recovery("test recovery")
    attachment_controller = (
        probe.refinement_controller.sensor_attachment_controller
    )

    aborted = probe.end_refinement(state.context)

    assert aborted.refinement is None
    assert attachment_controller.active_reservations == 0
    assert not aborted.dirty


def test_abort_cancels_active_setup_motion(tmp_path):
    probe, _, state = _begin_refinement(tmp_path)
    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(
            kind=ProbeMotionKind.MOVE_SAFE_APPROACH,
        ),
    )
    probe.submit_motion(operation)

    aborted = probe.end_refinement(state.context)

    assert aborted.refinement is None
    assert not probe.refinement_controller.request_ids_for(
        aborted.context
    )


def test_abort_revokes_active_finalization_and_ignores_late_failure(tmp_path):
    probe, _, state = _begin_refinement(tmp_path)
    request_id = "9b361339-3d85-4a2e-a5c3-88cf9a6d4872"
    probe.finalization_controller._active[state.context.context_id] = request_id

    aborted = probe.end_refinement(state.context)

    assert aborted.refinement is None
    assert not probe.finalization_controller.active_request_id(aborted.context)
    with pytest.raises(RuntimeError, match="does not match"):
        probe.prepare_motion(
            aborted.context,
            ProbeMotionRequest(ProbeMotionKind.MOVE_SAFE_APPROACH, retract_path=True),
            finalization_request_id=request_id,
        )
    late = probe.fail_finalization(aborted.context, request_id, "Cancelled")
    assert late.refinement is None
    attachment = probe.refinement_controller.sensor_attachment_controller
    assert attachment.active_reservations == 0


def test_abort_control_is_available_in_dialog_and_main_panel():
    dialog_source = inspect.getsource(ProbeRefinementDialog.__init__)
    controls_source = inspect.getsource(
        FinalizingInspectionControls._configure_probe_point_entry_ui
    )
    handler_source = inspect.getsource(
        FinalizingInspectionControls.handle_abort_probe_refinement
    )

    assert "Abort Setup" in dialog_source
    assert "Abort Probe Point Setup" in controls_source
    assert "OPERATION_END_REFINEMENT" in handler_source
