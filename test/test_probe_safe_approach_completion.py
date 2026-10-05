"""Safe staging trusts command success; alignment retains pose verification."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind, ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from fault_detector_spot.shared.geometry.models import PoseData
from test_probe_setup_coordinator import (
    coordinator, create_selected_routine, ImagePoint,
)


def test_alignment_still_rejects_the_reported_endpoint_error():
    achieved = PoseData.identity()
    achieved.position.x = 0.0171
    pending = SimpleNamespace(
        verify_achieved_pose=True, target_pose_object=PoseData.identity(),
    )
    motion = ProbeMotionRequest(kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH)
    with pytest.raises(RuntimeError, match="missed the target"):
        ProbeRefinementController.verify_achieved_motion(pending, motion, achieved)


def test_cancelled_safe_command_does_not_mark_pose_reached(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    state = probe.select_reference_pixel(
        state.context, "slot1_hand", ImagePoint(u=20, v=30),
        "surface_fit", 0.10, 0.20,
    )
    state = probe.begin_refinement(state.context)
    operation = probe.prepare_motion(
        state.context, ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
    )
    probe.submit_motion(operation)
    commands.cancel(operation.request_id)
    state = probe.snapshot(probe.context(state.context.context_id, "probe-ui"))
    assert state.refinement.motion_states[RefinementStage.SAFE_APPROACH].value == "Failed"
    assert state.refinement.pending_motion is None
