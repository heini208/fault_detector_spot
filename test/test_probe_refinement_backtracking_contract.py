"""Regression guards for physical-stage backtracking."""

import inspect
from types import SimpleNamespace

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)


def refinement_state():
    return SimpleNamespace(
        motion_states={
            RefinementStage.SAFE_APPROACH: RefinementMotionState.REACHED,
            RefinementStage.ALIGNMENT: RefinementMotionState.REACHED,
            RefinementStage.PROBE: RefinementMotionState.REACHED,
        },
    )


def test_safe_approach_motion_invalidates_downstream_physical_reach():
    refinement = refinement_state()

    ProbeRefinementController._invalidate_downstream_motion_state(
        refinement,
        RefinementStage.SAFE_APPROACH,
    )

    assert refinement.motion_states[RefinementStage.SAFE_APPROACH] is (
        RefinementMotionState.REACHED
    )
    assert refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.NOT_TESTED
    )
    assert refinement.motion_states[RefinementStage.PROBE] is (
        RefinementMotionState.NOT_TESTED
    )


def test_alignment_motion_invalidates_only_probe_physical_reach():
    refinement = refinement_state()

    ProbeRefinementController._invalidate_downstream_motion_state(
        refinement,
        RefinementStage.ALIGNMENT,
    )

    assert refinement.motion_states[RefinementStage.SAFE_APPROACH] is (
        RefinementMotionState.REACHED
    )
    assert refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )
    assert refinement.motion_states[RefinementStage.PROBE] is (
        RefinementMotionState.NOT_TESTED
    )


def test_motion_preparation_invalidates_downstream_before_dispatch():
    source = inspect.getsource(ProbeRefinementController.prepare_motion)

    assert "_invalidate_downstream_motion_state" in source
    assert source.index("_invalidate_downstream_motion_state") < source.index(
        "prepare_command"
    )
