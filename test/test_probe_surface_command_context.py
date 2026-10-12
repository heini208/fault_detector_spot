"""Surface-test commands retain only their own setup's reached checkpoint."""

from copy import deepcopy

import pytest

from fault_detector_msgs.msg import OperationalIntent
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
    CommandControllerStatus,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from test_pre_approach_path import begin_setup, snapshot
from test_probe_setup_coordinator import coordinator


@pytest.mark.parametrize("scope", ["current", "unscoped", "other"])
def test_surface_command_context_controls_alignment_invalidation(tmp_path, scope):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands, approved=True)
    approved_setup = deepcopy(state.setup)
    assert state.refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )
    contexts = {
        "current": state.context.context_id,
        "unscoped": "",
        "other": "different-workflow",
    }
    app = ApplicationController(commands)
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.target_surface_distance_m = 0.1
    intent.surface_tolerance_m = 0.005
    operation = app.prepare_operation(intent, "probe-ui", contexts[scope])
    app.submit(operation)

    expected = (
        RefinementMotionState.REACHED
        if scope == "current"
        else RefinementMotionState.NOT_TESTED
    )
    for transition in (
        CommandControllerState.DISPATCHED,
        CommandControllerState.RUNNING,
        CommandControllerState.SUCCEEDED,
    ):
        status = CommandControllerStatus(operation.request, transition)
        for listener in tuple(commands.listeners):
            listener(status)
        current = snapshot(probe, state)
        assert current.refinement.motion_states[RefinementStage.ALIGNMENT] is expected
        assert current.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
        assert current.setup == approved_setup

