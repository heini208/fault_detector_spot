"""Tests for extracted probe refinement controller."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import CommandControllerState

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
)


def test_motion_kind_mapping_is_owned_by_refinement_controller():
    assert ProbeRefinementController.motion_stage(
        ProbeMotionKind.MOVE_SAFE_APPROACH
    ) is RefinementStage.SAFE_APPROACH
    assert ProbeRefinementController.motion_stage(
        ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH
    ) is RefinementStage.ALIGNMENT
    assert ProbeRefinementController.motion_stage(
        ProbeMotionKind.ADJUST_SAFE_APPROACH
    ) is RefinementStage.SAFE_APPROACH
    assert ProbeRefinementController.motion_stage(
        ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH
    ) is RefinementStage.ALIGNMENT


def test_refinement_requirement_rejects_missing_session():
    with pytest.raises(RuntimeError, match="not active"):
        ProbeRefinementController.require_refinement(
            SimpleNamespace(refinement=None)
        )


@pytest.mark.parametrize("command", [CommandID.EMERGENCY_CANCEL, CommandID.RECOVER_ARM])
def test_stop_and_recovery_invalidate_saved_refinement_pose_evidence(command):
    draft = SimpleNamespace(refinement=Mock())
    draft.refinement.invalidate_external_arm_motion.return_value = True
    status = SimpleNamespace(
        state=CommandControllerState.DISPATCHED,
        request=SimpleNamespace(command=SimpleNamespace(command_id=command)),
    )

    assert ProbeRefinementController.invalidate_external_motion(None, draft, status)

    draft.refinement.invalidate_external_arm_motion.assert_called_once_with()


def test_motion_free_reset_does_not_invent_external_movement():
    draft = SimpleNamespace(refinement=Mock())
    status = SimpleNamespace(
        state=CommandControllerState.DISPATCHED,
        request=SimpleNamespace(command=SimpleNamespace(command_id=CommandID.RESET_SOFTWARE_STOP)),
    )

    assert not ProbeRefinementController.invalidate_external_motion(None, draft, status)

    draft.refinement.invalidate_external_arm_motion.assert_not_called()
