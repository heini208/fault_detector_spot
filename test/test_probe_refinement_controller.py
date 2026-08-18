"""Tests for extracted probe refinement controller."""

from types import SimpleNamespace

import pytest

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
