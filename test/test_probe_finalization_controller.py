"""Tests for extracted finalization state controller."""

from threading import RLock
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.commanding.request_identity import (
    new_request_id,
)
from fault_detector_spot.application.coordinators.probe_finalization_controller import (
    ProbeFinalizationController,
)


class FakeRefinementController:
    @staticmethod
    def require_refinement(draft):
        if draft.refinement is None:
            raise RuntimeError("Probe refinement is not active")
        return draft.refinement

    @staticmethod
    def require_physical_lane_idle():
        return None


def controller():
    return ProbeFinalizationController(
        object_repository=SimpleNamespace(),
        refinement_controller=FakeRefinementController(),
        state_lock=RLock(),
    )


def test_finalization_lock_is_context_scoped():
    finalization = controller()
    context = SimpleNamespace(context_id="context")
    request_id = new_request_id()

    finalization._active["context"] = request_id

    assert (
        finalization.active_request_id(context)
        == request_id
    )
    assert finalization.require(
        context,
        request_id,
    ) == request_id

    with pytest.raises(ValueError, match="UUID"):
        finalization.require(context, "other")


def test_discard_context_releases_finalization_lock():
    finalization = controller()
    context = SimpleNamespace(context_id="context")
    finalization._active["context"] = new_request_id()

    finalization.discard_context(context)

    assert finalization.active_request_id(context) == ""


@pytest.mark.parametrize("approved", [False, True])
def test_custom_finalization_requires_saved_pose_but_not_candidate_reach(approved):
    from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage

    finalization = controller()
    context = SimpleNamespace(context_id="custom")
    request_id = new_request_id()
    draft = SimpleNamespace(
        setup=SimpleNamespace(fully_custom=True),
        refinement=SimpleNamespace(stage_is_approved=lambda stage: stage is not RefinementStage.PROBE or approved),
    )
    if approved:
        finalization.begin(context, draft, request_id, True)
        assert finalization.active_request_id(context) == request_id
    else:
        with pytest.raises(ValueError, match="Save the custom final candidate"):
            finalization.begin(context, draft, request_id, True)
