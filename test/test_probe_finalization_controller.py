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
