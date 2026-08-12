"""Tests for extracted finalization state controller."""

from threading import RLock
from types import SimpleNamespace

import pytest

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

    finalization._active["context"] = "request"

    assert (
        finalization.active_request_id(context)
        == "request"
    )
    assert finalization.require(
        context,
        "request",
    ) == "request"

    with pytest.raises(RuntimeError, match="does not match"):
        finalization.require(context, "other")


def test_discard_context_releases_finalization_lock():
    finalization = controller()
    context = SimpleNamespace(context_id="context")
    finalization._active["context"] = "request"

    finalization.discard_context(context)

    assert finalization.active_request_id(context) == ""
