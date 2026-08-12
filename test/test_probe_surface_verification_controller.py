"""Tests for extracted surface-verification state controller."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.application.coordinators.probe_surface_verification_controller import (
    ProbeSurfaceVerificationController,
)


class FakeRefinementController:
    @staticmethod
    def require_refinement(draft):
        if draft.refinement is None:
            raise RuntimeError("Probe refinement is not active")
        return draft.refinement


def test_surface_controller_rejects_missing_session():
    controller = ProbeSurfaceVerificationController(
        FakeRefinementController(),
        SimpleNamespace(
            __enter__=lambda self: self,
            __exit__=lambda self, *args: False,
        ),
    )
    draft = SimpleNamespace(
        refinement=object(),
        surface_verification=None,
    )

    with pytest.raises(RuntimeError, match="not active"):
        controller.cancel(draft, "request")


def test_active_request_id_only_reports_active_verification():
    active = SimpleNamespace(
        active=True,
        request_id="request",
    )
    inactive = SimpleNamespace(
        active=False,
        request_id="old",
    )

    assert (
        ProbeSurfaceVerificationController.active_request_id(
            SimpleNamespace(surface_verification=active)
        )
        == "request"
    )
    assert (
        ProbeSurfaceVerificationController.active_request_id(
            SimpleNamespace(surface_verification=inactive)
        )
        == ""
    )
