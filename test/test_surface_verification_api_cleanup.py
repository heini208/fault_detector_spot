"""Ensure the obsolete setup-specific surface-verification API is gone."""

import inspect

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)


def test_application_node_does_not_create_legacy_surface_verification_api():
    source = inspect.getsource(ApplicationApiNode)

    assert "ProbeSurfaceVerificationApi" not in source
    assert "probe_surface_verification_api" not in source


def test_close_surface_remains_on_generic_application_operation_boundary():
    source = inspect.getsource(ApplicationApiNode)

    assert "ExecuteOperation" in source
    assert "application_controller" in source
