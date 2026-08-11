"""Tests for the thin UI surface-verification boundary."""

import inspect

from fault_detector_spot.ui.inspection.controls import InspectionControls
from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient


def test_surface_verification_button_delegates_to_remote_action():
    source = inspect.getsource(
        InspectionControls.handle_test_surface_distance
    )

    assert "execute_probe_surface_verification" in source
    assert "measure_probe_surface_distance" not in source
    assert "aggregate_surface_distance_samples" not in source


def test_probe_setup_client_exposes_surface_action():
    assert hasattr(ProbeSetupClient, "execute_surface_verification")
