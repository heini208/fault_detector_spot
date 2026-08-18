"""Contracts for removal of obsolete probe-setup verification state."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
SPOT = ROOT / "fault_detector_spot"


def source(relative):
    return (SPOT / relative).read_text(encoding="utf-8")


def test_probe_setup_client_does_not_read_removed_fields():
    client = source("ui/ros/probe_setup_client.py")

    for name in (
        "surface_verification_request_id",
        "surface_verification_state",
        "has_surface_distance_measurement",
        "measured_surface_distance_m",
        "surface_distance_error_m",
        "surface_verification_iteration",
        "surface_recovery_required",
    ):
        assert name not in client


def test_probe_setup_view_no_longer_reads_verification_message_state():
    adapter = source("ui/ros/probe_setup_state_adapter.py")

    assert "SURFACE_VERIFICATION_CONVERGED" not in adapter
    assert "state.surface_verification_state" not in adapter


def test_base_inspection_controls_no_longer_render_removed_transport_state():
    controls = source("ui/inspection/controls.py")

    assert "_render_surface_verification_state" not in controls
    assert "surface_verification_state" not in controls
    assert "surface_verification_request_id" not in controls


def test_finalizing_controls_need_no_compatibility_renderer():
    controls = source("ui/inspection/finalizing_controls.py")

    assert "_render_surface_verification_state" not in controls
