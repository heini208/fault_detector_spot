"""Regression guard for probe geometry extraction."""

from pathlib import Path


def test_probe_coordinator_delegates_geometry_draft_editing():
    package_root = Path(__file__).parents[1]
    source = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "ProbeGeometryEditor" in source
    assert "_resolve_geometry" not in source
    assert "_retained_distance_approvals" not in source
    assert "def select_reference_pixel(" in source
    assert "def clear_reference_pixel(" in source
    assert "def update_geometry(" in source
