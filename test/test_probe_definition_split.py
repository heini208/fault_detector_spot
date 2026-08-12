"""Regression guard for the probe coordinator split."""

from pathlib import Path


def test_probe_coordinator_delegates_definition_authoring():
    package_root = Path(__file__).parents[1]
    coordinator = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "ProbeDefinitionService" in coordinator
    assert "InspectionObject" not in coordinator
    assert "InspectionRoutine" not in coordinator
    assert "ReferenceTag" not in coordinator
    assert "_selected_definition_lists" not in coordinator
