"""Regression guards for the probe motion application boundary."""

from pathlib import Path

from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupOperation,
)


def _source(relative_path):
    return (
        Path(__file__).parents[1] / relative_path
    ).read_text(encoding="utf-8")


def test_probe_motion_uses_shared_setup_operation_directly():
    coordinator_source = _source(
        "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    )
    refinement_source = _source(
        "fault_detector_spot/application/coordinators/"
        "probe_refinement_controller.py"
    )

    assert "ProbeSetupMotionOperation" not in coordinator_source
    assert "-> SetupOperation:" in coordinator_source
    assert "require_operation(operation)" in coordinator_source
    assert "def require_operation(self, operation):" in refinement_source
    assert "tracked_operation" not in refinement_source
    assert SetupOperation.__name__ == "SetupOperation"


def test_probe_motion_is_not_duplicated_across_operation_wrappers():
    package_root = Path(__file__).parents[1]
    sources = tuple(
        (package_root / "fault_detector_spot").rglob("*.py")
    )

    for source in sources:
        text = source.read_text(encoding="utf-8")
        assert "ProbeSetupMotionOperation" not in text
