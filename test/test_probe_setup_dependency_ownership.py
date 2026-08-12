"""Regression guards for probe setup dependency ownership."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
COORDINATOR = (
    ROOT
    / "fault_detector_spot/application/coordinators/"
    "probe_setup_coordinator.py"
)


def test_probe_setup_facade_does_not_retain_collaborator_build_inputs():
    source = COORDINATOR.read_text(encoding="utf-8")

    assert "self.geometry =" not in source
    assert "self.motion_command_factory =" not in source
    assert "geometry = geometry or ProbeSetupGeometry(" in source
    assert "motion_command_factory = (" in source
    assert "motion_command_factory=motion_command_factory" in source
