"""Regression guard for probe refinement extraction."""

from pathlib import Path


def test_probe_coordinator_delegates_refinement_motion_ownership():
    package_root = Path(__file__).parents[1]
    source = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "ProbeRefinementController" in source
    assert "self._operations" not in source
    assert "def _motion_stage(" not in source
    assert "def _verify_achieved_motion(" not in source
    assert "def _absolute_motion_command(" not in source
    assert "def _relative_motion_command(" not in source
    assert "def _current_probe_pose(" not in source
