from pathlib import Path


def _source():
    return Path(
        "fault_detector_spot/ui/inspection/finalizing_controls.py"
    ).read_text()


def test_emergency_recovery_keeps_retract_available():
    source = _source()

    assert "presentation.recovery_required" in source
    assert "self._distance_failure_requires_retraction" in source
    assert "retraction_required" in source
    assert "self.retract_without_saving_button.setEnabled(" in source
