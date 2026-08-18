from pathlib import Path


def _source():
    return Path(
        "fault_detector_spot/ui/inspection/finalizing_controls.py"
    ).read_text()


def test_normal_close_ends_server_owned_refinement():
    source = _source()

    assert "ProbeSetupIntent.OPERATION_END_REFINEMENT" in source
    assert "self._submit_probe_setup(intent)" in source


def test_emergency_recovery_keeps_retract_available():
    source = _source()

    assert "presentation.recovery_required" in source
    assert "self._distance_failure_requires_retraction" in source
    assert "retraction_required" in source
    assert "self.retract_without_saving_button.setEnabled(" in source
