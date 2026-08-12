"""Regression guards for reference-capture readiness ownership."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
PACKAGE = ROOT / "fault_detector_spot"
SETUP = PACKAGE / "inspection/setup"


def test_retryable_capture_error_has_one_owner():
    source = (
        SETUP / "reference_view_validation.py"
    ).read_text(encoding="utf-8")

    assert "class ReferenceViewCaptureNotReady(RuntimeError)" in source
    assert "class ReferenceViewInputNotReady" not in source


def test_removed_capture_error_module_has_no_remaining_imports():
    assert not (SETUP / "reference_view_capture.py").exists()

    for path in PACKAGE.rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "inspection.setup.reference_view_capture" not in source
        assert "ReferenceViewInputNotReady" not in source
