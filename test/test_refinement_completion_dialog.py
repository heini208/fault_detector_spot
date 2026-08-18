"""Regression guard for completed probe refinement dialog closure."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_finalization_completion_uses_guard_bypassing_dialog_close():
    source = (
        ROOT / "ui/inspection/finalizing_controls.py"
    ).read_text(encoding="utf-8")

    start = source.index("    def _finish_refinement_workflow_close(self):")
    end = source.index("\n    def handle_test_surface_distance", start)
    method = source[start:end]

    assert "super()._finish_refinement_workflow_close()" in method
    assert "self.refinement_dialog.close_after_completion()" in method
