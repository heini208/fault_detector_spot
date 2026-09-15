"""UI wiring regression for direct move-close-to-surface execution."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_manipulation_ui_submits_distance_and_tolerance():
    source = (ROOT / "ui/manipulation/controls.py").read_text(encoding="utf-8")
    start = source.index("    def handle_move_close_to_surface(self):")
    end = source.index("\n    def _reset_all_zero", start)
    handler = source[start:end]

    assert "INTENT_MOVE_CLOSE_TO_SURFACE" in handler
    assert "surface_distance_input.value()" in handler
    assert "surface_tolerance_input.value()" in handler
    assert "target_surface_distance_m" in handler
    assert "surface_tolerance_m" in handler
    assert "execute_operation(intent)" in handler
    assert "show_setup_unavailable" not in handler
