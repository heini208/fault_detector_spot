"""Regression guards for presentation-only inspection command state."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
CONTROLS = ROOT / "fault_detector_spot/ui/inspection/controls.py"


def test_inspection_ui_does_not_own_application_command_state():
    source = CONTROLS.read_text(encoding="utf-8")

    forbidden = (
        "ApplicationCommandState",
        "PROBE_MOTION_SETTLE_SEC",
        "_command_state",
        "_buffered_command_count",
        "_last_command_completion_monotonic",
        "_require_command_path_idle",
        "_probe_motion_pending",
        "time.monotonic",
    )
    for value in forbidden:
        assert value not in source
