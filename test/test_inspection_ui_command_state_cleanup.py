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


def test_application_state_hook_does_not_derive_inspection_workflow_state():
    source = CONTROLS.read_text(encoding="utf-8")

    marker = "def handle_application_state(self, _status):"
    assert marker in source
    method = source.split(marker, 1)[1].split("def set_sensor_definitions", 1)[0]
    assert "return None" in method
    assert "status." not in method
