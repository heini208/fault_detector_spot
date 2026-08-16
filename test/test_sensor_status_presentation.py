"""Tests for the global sensor status presentation."""

from pathlib import Path


UI_PATH = (
    Path(__file__).parents[1]
    / "fault_detector_spot"
    / "ui"
    / "fault_detector_ui.py"
)


def test_main_ui_defines_global_sensor_status_label():
    source = UI_PATH.read_text(encoding="utf-8")

    assert "self.sensor_status_label = QLabel()" in source
    assert "self.set_sensor_status(" in source


def test_sensor_status_supports_required_visual_states():
    source = UI_PATH.read_text(encoding="utf-8")

    assert '"confirmed": ("#2E7D32", "Confirmed")' in source
    assert '"pending": ("#EF6C00", "Confirmation pending")' in source
    assert '"none": ("#757575", "No sensor")' in source


def test_sensor_status_is_passed_to_global_status_panel():
    source = UI_PATH.read_text(encoding="utf-8")

    assert "self.sensor_status_label," in source
