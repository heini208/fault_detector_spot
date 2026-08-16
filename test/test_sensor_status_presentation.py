"""Tests for the global sensor status presentation."""

from pathlib import Path


UI_PATH = (
    Path(__file__).parents[1]
    / "fault_detector_spot"
    / "ui"
    / "fault_detector_ui.py"
)


def test_main_ui_defines_global_sensor_status_and_confirmation_control():
    source = UI_PATH.read_text(encoding="utf-8")

    assert 'self.sensor_indicator_label = QLabel("●")' in source
    assert 'self.sensor_status_label = QLabel("Unknown")' in source
    assert 'self.sensor_confirm_button = QPushButton("✓")' in source
    assert "self.sensor_confirm_button.setFixedSize(24, 24)" in source
    assert "self._request_sensor_state_confirmation" in source


def test_sensor_state_text_is_exposed_as_ball_tooltip():
    source = UI_PATH.read_text(encoding="utf-8")

    assert '"confirmed": ("#2E7D32", "Confirmed")' in source
    assert '"pending": ("#EF6C00", "Confirmation pending")' in source
    assert (
        '"unknown": ("#757575", "Sensor state unavailable")'
        in source
    )
    assert "self.sensor_indicator_label.setToolTip(tooltip)" in source
    assert "self.sensor_status_label.setText(" in source


def test_visible_sensor_text_contains_only_sensor_name():
    source = UI_PATH.read_text(encoding="utf-8")

    assert 'self.set_sensor_status("pending", "No sensor")' in source
    assert 'self.set_sensor_status("confirmed", sensor_name)' in source
    assert 'self.sensor_status_label.setText(sensor_name.strip() or "Unknown")' in (
        source
    )
    assert "Confirmation pending —" not in source


def test_confirmation_button_uses_explicit_popup():
    source = UI_PATH.read_text(encoding="utf-8")

    assert "QMessageBox.question(" in source
    assert '"Confirm sensor state"' in source
    assert "QMessageBox.Yes | QMessageBox.No" in source
    assert "QMessageBox.No," in source


def test_sensor_status_widgets_are_global():
    source = UI_PATH.read_text(encoding="utf-8")

    assert "self.sensor_indicator_label," in source
    assert "self.sensor_status_label," in source
    assert "self.sensor_confirm_button," in source
