"""Sensor status reflects attachment evidence and requires explicit confirmation."""

import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QMessageBox, QPushButton, QWidget

from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.sensor.models import SensorAttachmentViewStatus


class StatusUI(QWidget):
    set_sensor_status = Fault_Detector_UI.set_sensor_status
    _sensor_display_name = Fault_Detector_UI._sensor_display_name
    _refresh_sensor_status = Fault_Detector_UI._refresh_sensor_status
    _request_sensor_state_confirmation = Fault_Detector_UI._request_sensor_state_confirmation
    _confirm_sensor_attachment = Fault_Detector_UI._confirm_sensor_attachment

    def __init__(self):
        super().__init__()
        self.sensor_indicator_label = QLabel()
        self.sensor_status_label = QLabel()
        self.sensor_confirm_button = QPushButton()
        self.status_label = QLabel()
        self.sensor_attachment_client = Mock()
        self._sensor_definitions = {"probe": SimpleNamespace(display_name="Hall probe")}
        self._sensor_attachment_state = None


@pytest.fixture
def ui():
    application = QApplication.instance() or QApplication([])
    widget = StatusUI()
    yield widget
    widget.close()
    # Keep the application alive for the lifetime of its widgets.
    del application


def attachment(status):
    return SimpleNamespace(
        status=status, pending_sensor_id="probe", active_sensor_id="probe",
        attachment_revision=7,
    )


@pytest.mark.parametrize("status,name,tooltip,can_confirm", [
    (None, "Unknown", "Sensor state unavailable", False),
    (SensorAttachmentViewStatus.PENDING, "Hall probe", "Confirmation pending", True),
    (SensorAttachmentViewStatus.NONE, "No sensor", "Confirmation pending", True),
    (SensorAttachmentViewStatus.ACTIVE, "Hall probe", "Confirmed", False),
])
def test_status_and_confirmation_follow_attachment_evidence(ui, status, name, tooltip, can_confirm):
    ui._sensor_attachment_state = attachment(status) if status is not None else None

    ui._refresh_sensor_status()

    assert ui.sensor_status_label.text() == name
    assert ui.sensor_indicator_label.toolTip() == tooltip
    assert ui.sensor_confirm_button.isEnabled() is can_confirm
    ui.sensor_attachment_client.confirm.assert_not_called()


@pytest.mark.parametrize("status,sensor_id", [
    (SensorAttachmentViewStatus.PENDING, "probe"),
    (SensorAttachmentViewStatus.NONE, ""),
])
@pytest.mark.parametrize("answer", [QMessageBox.No, QMessageBox.Yes])
def test_confirmation_requires_explicit_yes_and_preserves_attachment_revision(
    ui, monkeypatch, status, sensor_id, answer,
):
    ui._sensor_attachment_state = attachment(status)
    question = Mock(return_value=answer)
    monkeypatch.setattr(QMessageBox, "question", question)

    ui._request_sensor_state_confirmation()

    assert question.call_args.args[-1] == QMessageBox.No
    if answer == QMessageBox.Yes:
        ui.sensor_attachment_client.confirm.assert_called_once_with(sensor_id, 7)
    else:
        ui.sensor_attachment_client.confirm.assert_not_called()
