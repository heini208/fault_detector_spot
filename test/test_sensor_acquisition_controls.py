"""Focused UI check for authoritative sensor recording feedback."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QWidget

from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.ros.sensor_acquisition_client import (
    SensorAcquisitionClient,
)
from fault_detector_spot.ui.sensor.models import (
    SensorAcquisitionView,
    SensorAcquisitionViewStatus,
)


class HeaderHarness(QWidget):
    set_sensor_acquisition_state = (
        Fault_Detector_UI.set_sensor_acquisition_state
    )
    _toggle_sensor_recording = Fault_Detector_UI._toggle_sensor_recording
    open_measurement_directory = (
        Fault_Detector_UI.open_measurement_directory
    )

    def __init__(self):
        super().__init__()
        self.sensor_recording_indicator_label = QLabel()
        self.sensor_recording_button = QPushButton("Record")
        self.sensor_recording_button.clicked.connect(
            self._toggle_sensor_recording
        )
        self._sensor_acquisition_state = None
        self.requests = []
        self.status_label = QLabel()

    def _request_sensor_recording(self, start):
        self.requests.append(start)

    def _process_application_error(self, detail):
        self.status_label.setText(detail)


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_button_follows_authoritative_recording_state(application):
    ui = HeaderHarness()

    ui.set_sensor_acquisition_state(SensorAcquisitionView(
        SensorAcquisitionViewStatus.IDLE,
        "",
        "Idle",
    ))
    ui.sensor_recording_button.click()

    assert ui.requests == [True]

    ui.set_sensor_acquisition_state(SensorAcquisitionView(
        SensorAcquisitionViewStatus.RECORDING,
        "bmm150_probe",
        "Measurement recording is active",
    ))
    ui.sensor_recording_button.click()

    assert ui.sensor_recording_button.text() == "Stop"
    assert ui.requests == [True, False]
    ui.close()


def test_header_start_uses_empty_manual_context():
    intent = SensorAcquisitionClient.make_intent(True)

    assert intent.object_id == ""
    assert intent.routine_id == ""
    assert intent.probe_point_id == ""


def test_folder_button_opens_configured_measurement_root(
    application,
    monkeypatch,
    tmp_path,
):
    ui = HeaderHarness()
    ui.measurement_root = tmp_path / "measurements"
    opened_paths = []
    monkeypatch.setattr(
        QDesktopServices,
        "openUrl",
        lambda url: opened_paths.append(url.toLocalFile()) or True,
    )

    assert ui.open_measurement_directory() is True
    assert ui.measurement_root.is_dir()
    assert opened_paths == [str(ui.measurement_root)]
    ui.close()
