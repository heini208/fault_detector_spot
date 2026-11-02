"""Focused UI check for visible local command failures."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QMessageBox, QWidget

from fault_detector_msgs.msg import ApplicationCommandState
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI


class FailureHarness(QWidget):
    _show_local_command_failure = (
        Fault_Detector_UI._show_local_command_failure
    )

    def __init__(self):
        super().__init__()
        self.application_client = SimpleNamespace(client_id="operator_ui")


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_local_command_failure_opens_error_dialog(application, monkeypatch):
    dialogs = []
    monkeypatch.setattr(
        QMessageBox,
        "warning",
        lambda _parent, title, detail: dialogs.append((title, detail)),
    )
    state = ApplicationCommandState()
    state.client_id = "operator_ui"
    state.state = ApplicationCommandState.STATE_FAILED
    state.detail = "Sensor attachment confirmation is pending"

    FailureHarness()._show_local_command_failure(
        state,
        "MOVE_ARM_TO_TAG",
    )

    assert dialogs == [(
        "Move Arm To Tag failed",
        "Sensor attachment confirmation is pending",
    )]

    state.client_id = "record_manager"
    FailureHarness()._show_local_command_failure(state, "INTERNAL")

    assert len(dialogs) == 1


def test_emergency_service_acknowledgement_does_not_claim_stopping_finished(application):
    harness = SimpleNamespace(status_label=QLabel())

    Fault_Detector_UI._process_emergency_stop_result(harness, True, "Request accepted")

    assert "awaiting stop confirmation" in harness.status_label.text()
    Fault_Detector_UI._process_emergency_stop_result(harness, False, "Transport unavailable")
    assert "unconfirmed" in harness.status_label.text()


def test_stop_button_distinguishes_software_stop_from_robot_safety_estop(application):
    class StopHarness(QWidget):
        _make_estop_button = Fault_Detector_UI._make_estop_button

        def __init__(self):
            super().__init__()
            self.requests = 0

        def handle_emergency_stop(self):
            self.requests += 1

    harness = StopHarness()
    button = harness._make_estop_button()

    assert button.text() == "SOFTWARE STOP"
    assert "robot safety E-stop" in button.toolTip()
    button.click()
    button.click()
    assert harness.requests == 2
    assert harness._make_estop_button() is button
