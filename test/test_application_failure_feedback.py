"""Focused UI check for visible local command failures."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QMessageBox, QWidget

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
