"""Regression tests for persistent reference-selector enablement."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


class FakeProbeSetupClient:
    def __init__(self):
        self.context_id = ""
        self.preview_requests = []


class FakeUI:
    def __init__(self):
        self.node = None
        self.status_label = QLabel()
        self.probe_setup_client = FakeProbeSetupClient()

    def show_setup_unavailable(self, workflow):
        return False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_reference_selector_stays_enabled_after_state_refresh(application):
    controls = FinalizingInspectionControls(FakeUI())
    dialog = controls.refinement_dialog

    dialog.open_reference_selection(has_existing=True)
    assert not dialog.reference_view_group.isEnabled()

    dialog.enable_reference_selection()
    assert dialog.reference_view_group.isEnabled()

    dialog.update_reference_availability(True)

    assert dialog.reference_view_group.isEnabled()


def test_capture_state_temporarily_disables_reference_selector(application):
    controls = FinalizingInspectionControls(FakeUI())
    dialog = controls.refinement_dialog

    dialog.open_reference_selection(has_existing=True)
    dialog.enable_reference_selection()
    dialog.set_reference_capture_running()

    assert not dialog.reference_view_group.isEnabled()

    dialog.reference_views_ready()

    assert dialog.reference_view_group.isEnabled()
