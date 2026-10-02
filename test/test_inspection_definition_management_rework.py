"""Tests for typed inspection-definition authoring intent."""

import os
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
    ProbeSetupState,
)
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class FakeProbeSetupClient:
    def __init__(self):
        self.context_id = "probe-context"
        self.preview_requests = []

    def request_preview(self, view_id):
        self.preview_requests.append(view_id)


class FakeUI:
    def __init__(self):
        self.node = None
        self.status_label = QLabel()
        self.probe_setup_client = FakeProbeSetupClient()
        self.requests = []
        self.operations = []

    def execute_probe_setup(self, intent):
        self.requests.append(intent)
        return f"request-{len(self.requests)}"

    def execute_operation(self, intent, context_id=""):
        self.operations.append((intent, context_id))
        return f"operation-{len(self.operations)}"

    def show_setup_unavailable(self, workflow):
        return False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    result = FinalizingInspectionControls(FakeUI())
    result.ui.requests.clear()
    result.ui.operations.clear()
    result.show_warning = lambda title, message: None
    result.ask_question = lambda title, message: True
    return result


def state(object_id="", routine_id="", has_base_position=False):
    message = ProbeSetupState()
    message.client_id = "probe-ui"
    message.context_id = "probe-context"
    message.state = ProbeSetupState.STATE_READY
    message.object_ids = ["motor"]
    message.selected_object_id = object_id
    message.selected_reference_tag_id = 7 if routine_id else -1
    message.selected_reference_tag_family = (
        "36h11" if routine_id else ""
    )
    if object_id:
        message.routine_ids = ["magnetic_scan"]
    message.selected_routine_id = routine_id
    message.has_base_position = bool(has_base_position)
    return message


def test_snapshot_populates_definition_selectors(controls):
    controls.apply_setup_state(state("motor", "magnetic_scan"))

    assert controls.saved_object_dropdown.currentData() == "motor"
    assert controls.saved_routine_dropdown.currentData() == "magnetic_scan"
    assert controls.routine_parent_object_dropdown.currentData() == "motor"
    assert controls._probe_setup_state.selected_reference_tag_id == 7
    assert not hasattr(controls, "_selected_reference_tag_id")
    assert not hasattr(controls, "_selected_sensor_id")


def test_object_creation_submits_typed_intent(controls):
    controls.object_id_field.setText("motor")
    controls.object_display_name_field.setText("Motor")
    assert controls.handle_create_object() is True

    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_CREATE_OBJECT
    assert intent.object_id == "motor"
    assert intent.object_display_name == "Motor"
    assert not hasattr(controls, "object_repository")


def test_routine_creation_submits_typed_intent(controls):
    controls.apply_setup_state(state("motor"))
    controls.routine_id_field.setText("magnetic_scan")
    controls.routine_display_name_field.setText("Magnetic scan")
    controls.reference_tag_id_field.setText("7")
    controls.reference_tag_family_field.setText("36h11")

    assert controls.handle_create_routine() is True

    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_CREATE_ROUTINE
    assert intent.object_id == "motor"
    assert intent.routine_id == "magnetic_scan"
    assert intent.reference_tag_id == 7
    assert intent.reference_tag_family == "36h11"
    assert not hasattr(intent, "sensor_id")


def test_base_position_status_and_save_submit_typed_intent(controls):
    controls.apply_setup_state(
        state("motor", "magnetic_scan", has_base_position=True)
    )

    assert controls.set_base_position_button.isEnabled()
    assert controls.base_position_status_label.text() == (
        "Base position: configured"
    )
    assert controls.move_to_base_position_button.isEnabled()

    assert controls.handle_save_base_position() is True
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_SAVE_BASE_POSITION
    )


def test_move_to_base_position_submits_selected_routine_intent(controls):
    controls.apply_setup_state(
        state("motor", "magnetic_scan", has_base_position=True)
    )

    assert controls.handle_move_to_base_position() is True
    intent, context_id = controls.ui.operations[-1]
    assert intent.intent == (
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION
    )
    assert intent.object_id == "motor"
    assert intent.routine_id == "magnetic_scan"
    assert context_id.startswith("routine-base-")
    assert not controls.move_to_base_position_button.isEnabled()

    result = ApplicationCommandState()
    result.context_id = context_id
    result.intent = intent.intent
    result.state = ApplicationCommandState.STATE_SUCCEEDED
    controls.handle_application_state(result)

    assert controls.move_to_base_position_button.isEnabled()


def test_deletion_submits_typed_intent_without_local_mutation(controls):
    controls.apply_setup_state(state("motor", "magnetic_scan"))

    assert controls.handle_delete_routine() is True
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_DELETE_ROUTINE
    )

    assert controls.handle_delete_object() is True
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_DELETE_OBJECT
    )
