"""Routine navigation uses saved server assignments and queued operations."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtCore import Qt

from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent, ProbeSetupIntent, ProbeSetupState
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient
from test_probe_point_guided_workflow import FakeUI, application, make_state


@pytest.fixture
def controls(application):
    ui = FakeUI()
    ui.execute_operation = Mock(return_value="operation-request")
    return FinalizingInspectionControls(ui)


def navigation_state(map_name="factory", waypoint_name="wall"):
    state = make_state(with_references=False)
    state.routine_map_name = map_name
    state.routine_waypoint_name = waypoint_name
    state.navigation_map_names = ["factory", "workshop"]
    state.routine_waypoint_names = ["wall", "entry"] if map_name else []
    return state


def select(combo, name):
    index = combo.findData(name)
    assert index >= 0
    combo.setCurrentIndex(index)


def test_saved_snapshot_populates_inactive_map_choices_and_assignment_labels(controls):
    state = navigation_state()
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls

    assert nav.map_dropdown.currentData() == "factory"
    assert nav.waypoint_dropdown.currentData() == "wall"
    assert nav.waypoint_dropdown.isEnabled()
    assert nav.map_name_label.text() == "factory"
    assert nav.waypoint_name_label.text() == "wall"
    assert "#2e7d32" in nav.map_name_label.styleSheet()
    assert nav.map_name_label.textFormat() == Qt.PlainText
    assert nav.launch_map_button.isEnabled()
    assert nav.move_to_waypoint_button.isEnabled()
    assert not nav.set_map_button.isEnabled()
    assert not nav.set_waypoint_button.isEnabled()
    assert controls.ui.requests == []
    controls.ui.execute_operation.assert_not_called()


def test_map_save_waits_for_server_before_changing_labels_and_waypoints(controls):
    state = navigation_state()
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls
    select(nav.map_dropdown, "workshop")
    controls.apply_setup_state(state)
    assert nav.map_dropdown.currentData() == "workshop"
    assert nav.map_name_label.text() == "factory"
    assert nav.waypoint_name_label.text() == "wall"
    assert not nav.set_waypoint_button.isEnabled()
    assert not nav.waypoint_dropdown.isEnabled()
    nav.set_map_button.click()
    assert len(controls.ui.requests) == 1
    sent = controls.ui.requests[-1]
    assert (sent.operation, sent.object_id, sent.routine_id, sent.map_name) == (
        ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP, "motor", "scan", "workshop",
    )
    assert not nav.handle_set_map()
    assert not nav.set_waypoint_button.isEnabled()
    assert not nav.launch_map_button.isEnabled()
    assert nav.map_name_label.text() == "factory"

    state.operation = sent.operation
    state.state = ProbeSetupState.STATE_SUCCEEDED
    state.routine_map_name = "workshop"
    state.routine_waypoint_name = ""
    state.routine_waypoint_names = ["bench"]
    controls.apply_setup_state(state)
    assert nav.map_name_label.text() == "workshop"
    assert nav.waypoint_name_label.text() == "Not set"
    assert "#c62828" in nav.waypoint_name_label.styleSheet()
    assert nav.waypoint_dropdown.currentData() == ""
    assert nav.waypoint_dropdown.findData("bench") > 0
    assert nav.waypoint_dropdown.findData("wall") == -1
    assert nav.launch_map_button.isEnabled()
    assert not nav.move_to_waypoint_button.isEnabled()
    select(nav.waypoint_dropdown, "bench")
    nav.set_waypoint_button.click()
    assert len(controls.ui.requests) == 2
    sent = controls.ui.requests[-1]
    assert (sent.operation, sent.map_name, sent.waypoint_name) == (
        ProbeSetupIntent.OPERATION_SAVE_ROUTINE_WAYPOINT, "workshop", "bench",
    )
    assert nav.waypoint_name_label.text() == "Not set"


def test_waypoint_draft_survives_refresh_but_does_not_cross_routines(controls):
    state = navigation_state()
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls
    select(nav.waypoint_dropdown, "entry")
    controls.apply_setup_state(state)
    assert nav.waypoint_dropdown.currentData() == "entry"
    assert nav.waypoint_name_label.text() == "wall"
    state.selected_routine_id = "other"
    state.routine_ids.append("other")
    state.routine_map_name = "workshop"
    state.routine_waypoint_name = "bench"
    state.routine_waypoint_names = ["bench"]
    controls.apply_setup_state(state)
    assert nav.map_dropdown.currentData() == "workshop"
    assert nav.waypoint_dropdown.currentData() == "bench"


@pytest.mark.parametrize("kind", ["map", "waypoint"])
def test_empty_choice_explicitly_clears_saved_assignment(controls, kind):
    controls.apply_setup_state(navigation_state())
    nav = controls.routine_navigation_controls
    if kind == "map":
        select(nav.map_dropdown, "")
        assert nav.handle_set_map()
        assert controls.ui.requests[-1].map_name == ""
    else:
        select(nav.waypoint_dropdown, "")
        assert nav.handle_set_waypoint()
        assert controls.ui.requests[-1].map_name == "factory"
        assert controls.ui.requests[-1].waypoint_name == ""
    assert nav.map_name_label.text() == "factory"
    assert nav.waypoint_name_label.text() == "wall"


def test_launch_and_move_queue_from_saved_assignments_despite_unsaved_drafts(controls):
    controls.apply_setup_state(navigation_state())
    nav = controls.routine_navigation_controls
    select(nav.map_dropdown, "workshop")
    nav.launch_map_button.click()
    nav.move_to_waypoint_button.click()
    nav.launch_map_button.click()
    calls = controls.ui.execute_operation.call_args_list
    assert [call.args[0].intent for call in calls] == [
        OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP,
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_WAYPOINT,
        OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP,
    ]
    assert len({call.kwargs["context_id"] for call in calls}) == 3
    for call in calls:
        assert (call.args[0].object_id, call.args[0].routine_id) == ("motor", "scan")
        assert call.args[0].map_name == ""
        assert call.args[0].waypoint_name == ""
    assert nav.set_map_button.isEnabled()
    assert nav.map_dropdown.isEnabled()
    state = ApplicationCommandState()
    state.context_id = calls[0].kwargs["context_id"]
    state.state = state.STATE_QUEUED
    controls.handle_application_state(state)
    assert "queued" in nav.status_label.text()
    nav.handle_operation_rejected("Connection failed", calls[1].kwargs["context_id"])
    assert len(nav._operation_contexts) == 2
    state.state = state.STATE_SUCCEEDED
    controls.handle_application_state(state)
    assert len(nav._operation_contexts) == 1
    assert nav.handle_set_map()


@pytest.mark.parametrize("field", ["refinement_active", "motion_pending", "queued", "running"])
def test_conflicting_setup_blocks_navigation_controls(controls, field):
    state = navigation_state()
    if field in {"queued", "running"}:
        state.state = (state.STATE_QUEUED if field == "queued" else state.STATE_RUNNING)
    else:
        setattr(state, field, True)
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls
    assert not nav.launch_map_button.isEnabled()
    assert not nav.move_to_waypoint_button.isEnabled()
    assert not nav.map_dropdown.isEnabled()
    assert not nav.waypoint_dropdown.isEnabled()


def test_failed_save_keeps_assignments_and_allows_retry(controls):
    state = navigation_state()
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls
    select(nav.map_dropdown, "workshop")
    assert nav.handle_set_map()
    state.operation = ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP
    state.state = ProbeSetupState.STATE_FAILED
    state.detail = "Map was removed"
    controls.apply_setup_state(state)
    assert nav.status_label.text() == "Map was removed"
    assert nav.map_name_label.text() == "factory"
    assert nav.map_dropdown.currentData() == "workshop"
    assert nav.handle_set_map()


def test_operation_rejection_is_correlated_after_routine_switch(controls):
    state = navigation_state()
    controls.apply_setup_state(state)
    nav = controls.routine_navigation_controls
    assert nav.handle_launch_map()
    old_context = controls.ui.execute_operation.call_args.kwargs["context_id"]
    state.selected_routine_id = "other"
    state.routine_ids.append("other")
    controls.apply_setup_state(state)
    old_status = nav.status_label.text()
    harness = SimpleNamespace(inspection_controls=controls)
    Fault_Detector_UI._process_operation_rejected(harness, old_context, "Old request failed")
    assert nav.status_label.text() == old_status
    assert not nav._operation_contexts


def test_unsubmitted_save_and_operation_can_be_retried(controls):
    controls.apply_setup_state(navigation_state())
    nav = controls.routine_navigation_controls
    select(nav.map_dropdown, "workshop")
    controls.ui.execute_probe_setup = Mock(return_value=None)
    assert not nav.handle_set_map()
    assert nav.set_map_button.isEnabled()
    controls.ui.execute_operation.return_value = None
    assert not nav.handle_launch_map()
    assert not nav._operation_contexts
    assert nav.launch_map_button.isEnabled()


def test_other_setup_rejection_does_not_unlock_pending_navigation_save(controls, monkeypatch):
    import fault_detector_spot.ui.ros.probe_setup_client as client_module

    response = Future()
    execute_service = Mock()
    execute_service.call_async.return_value = response
    node = Mock()
    node.create_client.return_value = execute_service
    monkeypatch.setattr(client_module, "ActionClient", Mock())
    client = ProbeSetupClient(node, "ui")
    client.context_id = "probe-context"
    controls.ui.execute_probe_setup = client.execute
    harness = SimpleNamespace(inspection_controls=controls)
    client.request_rejected.connect(
        lambda detail: Fault_Detector_UI._process_probe_setup_rejected(harness, detail)
    )
    client.transaction_failed.connect(
        lambda operation, detail: Fault_Detector_UI._process_probe_setup_transaction_failed(
            harness, operation, detail,
        )
    )
    controls.apply_setup_state(navigation_state())
    nav = controls.routine_navigation_controls
    select(nav.map_dropdown, "workshop")
    assert nav.handle_set_map()
    other = ProbeSetupIntent()
    other.operation = ProbeSetupIntent.OPERATION_REFRESH
    assert client.execute(other) is None
    assert not nav.set_map_button.isEnabled()
    response.set_exception(RuntimeError("Connection lost"))
    assert nav.set_map_button.isEnabled()
    assert nav.status_label.text() == "Connection lost"
