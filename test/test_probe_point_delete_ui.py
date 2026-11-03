"""Saved-point deletion requires confirmation and an authoritative response."""

from unittest.mock import Mock

from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_probe_point_guided_workflow import FakeUI, application, make_state


def test_saved_probe_delete_waits_for_confirmation_and_authoritative_result(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two"]
    controls.apply_setup_state(state)
    controls.saved_probe_points_list.setCurrentRow(1)
    controls.ask_question = Mock(return_value=False)

    controls.delete_saved_probe_point_button.click()

    assert ui.requests == []
    assert controls.saved_probe_points_list.count() == 2
    controls.ask_question.return_value = True
    controls.delete_saved_probe_point_button.click()

    [request] = ui.requests
    assert request.operation == ProbeSetupIntent.OPERATION_DELETE_PROBE_POINT
    assert (request.object_id, request.routine_id, request.probe_point_id) == (
        "motor", "scan", "two",
    )
    assert controls.saved_probe_points_list.count() == 2
    assert not controls.saved_probe_points_list.isEnabled()
    assert not controls.delete_saved_probe_point_button.isEnabled()
    assert not controls.handle_delete_saved_probe_point()
    assert len(ui.requests) == 1

    state.operation = ProbeSetupIntent.OPERATION_DELETE_PROBE_POINT
    state.state = ProbeSetupState.STATE_SUCCEEDED
    state.probe_point_ids = ["one"]
    controls.apply_setup_state(state)

    assert controls.saved_probe_points_list.count() == 1
    assert controls.saved_probe_points_list.item(0).text() == "one"
    assert controls.saved_probe_points_list.currentItem() is None
    assert controls.saved_probe_points_list.isEnabled()
    assert not controls.delete_saved_probe_point_button.isEnabled()
    assert controls.saved_probe_motion_status.text() == "two: deleted."
    controls.refinement_dialog.close()
