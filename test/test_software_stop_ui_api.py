"""Software-stop presentation never equates reset acceptance with recovery."""

from concurrent.futures import Future
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
from fault_detector_msgs.msg import OperationalIntent, SoftwareStopState
from fault_detector_msgs.srv import EmergencyStop, ResetSoftwareStop

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.ros import application_client as module


@pytest.fixture(scope="module", autouse=True)
def application():
    application = QApplication.instance() or QApplication([])
    yield application


@pytest.fixture
def client(monkeypatch):
    transport = Mock()
    transport.send_goal_async.return_value = Future()
    monkeypatch.setattr(module, "ActionClient", Mock(return_value=transport))
    clients = {}
    node = Mock()
    node.create_client.side_effect = lambda _type, name: clients.setdefault(name, Mock())
    adapter = module.ApplicationClient(node, "operator-ui")
    yield adapter, transport, clients
    adapter.destroy()


def stop_state(sequence, *, latched=True, confirmed=False):
    return SoftwareStopState(
        request_id="stop", latched=latched, stop_confirmed=confirmed,
        status_sequence=sequence,
    )


def test_local_stop_blocks_motion_even_if_service_is_unavailable(client):
    adapter, transport, clients = client
    clients["fault_detector/application/emergency_stop"].service_is_ready.return_value = False
    states, errors = [], []
    adapter.software_stop_state_changed.connect(states.append)
    adapter.emergency_stop_finished.connect(lambda *values: errors.append(values))

    assert adapter.emergency_stop() is None
    assert states[-1].latched
    assert not states[-1].stop_confirmed
    assert not errors[-1][0]
    # A retained pre-stop idle observation cannot silently release this stop.
    adapter._receive_software_stop_state(stop_state(1, latched=False))
    assert adapter.execute(OperationalIntent(intent=OperationalIntent.INTENT_STAND_UP)) is None
    transport.send_goal_async.assert_not_called()


def test_reset_acceptance_does_not_clear_latch_or_start_recovery(client):
    adapter, transport, clients = client
    adapter._receive_software_stop_state(stop_state(10, confirmed=True))
    reset = clients["fault_detector/application/reset_software_stop"]
    result = Future()
    reset.call_async.return_value = result

    assert adapter.reset_software_stop() is result
    result.set_result(ResetSoftwareStop.Response(accepted=True, request_id="reset"))

    assert adapter._software_stop_state.latched
    assert adapter.execute(OperationalIntent(intent=OperationalIntent.INTENT_RECOVER_ARM)) is None
    transport.send_goal_async.assert_not_called()

    adapter._receive_software_stop_state(stop_state(9, latched=False))
    assert adapter._software_stop_state.latched
    adapter._receive_software_stop_state(stop_state(11, latched=False))
    assert not adapter._software_stop_state.latched
    transport.send_goal_async.assert_not_called()

    assert adapter.execute(OperationalIntent(intent=OperationalIntent.INTENT_RECOVER_ARM))
    assert transport.send_goal_async.call_count == 1
    assert transport.send_goal_async.call_args.args[0].intent.intent == OperationalIntent.INTENT_RECOVER_ARM


def test_stop_during_reset_rejects_previous_stop_clear_feedback(client):
    adapter, transport, clients = client
    adapter._receive_software_stop_state(stop_state(1, confirmed=True))
    stop_response = Future()
    clients["fault_detector/application/emergency_stop"].call_async.return_value = stop_response

    adapter.emergency_stop()
    adapter._receive_software_stop_state(stop_state(2, confirmed=True))
    adapter._receive_software_stop_state(stop_state(3, latched=False))
    assert adapter._software_stop_requested_locally
    stop_response.set_result(EmergencyStop.Response(accepted=True, request_id="new-stop"))
    adapter._receive_software_stop_state(stop_state(4, latched=False))
    assert adapter._software_stop_requested_locally
    adapter._receive_software_stop_state(SoftwareStopState(
        request_id="new-stop", latched=True, stop_confirmed=True, status_sequence=5,
    ))
    assert adapter.execute(OperationalIntent(intent=OperationalIntent.INTENT_STAND_UP)) is None
    adapter._receive_software_stop_state(SoftwareStopState(
        request_id="new-stop", latched=False, status_sequence=6,
    ))
    assert not adapter._software_stop_requested_locally
    transport.send_goal_async.assert_not_called()


def test_repeated_stop_presentation_never_offers_reset_without_confirmation(client):
    adapter, _transport, clients = client
    clients["fault_detector/application/emergency_stop"].call_async.return_value = Future()
    adapter._receive_software_stop_state(stop_state(1, confirmed=True))
    states = []
    adapter.software_stop_state_changed.connect(states.append)

    adapter.emergency_stop()
    adapter.emergency_stop()

    assert all(state.latched and not state.stop_confirmed for state in states)
    adapter._receive_software_stop_state(stop_state(2, confirmed=True))
    assert states[-1].latched and states[-1].stop_confirmed


def test_reset_api_reports_request_acceptance_without_claiming_completion():
    harness = SimpleNamespace(application_controller=Mock(), _shutdown=Event(), _startup_unresolved={})
    harness.application_controller.reset_software_stop.return_value = "reset-request"

    response = ApplicationApiNode._reset_software_stop(
        harness, ResetSoftwareStop.Request(client_id="operator-ui"), ResetSoftwareStop.Response(),
    )

    assert response.accepted and response.request_id == "reset-request"
    assert "awaiting owner confirmation" in response.detail
    harness.application_controller.reset_software_stop.assert_called_once_with("operator-ui")
    harness.application_controller.emergency_stop.assert_not_called()


def test_reset_api_preserves_missing_stop_confirmation_failure():
    harness = SimpleNamespace(application_controller=Mock(), _shutdown=Event(), _startup_unresolved={})
    harness.application_controller.reset_software_stop.side_effect = RuntimeError("Base stop remains unconfirmed")

    response = ApplicationApiNode._reset_software_stop(
        harness, ResetSoftwareStop.Request(client_id="operator-ui"), ResetSoftwareStop.Response(),
    )

    assert not response.accepted
    assert response.detail == "Base stop remains unconfirmed"


def test_ui_reset_and_recovery_are_distinct_explicit_actions():
    harness = SimpleNamespace(
        application_client=Mock(), status_label=QLabel(),
        handle_simple_operation=Mock(), software_stop_label=QLabel(),
        reset_software_stop_button=QPushButton(), recover_arm_button=QPushButton(),
    )
    Fault_Detector_UI._process_software_stop_state(harness, stop_state(1))
    assert not harness.reset_software_stop_button.isEnabled()
    assert not harness.recover_arm_button.isEnabled()
    Fault_Detector_UI._process_software_stop_state(harness, stop_state(2, confirmed=True))
    assert harness.reset_software_stop_button.isEnabled()
    assert not harness.recover_arm_button.isEnabled()

    Fault_Detector_UI.handle_reset_software_stop(harness)

    harness.application_client.reset_software_stop.assert_called_once_with()
    harness.handle_simple_operation.assert_not_called()
    Fault_Detector_UI._process_software_stop_reset_result(harness, True, "Accepted")
    assert not harness.recover_arm_button.isEnabled()
    Fault_Detector_UI._process_software_stop_state(harness, stop_state(3, latched=False))
    assert not harness.reset_software_stop_button.isEnabled()
    assert harness.recover_arm_button.isEnabled()
    Fault_Detector_UI.handle_recover_arm(harness)
    harness.handle_simple_operation.assert_called_once_with(OperationalIntent.INTENT_RECOVER_ARM)
