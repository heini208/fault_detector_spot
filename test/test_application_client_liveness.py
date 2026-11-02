"""UI reconnects preserve pending commands and require fresh owner evidence."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtWidgets import QApplication
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent
from rclpy.clock import ClockType

from fault_detector_spot.ui.ros import application_client as module


@pytest.fixture(scope="module", autouse=True)
def application():
    application = QApplication.instance() or QApplication([])
    yield application


def state(sequence, *, owner="owner-a", code=ApplicationCommandState.STATE_RUNNING, context="context"):
    return ApplicationCommandState(
        client_id="ui", context_id=context, request_id="request",
        execution_instance_id=owner, status_sequence=sequence, state=code,
    )


@pytest.fixture
def client(monkeypatch, request):
    transport, node = Mock(), Mock()
    goal_response, result_response = Future(), Future()
    transport.send_goal_async.return_value = goal_response
    monkeypatch.setattr(module, "ActionClient", Mock(return_value=transport))
    now = [0.0]
    adapter = module.ApplicationClient(node, "ui", monotonic_clock=lambda: now[0])
    context = getattr(request, "param", "context")
    local_id = adapter.execute(OperationalIntent(), context)
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_response
    goal_response.set_result(handle)
    yield adapter, local_id, now, transport, node, result_response
    adapter.destroy()


def test_silent_connection_loss_retains_acknowledged_request_and_rejects_stale_recovery(client):
    adapter, local_id, now, transport, node, result_response = client
    uncertain, received = [], []
    adapter.operation_uncertain.connect(lambda *args: uncertain.append(args))
    adapter.state_changed.connect(received.append)
    adapter._receive_feedback(local_id, SimpleNamespace(feedback=SimpleNamespace(state=state(10))))
    assert node.create_timer.call_args.kwargs["clock"].clock_type == ClockType.STEADY_TIME

    now[0] = 5.1
    adapter._poll_liveness()
    adapter._poll_liveness()
    assert len(uncertain) == 1
    assert local_id in adapter._goal_handles
    assert local_id in adapter._pending_operations
    assert adapter._pending_operations[local_id].uncertain
    assert transport.send_goal_async.call_count == 1

    for stale in (state(100, owner="owner-b", code=ApplicationCommandState.STATE_STOP_CONFIRMED),
                  state(9, code=ApplicationCommandState.STATE_STOP_CONFIRMED), state(10)):
        adapter._receive_state(stale)
        assert adapter._pending_operations[local_id].uncertain
        assert local_id in adapter._goal_handles
    assert len(received) == 1

    adapter._receive_state(state(11))
    assert not adapter._pending_operations[local_id].uncertain
    now[0] = 10.2
    adapter._poll_liveness()
    assert len(uncertain) == 2
    adapter._receive_state(state(12, code=ApplicationCommandState.STATE_STOP_CONFIRMED))
    assert not adapter._pending_operations
    assert not adapter._goal_handles
    result_response.set_exception(RuntimeError("Late result channel failure"))
    assert len(uncertain) == 2


def test_result_loss_keeps_handle_until_fresh_correlated_terminal_state(client):
    adapter, local_id, _now, transport, _node, result_response = client
    adapter._receive_feedback(local_id, SimpleNamespace(feedback=SimpleNamespace(state=state(1))))
    result_response.set_exception(RuntimeError("Disconnected"))
    assert local_id in adapter._goal_handles
    assert adapter._pending_operations[local_id].uncertain
    adapter._receive_state(state(2, code=ApplicationCommandState.STATE_SUCCEEDED))
    assert not adapter._goal_handles
    assert not adapter._pending_operations
    assert transport.send_goal_async.call_count == 1


def test_goal_ack_without_feedback_still_expires_to_uncertainty(client):
    adapter, local_id, now, _transport, _node, _result_response = client
    uncertain = []
    adapter.operation_uncertain.connect(lambda *args: uncertain.append(args))
    now[0] = 2.9
    adapter._poll_liveness()
    assert not uncertain
    now[0] = 3.1
    adapter._poll_liveness()
    assert uncertain
    assert local_id in adapter._goal_handles
    # The API topic can establish the identity for a unique pending context.
    adapter._receive_state(state(1))
    assert adapter._pending_operations[local_id].request_id == "request"
    assert not adapter._pending_operations[local_id].uncertain
    now[0] = 8.0
    adapter._poll_liveness()
    assert len(uncertain) == 1
    now[0] = 8.2
    adapter._poll_liveness()
    assert len(uncertain) == 2


@pytest.mark.parametrize("client", [""], indirect=True)
@pytest.mark.parametrize("channel", ["feedback", "result"])
def test_topic_first_terminal_state_can_complete_through_exact_duplicate_action_state(client, channel):
    adapter, local_id, now, _transport, _node, result_response = client
    received = []
    adapter.state_changed.connect(received.append)
    done = state(2, context="", code=ApplicationCommandState.STATE_STOP_CONFIRMED)
    adapter._receive_state(done)
    assert adapter._pending_operations[local_id].request_id == ""
    assert local_id in adapter._goal_handles
    now[0] = 2.0
    if channel == "feedback":
        adapter._receive_feedback(local_id, SimpleNamespace(feedback=SimpleNamespace(state=done)))
    else:
        result_response.set_result(SimpleNamespace(result=SimpleNamespace(state=done)))
    assert not adapter._pending_operations
    assert not adapter._goal_handles
    assert received == [done]


@pytest.mark.parametrize("client", [""], indirect=True)
def test_topic_first_feedback_binds_without_refreshing_liveness_or_accepting_mismatches(client):
    adapter, local_id, now, _transport, _node, _result_response = client
    running = state(1, context="")
    adapter._receive_state(running)
    now[0] = 2.0
    for invalid in (
        state(1, context="", code=ApplicationCommandState.STATE_STOP_CONFIRMED),
        state(1, context="", owner="owner-b"),
        state(1, context="other"),
    ):
        adapter._receive_feedback(local_id, SimpleNamespace(feedback=SimpleNamespace(state=invalid)))
        assert adapter._pending_operations[local_id].request_id == ""
    feedback = SimpleNamespace(feedback=SimpleNamespace(state=running))
    adapter._receive_feedback(local_id, feedback)
    pending = adapter._pending_operations[local_id]
    assert pending.request_id == "request"
    assert pending.last_seen == 0.0
    now[0] = 5.1
    adapter._poll_liveness()
    assert pending.uncertain
    adapter._receive_feedback(local_id, feedback)
    assert pending.uncertain
    assert pending.last_seen == 0.0
    adapter._receive_feedback(local_id, SimpleNamespace(feedback=SimpleNamespace(state=state(2, context=""))))
    assert not pending.uncertain
    assert pending.last_seen == 5.1


def test_ambiguous_send_exception_keeps_context_and_visible_uncertainty(monkeypatch):
    from PyQt5.QtWidgets import QLabel
    from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI

    transport = Mock()
    transport.send_goal_async.side_effect = RuntimeError("send outcome lost")
    monkeypatch.setattr(module, "ActionClient", Mock(return_value=transport))
    adapter = module.ApplicationClient(Mock(), "ui")
    harness = SimpleNamespace(application_client=adapter, status_label=QLabel())
    adapter.operation_uncertain.connect(
        lambda context, detail: Fault_Detector_UI._process_operation_uncertain(harness, context, detail)
    )
    try:
        local_id = Fault_Detector_UI.execute_operation(harness, OperationalIntent(), "context")
        assert local_id in adapter._pending_operations
        assert local_id in adapter._goal_handles
        assert "outcome unknown" in harness.status_label.text()
        assert transport.send_goal_async.call_count == 1
    finally:
        adapter.destroy()


def test_queued_request_needs_its_own_controller_heartbeat_while_other_work_runs(client):
    from builtin_interfaces.msg import Time
    from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
    from fault_detector_spot.application.controllers.application_controller import ApplicationController
    from fault_detector_spot.application.controllers.command_controller import (
        CommandController, CommandControllerState, CommandExecutionStatus,
    )

    adapter, local_id, now, _transport, _node, _result_response = client
    dispatched, published, uncertainties = [], [], []
    adapter.operation_uncertain.connect(lambda *args: uncertainties.append(args))
    commands = CommandController(dispatched.append, monotonic_clock=lambda: now[0])
    application = ApplicationController(commands)
    api = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        _public_state=ApplicationApiNode._public_state,
    )
    deliver_queued_updates = [True]

    def receive_status(status):
        message = ApplicationApiNode._state_message(api, status)
        published.append(message)
        if deliver_queued_updates[0] or message.state != message.STATE_QUEUED:
            adapter._receive_state(message)

    application.add_status_listener(receive_status)
    intent = OperationalIntent(intent=OperationalIntent.INTENT_STAND_UP)
    active = application.prepare_operation(intent, "ui", "active-context")
    queued = application.prepare_operation(intent, "ui", "context")
    application.submit(active)
    application.submit(queued)

    def advance(instant):
        now[0] = float(instant)
        assert commands.handle_execution_status(CommandExecutionStatus(
            active.request_id, CommandControllerState.RUNNING,
            execution_instance_id="healthy-owner", status_sequence=instant,
        ))
        commands.poll()
        adapter._poll_liveness()

    # Real queued workflows can outlast several heartbeat deadlines without
    # reaching the execution owner. Only the controller certifies this queue.
    for instant in range(1, 13):
        advance(instant)
        assert adapter._pending_operations[local_id].request_id == queued.request_id
        assert not adapter._pending_operations[local_id].uncertain
    queued_states = [message for message in published if message.request_id == queued.request_id]
    assert len(queued_states) > 2
    assert all(message.state == message.STATE_QUEUED for message in queued_states)
    assert all(message.execution_instance_id == "" for message in queued_states)
    sequences = [message.status_sequence for message in queued_states]
    assert all(first < second for first, second in zip(sequences, sequences[1:]))
    assert not uncertainties
    assert dispatched == [active.request]

    # Other requests' healthy updates cannot hide the loss of this request's
    # public state stream or release its pending ownership.
    deliver_queued_updates[0] = False
    for instant in range(13, 19):
        advance(instant)
    assert adapter._pending_operations[local_id].uncertain
    assert local_id in adapter._goal_handles
    assert commands.queued_request_ids == (queued.request_id,)
    assert dispatched == [active.request]
    assert len(uncertainties) == 1
