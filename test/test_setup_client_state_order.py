"""Setup clients must not render stale revisions or regress terminal results."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtWidgets import QApplication
from fault_detector_msgs.msg import (
    NavigationSetupIntent, NavigationSetupState, ProbeSetupMotionIntent, ProbeSetupState,
)

import fault_detector_spot.ui.ros.navigation_setup_client as navigation_module
import fault_detector_spot.ui.ros.probe_setup_client as probe_module


@pytest.fixture(scope="module", autouse=True)
def qt_application():
    application = QApplication.instance() or QApplication([])
    yield application


@pytest.fixture(params=[navigation_module, probe_module])
def client(request, monkeypatch):
    module = request.param
    monkeypatch.setattr(module, "ActionClient", Mock())
    client_type = (module.NavigationSetupClient if module is navigation_module
                   else module.ProbeSetupClient)
    value = client_type(Mock(), "ui")
    value.context_id = "context"
    received = []
    value.state_changed.connect(received.append)
    message_type = NavigationSetupState if module is navigation_module else ProbeSetupState
    return value, received, message_type


def state(message_type, revision, code, request="request", context="context"):
    message = message_type()
    message.client_id = "ui"
    message.context_id = context
    message.request_id = request
    message.revision = revision
    message.state = code
    return message


def test_older_revision_cannot_replace_newer_snapshot(client):
    adapter, received, message_type = client
    newer = state(message_type, 5, message_type.STATE_READY)
    adapter._receive_state(newer)
    adapter._receive_state(state(message_type, 4, message_type.STATE_SUCCEEDED))
    assert received == [newer]


@pytest.mark.parametrize("terminal", [
    "STATE_SUCCEEDED", "STATE_FAILED", "STATE_CANCELLED", "STATE_STOP_CONFIRMED",
])
def test_terminal_state_cannot_regress_but_new_request_can_progress(client, terminal):
    adapter, received, message_type = client
    done = state(message_type, 5, getattr(message_type, terminal))
    adapter._emit_state(done)
    adapter._emit_state(state(message_type, 5, message_type.STATE_RUNNING))
    adapter._emit_state(state(message_type, 5, message_type.STATE_QUEUED))
    assert received == [done]
    next_request = state(message_type, 5, message_type.STATE_RUNNING, request="next")
    adapter._emit_state(next_request)
    assert received == [done, next_request]
    adapter._emit_state(state(message_type, 5, message_type.STATE_QUEUED, request="next"))
    assert received == [done, next_request]


def test_closed_context_cannot_be_resurrected_and_new_context_resets_revision(client):
    adapter, received, message_type = client
    old = state(message_type, 10, message_type.STATE_SUCCEEDED)
    adapter._emit_state(old)
    future = Mock()
    future.result.return_value = SimpleNamespace(closed=True, detail="closed")
    adapter._receive_close(future)
    adapter._receive_state(old)
    assert adapter.context_id == ""
    assert received == [old]
    new = state(message_type, 1, message_type.STATE_READY, context="new-context")
    adapter._receive_state(new)
    assert adapter.context_id == "new-context"
    assert received == [old, new]


@pytest.mark.parametrize("result_method,feedback_method", [
    ("_receive_result", "_receive_feedback"),
    ("_receive_motion_result", "_receive_motion_feedback"),
    ("_receive_finalization_result", "_receive_finalization_feedback"),
    ("_receive_capture_result", "_receive_capture_feedback"),
])
def test_late_action_feedback_cannot_replace_result(monkeypatch, result_method, feedback_method):
    module = navigation_module if result_method == "_receive_result" else probe_module
    monkeypatch.setattr(module, "ActionClient", Mock())
    adapter = (module.NavigationSetupClient(Mock(), "ui") if module is navigation_module
               else module.ProbeSetupClient(Mock(), "ui"))
    message_type = NavigationSetupState if module is navigation_module else ProbeSetupState
    received = []
    adapter.state_changed.connect(received.append)
    done = state(message_type, 3, message_type.STATE_SUCCEEDED)
    future = Mock()
    future.result.return_value = SimpleNamespace(result=SimpleNamespace(state=done))
    getattr(adapter, result_method)("local", future)
    feedback = SimpleNamespace(feedback=SimpleNamespace(
        state=state(message_type, 3, message_type.STATE_RUNNING),
    ))
    getattr(adapter, feedback_method)("local", feedback)
    assert received == [done]


def test_stale_capture_result_does_not_refresh_old_previews(monkeypatch):
    monkeypatch.setattr(probe_module, "ActionClient", Mock())
    adapter = probe_module.ProbeSetupClient(Mock(), "ui")
    adapter._refresh_reference_previews = Mock()
    adapter._emit_state(state(ProbeSetupState, 4, ProbeSetupState.STATE_READY))
    future = Mock()
    future.result.return_value = SimpleNamespace(result=SimpleNamespace(
        state=state(ProbeSetupState, 3, ProbeSetupState.STATE_SUCCEEDED),
    ))
    adapter._receive_capture_result("local", future)
    adapter._refresh_reference_previews.assert_not_called()


@pytest.fixture(params=["navigation", "motion", "finalization", "capture"])
def pending_action(request, monkeypatch):
    kind = request.param
    module = navigation_module if kind == "navigation" else probe_module
    monkeypatch.setattr(module, "ActionClient", Mock())
    adapter = (module.NavigationSetupClient(Mock(), "ui") if kind == "navigation"
               else module.ProbeSetupClient(Mock(), "ui"))
    adapter.context_id = "context"
    if kind == "navigation":
        local_id = adapter.execute(NavigationSetupIntent(
            operation=NavigationSetupIntent.OPERATION_START_MAPPING,
        ))
        methods = ("_receive_goal_response", "_receive_result", "_receive_feedback")
        handles = adapter._goal_handles
        message_type = NavigationSetupState
    else:
        if kind == "motion":
            local_id = adapter.execute_motion(ProbeSetupMotionIntent())
        elif kind == "finalization":
            local_id = adapter.finalize_refinement(False)
        else:
            local_id = adapter.capture_reference_views(["hand", "frontleft", "frontright"])
        methods = tuple(f"_receive_{kind}_{phase}" for phase in ("goal", "result", "feedback"))
        handles = getattr(adapter, f"_{kind}_goal_handles")
        message_type = ProbeSetupState
    assert local_id in handles
    return adapter, local_id, handles, methods, message_type


@pytest.mark.parametrize("phase", [0, 1], ids=["goal-response", "result"])
def test_ambiguous_setup_response_keeps_busy_until_correlated_owner_confirmation(pending_action, phase):
    adapter, local_id, handles, methods, message_type = pending_action
    uncertain, rejected, close_results = [], [], []
    adapter.request_uncertain.connect(uncertain.append)
    adapter.request_rejected.connect(rejected.append)
    adapter.close_finished.connect(lambda *args: close_results.append(args))
    feedback = lambda code: SimpleNamespace(feedback=SimpleNamespace(
        state=state(message_type, 3, code),
    ))
    # Feedback establishes the server identity; a lost action response does not
    # erase it or release the client's close guard.
    getattr(adapter, methods[2])(local_id, feedback(message_type.STATE_RUNNING))
    failed = Mock()
    failed.result.side_effect = RuntimeError("response lost")
    getattr(adapter, methods[phase])(local_id, failed)
    assert uncertain == ["response lost"]
    assert not rejected
    assert local_id in handles
    assert adapter.close() is None
    adapter._close_client.call_async.assert_not_called()
    assert close_results[-1][0] is False

    adapter._receive_state(state(message_type, 3, message_type.STATE_STOP_CONFIRMED, request="other"))
    assert local_id in handles
    for code in (message_type.STATE_UNKNOWN, message_type.STATE_STOP_REQUESTED,
                 message_type.STATE_STOP_UNCONFIRMED):
        adapter._receive_state(state(message_type, 3, code))
        assert local_id in handles
    adapter._receive_state(state(message_type, 3, message_type.STATE_STOP_CONFIRMED))
    assert local_id not in handles
    adapter.close()
    adapter._close_client.call_async.assert_called_once()


def test_explicit_setup_goal_rejection_releases_only_its_pending_guard(pending_action):
    adapter, local_id, handles, methods, _ = pending_action
    other = object()
    handles["other"] = other
    rejected, uncertain = [], []
    adapter.request_rejected.connect(rejected.append)
    adapter.request_uncertain.connect(uncertain.append)
    future = Mock()
    future.result.return_value = SimpleNamespace(accepted=False)
    getattr(adapter, methods[0])(local_id, future)
    assert local_id not in handles
    assert handles["other"] is other
    assert rejected
    assert not uncertain


def test_setup_result_request_exception_retains_accepted_goal(pending_action):
    adapter, local_id, handles, methods, _ = pending_action
    uncertain, rejected = [], []
    adapter.request_uncertain.connect(uncertain.append)
    adapter.request_rejected.connect(rejected.append)
    goal = Mock(accepted=True)
    goal.get_result_async.side_effect = RuntimeError("result request lost")
    future = Mock()
    future.result.return_value = goal
    getattr(adapter, methods[0])(local_id, future)
    assert handles[local_id] is goal
    assert uncertain == ["result request lost"]
    assert not rejected


def test_navigation_open_ready_result_releases_pending_close_guard(monkeypatch):
    monkeypatch.setattr(navigation_module, "ActionClient", Mock())
    adapter = navigation_module.NavigationSetupClient(Mock(), "ui")
    local_id = adapter.open()
    ready = state(NavigationSetupState, 1, NavigationSetupState.STATE_READY)
    ready.operation = NavigationSetupIntent.OPERATION_OPEN
    future = Mock()
    future.result.return_value = SimpleNamespace(result=SimpleNamespace(state=ready))
    adapter._receive_result(local_id, future)
    assert adapter.context_id == "context"
    assert not adapter._goal_handles
    adapter.close()
    adapter._close_client.call_async.assert_called_once()
