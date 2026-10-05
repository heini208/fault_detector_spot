"""Setup clients must not render stale revisions or regress terminal results."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import NavigationSetupState, ProbeSetupState

import fault_detector_spot.ui.ros.navigation_setup_client as navigation_module
import fault_detector_spot.ui.ros.probe_setup_client as probe_module


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


@pytest.mark.parametrize("terminal", ["STATE_SUCCEEDED", "STATE_FAILED", "STATE_CANCELLED"])
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
