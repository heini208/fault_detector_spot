"""Regression guards for reference-preview request lifecycle."""

import inspect

from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient


def test_preview_request_is_retained_until_service_is_ready():
    source = inspect.getsource(ProbeSetupClient.request_preview)

    assert "_pending_preview_requests[view_id] = generation" in source
    assert "_preview_generations" in source


def test_pending_previews_have_a_retry_path():
    source = inspect.getsource(ProbeSetupClient._flush_pending_previews)

    assert "service_is_ready()" in source
    assert "_send_preview_request" in source


def test_capture_completion_forces_fresh_preview_bytes():
    source = inspect.getsource(ProbeSetupClient._receive_capture_result)

    assert "_refresh_reference_previews(result.state)" in source


def test_old_preview_response_cannot_overwrite_newer_generation():
    source = inspect.getsource(ProbeSetupClient._receive_preview)

    assert "_preview_generations.get(view_id) != generation" in source


def test_failed_capture_does_not_reload_unchanged_previews():
    from types import SimpleNamespace
    from fault_detector_msgs.msg import ProbeSetupState

    refreshed = []
    emitted = []
    client = SimpleNamespace(
        _capture_goal_handles={'request': object()},
        _emit_state=lambda state: emitted.append(state) or True,
        _refresh_reference_previews=refreshed.append,
    )
    for state_code in (ProbeSetupState.STATE_FAILED,
                       ProbeSetupState.STATE_CANCELLED,
                       ProbeSetupState.STATE_SUCCEEDED):
        state = ProbeSetupState()
        state.state = state_code
        future = SimpleNamespace(result=lambda: SimpleNamespace(
            result=SimpleNamespace(state=state),
        ))
        ProbeSetupClient._receive_capture_result(client, 'request', future)
    assert len(emitted) == 3
    assert [state.state for state in refreshed] == [ProbeSetupState.STATE_SUCCEEDED]


def test_capture_refresh_preloads_only_hand_and_front_cameras():
    from types import SimpleNamespace
    from test_single_reference_preview_first_pass import make_state

    requests = []
    client = SimpleNamespace(request_preview=requests.append)
    ProbeSetupClient._refresh_reference_previews(client, make_state())
    assert requests == ['slot6_hand', 'slot1_frontleft', 'slot2_frontright']
