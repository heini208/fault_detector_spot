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
