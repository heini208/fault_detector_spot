"""Metadata shares admission with asynchronous setup and close cancels ownership."""

from types import SimpleNamespace
from unittest.mock import Mock
from uuid import uuid4

from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from fault_detector_msgs.srv import CloseSetup, ExecuteProbeSetup

from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi


def api_with_admission():
    admission = OperationAdmission()
    coordinator, adapter = Mock(), Mock()
    context = SimpleNamespace(context_id=str(uuid4()), client_id="ui")
    coordinator.context.return_value = context
    adapter.message.return_value = ProbeSetupState()
    api = ProbeSetupApi(Mock(), coordinator, Mock(), adapter, Mock(), admission=admission)
    return api, admission, context


def test_async_context_owner_rejects_metadata_without_mutation():
    api, admission, context = api_with_admission()
    assert admission.reserve("capture", context_id=context.context_id, lane="physical")
    assert admission.claim("capture")
    request = ExecuteProbeSetup.Request(client_id="ui", context_id=context.context_id)
    request.intent.operation = ProbeSetupIntent.OPERATION_SELECT_OBJECT

    api._execute(request, ExecuteProbeSetup.Response())

    api.coordinator.select_object.assert_not_called()
    assert not admission.shutdown_status()[0]
    assert api.state_adapter.message.call_args.args[2] == ProbeSetupState.STATE_FAILED


def test_close_cancels_active_context_before_domain_close_without_releasing_it():
    api, admission, context = api_with_admission()
    assert admission.reserve("capture", context_id=context.context_id, lane="physical")
    assert admission.claim("capture")

    def close(closing_context):
        assert closing_context == context
        assert admission.cancel_requested("capture")
        assert not admission.shutdown_status()[0]

    api.coordinator.close_context.side_effect = close
    result = api._close(CloseSetup.Request(client_id="ui", context_id=context.context_id),
                        CloseSetup.Response())
    assert result.closed
    assert not admission.shutdown_status()[0]


def test_metadata_releases_reservation_after_failure():
    api, admission, context = api_with_admission()
    api.coordinator.select_object.side_effect = RuntimeError("Missing object")
    request = ExecuteProbeSetup.Request(client_id="ui", context_id=context.context_id)
    request.intent.operation = ProbeSetupIntent.OPERATION_SELECT_OBJECT

    api._execute(request, ExecuteProbeSetup.Response())

    assert admission.shutdown_status()[0]
