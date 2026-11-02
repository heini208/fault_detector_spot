"""Setup action lifetimes yield callback capacity until confirmed completion."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.action import ExecuteNavigationSetup, ExecuteProbeSetupMotion
from fault_detector_msgs.msg import NavigationSetupIntent, NavigationSetupState, ProbeSetupState
from rclpy.action import CancelResponse, GoalResponse

from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity
from fault_detector_spot.application.controllers.command_controller import CommandControllerState
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind
from test_command_request_correlation import FakeClock


@pytest.fixture(params=[NavigationSetupApi, ProbeSetupMotionApi])
def rig(request):
    api = request.param.__new__(request.param)
    api.node = Mock()
    api.node.get_clock.return_value = FakeClock()
    api._lock = RLock()
    api._shutdown = Event()
    api._shutdown_callbacks = ShutdownCallbackActivity()
    api._admission = OperationAdmission(limit=1)
    api._executions = {}
    api._early_states = {}
    api._runtime_submissions = 0
    api._pose_waiters = set()
    api._motion_request = Mock()
    api._state_publisher = api.state_publisher = Mock()
    api.coordinator = Mock()
    context = SimpleNamespace(context_id="context", client_id="client", revision=0)
    api.coordinator.context.return_value = context
    operation = SimpleNamespace(request_id="request")
    api.coordinator.prepare_motion.return_value = operation
    api.coordinator.submit_runtime_operation.return_value = operation
    if request.param is NavigationSetupApi:
        goal = ExecuteNavigationSetup.Goal(
            client_id="client", context_id="context",
            intent=NavigationSetupIntent(operation=NavigationSetupIntent.OPERATION_START_MAPPING),
        )
        api._state = lambda snapshot, operation, code, detail, request_id="": NavigationSetupState(
            state=code, detail=detail, request_id=request_id,
        )
        cancel = api.coordinator.cancel
        submit = api.coordinator.submit_runtime_operation
        state_type = NavigationSetupState
    else:
        goal = ExecuteProbeSetupMotion.Goal(client_id="client", context_id="context")
        api.state_adapter = Mock()
        api.state_adapter.message.side_effect = lambda snapshot, operation, code, detail, **kwargs: ProbeSetupState(
            state=code, detail=detail, request_id=kwargs.get("request_id", ""),
        )
        cancel = api.coordinator.cancel_motion
        submit = api.coordinator.submit_motion
        state_type = ProbeSetupState
    handle = Mock(request=goal, is_cancel_requested=False)
    assert api._accept(goal) == GoalResponse.ACCEPT

    def status(state):
        api._receive_status(SimpleNamespace(
            state=state, request_id="request", snapshot=object(), operation_code=0,
            detail=state.value, execution_instance_id="owner", status_sequence=1,
            motion=SimpleNamespace(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
        ))

    return SimpleNamespace(api=api, handle=handle, status=status, cancel=cancel,
                           submit=submit, operation=operation, state_type=state_type)


def finish(coroutine):
    with pytest.raises(StopIteration) as completed:
        coroutine.send(None)
    return completed.value.value


def test_one_callback_worker_can_dispatch_cancel_and_complete_setup(rig):
    coroutine = rig.api._execute(rig.handle)
    with ThreadPoolExecutor(max_workers=1) as callbacks:
        waiting = callbacks.submit(coroutine.send, None).result(timeout=1)
        assert not waiting.done()
        rig.handle.is_cancel_requested = True
        assert callbacks.submit(rig.api._accept_cancel, rig.handle).result(timeout=1) == CancelResponse.ACCEPT
        callbacks.submit(rig.api._accept_cancel, rig.handle).result(timeout=1)
        rig.cancel.assert_called_once()
        callbacks.submit(rig.status, CommandControllerState.STOP_UNCONFIRMED).result(timeout=1)
        assert not waiting.done()
        callbacks.submit(rig.status, CommandControllerState.STOP_CONFIRMED).result(timeout=1)
        result = callbacks.submit(finish, coroutine).result(timeout=1)
    assert result.state.state == rig.state_type.STATE_STOP_CONFIRMED
    rig.handle.canceled.assert_called_once()
    rig.handle.succeed.assert_not_called()
    rig.handle.abort.assert_not_called()
    assert not rig.api._executions
    assert rig.api._shutdown_callbacks.status()[0]


def test_duplicate_terminal_status_cannot_change_action_result(rig):
    coroutine = rig.api._execute(rig.handle)
    coroutine.send(None)
    rig.status(CommandControllerState.FAILED)
    rig.status(CommandControllerState.SUCCEEDED)
    result = finish(coroutine)
    assert result.state.state == rig.state_type.STATE_FAILED
    rig.handle.abort.assert_called_once()
    rig.handle.succeed.assert_not_called()
    assert rig.handle.publish_feedback.call_count == 1


def test_pending_admission_is_bounded_and_released_only_at_terminal_result(rig):
    assert rig.api._accept(rig.handle.request) == GoalResponse.ACCEPT
    other = type(rig.handle.request)()
    other.client_id = "other"
    other.context_id = "other-context"
    other.intent = rig.handle.request.intent
    assert rig.api._accept(other) == GoalResponse.REJECT
    coroutine = rig.api._execute(rig.handle)
    coroutine.send(None)
    rig.status(CommandControllerState.STOP_UNCONFIRMED)
    assert rig.api._accept(other) == GoalResponse.REJECT
    rig.status(CommandControllerState.STOP_CONFIRMED)
    finish(coroutine)
    assert rig.api._accept(other) == GoalResponse.ACCEPT


def test_cancel_before_execute_never_dispatches_work(rig):
    rig.handle.is_cancel_requested = True
    result = finish(rig.api._execute(rig.handle))
    assert result.state.state == rig.state_type.STATE_CANCELLED
    rig.submit.assert_not_called()
    rig.handle.canceled.assert_called_once()
    rig.handle.abort.assert_not_called()


def test_shutdown_wakes_yielded_action_once_and_late_status_cannot_claim_success(rig):
    coroutine = rig.api._execute(rig.handle)
    waiting = coroutine.send(None)
    rig.api.request_shutdown()
    rig.api.request_shutdown()
    assert waiting.done()
    rig.status(CommandControllerState.SUCCEEDED)
    result = finish(coroutine)
    assert result.state.state == rig.state_type.STATE_FAILED
    assert "stop unconfirmed" in result.state.detail
    rig.handle.abort.assert_called_once()
    rig.handle.succeed.assert_not_called()
    rig.cancel.assert_called_once()
    assert rig.api._accept(rig.handle.request) == GoalResponse.REJECT


@pytest.mark.parametrize("rig", [NavigationSetupApi], indirect=True)
def test_navigation_synchronous_terminal_reply_yields_exactly_one_result_without_retained_cache(rig):

    def submit(**_kwargs):
        rig.status(CommandControllerState.SUCCEEDED)
        rig.status(CommandControllerState.RUNNING)
        return rig.operation

    rig.submit.side_effect = submit
    result = finish(rig.api._execute(rig.handle))
    assert result.state.state == NavigationSetupState.STATE_SUCCEEDED
    rig.handle.succeed.assert_called_once()
    rig.status(CommandControllerState.SUCCEEDED)
    assert not rig.api._early_states


def test_cancel_intent_survives_preparation_before_ros_cancel_state_changes(rig):
    def cancel_during_preparation(*_args, **_kwargs):
        assert not rig.handle.is_cancel_requested
        rig.api._accept_cancel(rig.handle)
        assert not rig.api._executions
        return rig.operation

    if isinstance(rig.api, NavigationSetupApi):
        rig.submit.side_effect = cancel_during_preparation
    else:
        rig.api.coordinator.prepare_motion.side_effect = cancel_during_preparation
        rig.cancel.side_effect = lambda *_args: rig.status(CommandControllerState.CANCELLED)
        result = finish(rig.api._execute(rig.handle))
        rig.submit.assert_not_called()
        rig.cancel.assert_called_once()
        assert result.state.state == ProbeSetupState.STATE_CANCELLED
        # ROS has not yet transitioned to CANCELING, so abort is the valid terminal transition.
        rig.handle.abort.assert_called_once()
        return
    coroutine = rig.api._execute(rig.handle)
    waiting = coroutine.send(None)
    rig.cancel.assert_called_once()
    assert not waiting.done()
    rig.handle.is_cancel_requested = True
    rig.status(CommandControllerState.STOP_CONFIRMED)
    assert finish(coroutine).state.state == rig.state_type.STATE_STOP_CONFIRMED
    rig.handle.canceled.assert_called_once()


@pytest.mark.parametrize("rig", [ProbeSetupMotionApi], indirect=True)
def test_probe_cancel_during_submission_is_sent_after_command_is_registered(rig):
    def submit(_operation):
        execution = rig.api._executions["request"]
        assert not execution.submitted
        rig.api._accept_cancel(rig.handle)
        rig.cancel.assert_not_called()

    rig.submit.side_effect = submit
    coroutine = rig.api._execute(rig.handle)
    waiting = coroutine.send(None)
    rig.cancel.assert_called_once()
    assert not waiting.done()
    rig.handle.is_cancel_requested = True
    rig.status(CommandControllerState.STOP_CONFIRMED)
    finish(coroutine)
    rig.handle.canceled.assert_called_once()


@pytest.mark.parametrize("rig", [ProbeSetupMotionApi], indirect=True)
def test_shutdown_during_submission_still_cancels_newly_submitted_motion(rig):
    def submit(_operation):
        rig.api.request_shutdown()
        rig.cancel.assert_not_called()

    rig.submit.side_effect = submit
    result = finish(rig.api._execute(rig.handle))
    rig.cancel.assert_called_once()
    assert result.state.state == ProbeSetupState.STATE_FAILED
    assert "stop unconfirmed" in result.state.detail
    rig.handle.abort.assert_called_once()


@pytest.mark.parametrize("rig", [NavigationSetupApi], indirect=True)
def test_navigation_close_releases_disconnected_accepted_goal(rig):
    assert rig.api._accept(rig.handle.request) == GoalResponse.ACCEPT
    assert not rig.api._admission.shutdown_status()[0]
    response = SimpleNamespace()
    rig.api._close(rig.handle.request, response)
    assert response.closed
    assert rig.api._admission.shutdown_status()[0]


@pytest.mark.parametrize("failure", ["state", "feedback"])
@pytest.mark.parametrize("during_submission", [False, True])
def test_terminal_result_survives_status_publication_failure(rig, failure, during_submission):
    if failure == "state":
        rig.api.state_publisher.publish.side_effect = RuntimeError("publisher unavailable")
    else:
        rig.handle.publish_feedback.side_effect = RuntimeError("goal client disconnected")

    if during_submission:
        def submit(*_args, **_kwargs):
            rig.status(CommandControllerState.SUCCEEDED)
            return rig.operation
        rig.submit.side_effect = submit
        result = finish(rig.api._execute(rig.handle))
    else:
        coroutine = rig.api._execute(rig.handle)
        waiting = coroutine.send(None)
        rig.status(CommandControllerState.SUCCEEDED)
        assert waiting.done()
        result = finish(coroutine)
    assert result.state.state == rig.state_type.STATE_SUCCEEDED
    rig.handle.succeed.assert_called_once()
    rig.handle.abort.assert_not_called()
    assert not rig.api._executions
    assert rig.api._admission.shutdown_status()[0]
    assert rig.api._shutdown_callbacks.status()[0]


def test_shutdown_after_synchronous_terminal_reply_preserves_owner_result(rig):
    def submit(*_args, **_kwargs):
        rig.status(CommandControllerState.SUCCEEDED)
        rig.api.request_shutdown()
        return rig.operation

    rig.submit.side_effect = submit
    result = finish(rig.api._execute(rig.handle))
    assert result.state.state == rig.state_type.STATE_SUCCEEDED
    rig.handle.succeed.assert_called_once()
    rig.handle.abort.assert_not_called()
    rig.cancel.assert_not_called()


def test_shutdown_keeps_accepted_goal_pending_until_its_callback_reports_terminal(rig):
    assert rig.api._accept(rig.handle.request) == GoalResponse.ACCEPT
    rig.api.request_shutdown()
    rig.api.request_shutdown()
    assert rig.api._shutdown_callbacks.status()[0]
    confirmed, detail = rig.api._admission.shutdown_status()
    assert not confirmed
    assert '1 queued' in detail
    result = finish(rig.api._execute(rig.handle))
    assert result.state.state == rig.state_type.STATE_FAILED
    assert 'shutting down' in result.state.detail
    rig.submit.assert_not_called()
    rig.handle.abort.assert_called_once()
    assert rig.api._admission.shutdown_status()[0]


def test_delayed_callback_cannot_recreate_admission_after_context_cancellation(rig):
    rig.api._admission.cancel_context("context")
    assert not rig.api._shutdown.is_set()
    assert rig.api._admission.shutdown_status()[0]
    # Ending refinement can leave the context open. A revoked request still
    # cannot dispatch, even though resolving that context would succeed.
    result = finish(rig.api._execute(rig.handle))
    assert result.state.state == rig.state_type.STATE_FAILED
    rig.submit.assert_not_called()
    rig.api.coordinator.prepare_motion.assert_not_called()
    rig.handle.abort.assert_called_once()
    assert rig.api._admission.shutdown_status()[0]
