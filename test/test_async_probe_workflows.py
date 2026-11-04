"""Bounded probe workflow execution preserves persistence and recovery."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.probe_refinement_finalization_coordinator import (
    FinalizationPhase,
    ProbeRefinementFinalizationCoordinator,
    ProbeRefinementFinalizationSpec,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind


def _runner():
    context = SimpleNamespace(context_id="context", client_id="client")
    refinement = SimpleNamespace(saved=False, recovery_required=False, recovery_message="")
    snapshot = SimpleNamespace(refinement=refinement)
    coordinator = Mock()
    coordinator.context.return_value = context
    coordinator.begin_finalization.return_value = snapshot
    coordinator.approve_probe_geometry_for_finalization.return_value = snapshot
    coordinator.snapshot.return_value = snapshot

    def save(*args):
        refinement.saved = True
        return snapshot

    def fail(*args):
        refinement.recovery_required = True
        refinement.recovery_message = args[-1]
        return snapshot

    coordinator.save_probe_point_for_finalization.side_effect = save
    coordinator.fail_finalization.side_effect = fail
    return ProbeRefinementFinalizationCoordinator(coordinator), context, snapshot


@pytest.mark.parametrize("failure", [
    RuntimeError("retraction submission failed"),
    SimpleNamespace(state=CommandControllerState.FAILED, detail="retraction failed"),
    SimpleNamespace(state=CommandControllerState.STOP_CONFIRMED, detail="cancelled"),
])
def test_saved_probe_survives_retraction_failure_with_recovery(failure):
    runner, context, snapshot = _runner()
    runner._execute_retraction = Mock(side_effect=[failure])
    progress = Mock()

    result, saved, phase = runner.run(
        context, "request", ProbeRefinementFinalizationSpec(save_requested=True),
        cancel_requested=lambda: False, state_changed=progress,
    )

    assert result is snapshot
    assert saved and snapshot.refinement.saved
    assert phase is FinalizationPhase.RECOVERY_REQUIRED
    assert snapshot.refinement.recovery_required
    runner.coordinator.complete_finalization.assert_not_called()
    assert progress.call_args.args == (phase, True, snapshot)


def test_cancellation_after_save_preserves_saved_data_without_starting_retraction():
    runner, context, snapshot = _runner()
    cancelled = Event()
    runner._execute_retraction = Mock()

    def progress(phase, saved, current):
        if phase is FinalizationPhase.RETRACTING_TO_ALIGNED:
            assert saved
            cancelled.set()

    result, saved, phase = runner.run(
        context, "request", ProbeRefinementFinalizationSpec(save_requested=True),
        cancel_requested=cancelled.is_set, state_changed=progress,
    )

    assert result is snapshot and saved
    assert phase is FinalizationPhase.RECOVERY_REQUIRED
    assert snapshot.refinement.saved and snapshot.refinement.recovery_required
    runner._execute_retraction.assert_not_called()


def test_missing_retraction_result_cancels_once_and_releases_worker_with_recovery():
    coordinator = Mock()
    coordinator.context.return_value = SimpleNamespace(context_id="context", client_id="client")
    coordinator.prepare_motion.return_value = SimpleNamespace(request_id="motion")
    runner = ProbeRefinementFinalizationCoordinator(
        coordinator, poll_sec=0.001, motion_timeout_sec=0.005,
        cancellation_timeout_sec=0.005,
    )

    with ThreadPoolExecutor(max_workers=1) as worker:
        finished = worker.submit(
            runner._execute_retraction, "context", "client", "request",
            ProbeMotionKind.MOVE_SAFE_APPROACH, lambda: False, 0.01, 0.1,
        )
        with pytest.raises(RuntimeError, match="stop unconfirmed"):
            finished.result(timeout=0.5)

    coordinator.cancel_motion.assert_called_once_with(
        coordinator.context.return_value, "motion",
    )
    assert not runner._waiters


def test_shutdown_sends_owned_retraction_cancellation_before_releasing_waiter():
    coordinator = Mock()
    coordinator.context.return_value = SimpleNamespace(context_id="context", client_id="client")
    coordinator.prepare_motion.return_value = SimpleNamespace(request_id="motion")
    entered = Event()
    coordinator.submit_motion.side_effect = lambda operation: entered.set()
    runner = ProbeRefinementFinalizationCoordinator(coordinator, poll_sec=0.001)
    with ThreadPoolExecutor(max_workers=1) as worker:
        finished = worker.submit(
            runner._execute_retraction, "context", "client", "request",
            ProbeMotionKind.MOVE_SAFE_APPROACH, lambda: False, 0.01, 0.1,
        )
        assert entered.wait(0.5)
        runner.request_shutdown()
        with pytest.raises(RuntimeError, match="shutdown"):
            finished.result(timeout=0.5)
    coordinator.cancel_motion.assert_called_once_with(
        coordinator.context.return_value, "motion",
    )
    assert not runner._waiters


def _api_pair(monkeypatch, worker_limit=1):
    from builtin_interfaces.msg import Time
    from fault_detector_msgs.msg import ProbeSetupState
    from fault_detector_spot.application.api import (
        probe_reference_capture_api as capture_module,
        probe_refinement_finalization_api as finalization_module,
    )
    from fault_detector_spot.application.api.async_operation import OperationAdmission
    monkeypatch.setattr(capture_module, "ActionServer", Mock())
    monkeypatch.setattr(finalization_module, "ActionServer", Mock())
    worker = OperationAdmission(worker_limit)
    coordinator = Mock()
    coordinator.is_context_closing.return_value = False
    coordinator.is_current.return_value = True
    coordinator.snapshot.return_value = SimpleNamespace(refinement=None)
    coordinator.fail_finalization.side_effect = RuntimeError("No active finalization")
    node = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    adapter = Mock()

    def message(snapshot, operation, state, detail, **kwargs):
        value = ProbeSetupState(state=state, detail=detail)
        refinement = getattr(snapshot, "refinement", None)
        if refinement is not None:
            value.refinement_recovery_required = refinement.recovery_required
            value.refinement_recovery_message = refinement.recovery_message
        return value

    adapter.message.side_effect = message
    capture = capture_module.ProbeReferenceCaptureApi(
        node, coordinator, Mock(), Mock(), adapter, admission=worker,
    )
    finalization = finalization_module.ProbeRefinementFinalizationApi(
        node, coordinator, Mock(), adapter, admission=worker,
    )
    return capture, finalization, worker


def _capture_goal():
    from fault_detector_msgs.action import CaptureProbeReferenceViews
    goal = CaptureProbeReferenceViews.Goal()
    goal.client_id = "client"
    goal.context_id = "0939a2c7-8040-4507-bdbd-6b68b7a7d957"
    return goal


def _finalization_goal():
    from fault_detector_msgs.action import FinalizeProbeRefinement
    goal = FinalizeProbeRefinement.Goal()
    goal.client_id = "client"
    goal.context_id = "0939a2c7-8040-4507-bdbd-6b68b7a7d957"
    goal.mode = FinalizeProbeRefinement.Goal.MODE_SAVE_AND_RETRACT
    return goal


def _finish_callback(callback, pending):
    from time import monotonic, sleep
    deadline = monotonic() + 1.0
    while monotonic() < deadline:
        if pending.done():
            try:
                pending = callback.send(None)
            except StopIteration as result:
                return result.value
        sleep(0.001)
    callback.close()
    pytest.fail("Workflow callback did not complete")


def test_probe_workflows_share_bounded_admission_before_execution(monkeypatch):
    from rclpy.action import GoalResponse
    capture, finalization, worker = _api_pair(monkeypatch, worker_limit=2)
    capture_goal = _capture_goal()
    finalization_goal = _finalization_goal()

    assert capture._accept(capture_goal) is GoalResponse.ACCEPT
    # The physical probe lane stays exclusive even with two worker slots.
    assert finalization._accept(finalization_goal) is GoalResponse.REJECT
    capture.request_shutdown()
    assert not worker.shutdown_status()[0]
    capture_handle = Mock(request=capture_goal, is_cancel_requested=False)
    _execute_callback(capture._execute(capture_handle))
    capture_handle.abort.assert_called_once()
    capture.capture_coordinator.run.assert_not_called()
    assert worker.shutdown_status()[0]
    assert finalization._accept(finalization_goal) is GoalResponse.ACCEPT
    finalization.request_shutdown()
    assert not worker.shutdown_status()[0]
    finalization_handle = Mock(request=finalization_goal, is_cancel_requested=False)
    _execute_callback(finalization._execute(finalization_handle))
    finalization_handle.abort.assert_called_once()
    finalization.coordinator.begin_finalization.assert_not_called()
    assert worker.shutdown_status()[0]
    worker.close()


@pytest.mark.parametrize("cancel_kind", ["cancel", "disconnect", "shutdown"])
def test_capture_yields_while_worker_waits_and_cancellation_is_responsive(monkeypatch, cancel_kind):
    from rclpy.action import GoalResponse
    from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
        ReferenceCaptureCancelled,
    )
    capture, _, worker = _api_pair(monkeypatch)
    entered = Event()
    context_closing = Event()
    capture.probe_setup_coordinator.is_context_closing.side_effect = (
        lambda context: context_closing.is_set()
    )

    def run(context, request_id, spec, cancel_requested, state_changed):
        entered.set()
        while not cancel_requested():
            Event().wait(0.001)
        raise ReferenceCaptureCancelled("capture cancelled")

    capture.capture_coordinator.run.side_effect = run
    goal = Mock(request=_capture_goal(), is_cancel_requested=False)
    assert capture._accept(goal.request) is GoalResponse.ACCEPT
    callback = capture._execute(goal)
    pending = callback.send(None)
    assert entered.wait(0.5)
    assert not capture._shutdown_callbacks.status()[0]
    assert not worker.shutdown_status()[0]
    # An executor is free to dispatch cancellation/status while capture blocks
    # only its dedicated worker. A second workflow is rejected at admission.
    assert capture._accept(_capture_goal()) is GoalResponse.REJECT
    if cancel_kind == "cancel":
        goal.is_cancel_requested = True
        capture._accept_cancel(goal)
    elif cancel_kind == "disconnect":
        context_closing.set()
    else:
        capture.request_shutdown()
    result = _finish_callback(callback, pending)
    assert result.state.detail == "capture cancelled"
    assert goal.succeed.call_count == 0
    assert goal.canceled.call_count == int(cancel_kind == "cancel")
    assert goal.abort.call_count == int(cancel_kind != "cancel")
    assert capture._shutdown_callbacks.status()[0]
    assert worker.shutdown_status()[0]
    worker.close()


@pytest.mark.parametrize("updates_fail", [False, True])
def test_finalization_worker_yields_to_status_callbacks_and_keeps_saved_recovery(monkeypatch, updates_fail):
    from rclpy.action import GoalResponse
    from fault_detector_msgs.msg import ProbeSetupState
    capture, finalization, worker = _api_pair(monkeypatch)
    runner, context, snapshot = _runner()
    runner.coordinator.is_context_closing.return_value = False
    finalization.coordinator = runner.coordinator
    finalization.runner = runner
    runner.coordinator.prepare_motion.return_value = SimpleNamespace(request_id="motion")
    submitted = Event()
    runner.coordinator.submit_motion.side_effect = lambda operation: submitted.set()
    goal = Mock(request=_finalization_goal(), is_cancel_requested=False)
    if updates_fail:
        finalization.state_publisher.publish.side_effect = RuntimeError("publisher disconnected")
        goal.publish_feedback.side_effect = RuntimeError("feedback disconnected")
    assert finalization._accept(goal.request) is GoalResponse.ACCEPT
    callback = finalization._execute(goal)
    pending = callback.send(None)
    assert submitted.wait(0.5)
    assert not finalization._shutdown_callbacks.status()[0]
    assert capture._accept(_capture_goal()) is GoalResponse.REJECT
    # This is the status callback that a blocked ROS execute callback formerly
    # prevented from running when the executor worker limit was exhausted.
    runner._receive_motion_status(SimpleNamespace(
        request_id="motion", state=CommandControllerState.FAILED,
        detail="retraction failed",
    ))
    result = _finish_callback(callback, pending)
    assert result.probe_point_saved
    assert result.state.refinement_recovery_required
    assert result.state.state == ProbeSetupState.STATE_FAILED
    assert snapshot.refinement.saved
    goal.abort.assert_called_once()
    goal.succeed.assert_not_called()
    goal.canceled.assert_not_called()
    assert finalization._shutdown_callbacks.status()[0]
    worker.close()


def test_save_completion_error_retains_committed_point_and_recovery():
    runner, context, snapshot = _runner()

    def save(*args):
        snapshot.refinement.saved = True
        raise RuntimeError("post-save snapshot failed")

    runner.coordinator.save_probe_point_for_finalization.side_effect = save
    result, saved, phase = runner.run(
        context, "request", ProbeRefinementFinalizationSpec(save_requested=True),
        cancel_requested=lambda: False,
    )
    assert result is snapshot and saved
    assert phase is FinalizationPhase.RECOVERY_REQUIRED
    assert snapshot.refinement.saved and snapshot.refinement.recovery_required


def test_first_terminal_retraction_status_cannot_be_overwritten():
    from fault_detector_spot.application.coordinators.probe_refinement_finalization_coordinator import (
        _MotionWaiter,
    )
    runner, _, _ = _runner()
    waiter = _MotionWaiter()
    runner._waiters["motion"] = waiter
    terminal = SimpleNamespace(request_id="motion", state=CommandControllerState.FAILED)
    runner._receive_motion_status(terminal)
    runner._receive_motion_status(SimpleNamespace(
        request_id="motion", state=CommandControllerState.SUCCEEDED,
    ))
    assert waiter.event.is_set()
    assert waiter.status is terminal


def test_saved_finalization_fallback_result_explicitly_requires_recovery(monkeypatch):
    from builtin_interfaces.msg import Time
    capture, finalization, worker = _api_pair(monkeypatch)
    finalization.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    goal = Mock(request=_finalization_goal(), is_cancel_requested=False)
    finalization.state_publisher.publish.side_effect = RuntimeError("publisher disconnected")
    result = finalization._abort(
        goal, goal.request, "request", True, None, "context disappeared after save",
    )
    assert result.probe_point_saved
    assert result.state.refinement_recovery_required
    assert result.state.refinement_recovery_message == "context disappeared after save"
    goal.abort.assert_called_once()
    worker.close()


def _execute_callback(callback):
    try:
        pending = callback.send(None)
    except StopIteration as result:
        return result.value
    return _finish_callback(callback, pending)


def test_successful_saved_finalization_keeps_terminal_result_when_updates_fail(monkeypatch):
    from fault_detector_msgs.msg import ProbeSetupState
    _, finalization, worker = _api_pair(monkeypatch)
    runner, _, snapshot = _runner()
    runner.coordinator.is_context_closing.return_value = False
    runner.coordinator.complete_finalization.return_value = snapshot
    runner._execute_retraction = Mock(return_value=SimpleNamespace(
        state=CommandControllerState.SUCCEEDED,
    ))
    finalization.coordinator = runner.coordinator
    finalization.runner = runner
    finalization.state_publisher.publish.side_effect = RuntimeError("publisher disconnected")
    goal = Mock(request=_finalization_goal(), is_cancel_requested=False)
    goal.publish_feedback.side_effect = RuntimeError("feedback disconnected")

    from rclpy.action import GoalResponse
    assert finalization._accept(goal.request) is GoalResponse.ACCEPT
    result = _execute_callback(finalization._execute(goal))

    assert result.probe_point_saved
    assert result.state.state == ProbeSetupState.STATE_SUCCEEDED
    goal.succeed.assert_called_once()
    goal.abort.assert_not_called()
    goal.canceled.assert_not_called()
    worker.close()


def test_capture_terminal_result_survives_publisher_and_feedback_failure(monkeypatch):
    from fault_detector_msgs.msg import ProbeSetupState
    from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
        ReferenceCapturePhase,
    )
    capture, _, worker = _api_pair(monkeypatch)
    snapshot = SimpleNamespace(refinement=None)
    capture.state_publisher.publish.side_effect = RuntimeError("publisher disconnected")
    goal = Mock(request=_capture_goal(), is_cancel_requested=False)
    goal.publish_feedback.side_effect = RuntimeError("feedback disconnected")

    def capture_result(context, request_id, spec, cancel_requested, state_changed):
        state_changed(ReferenceCapturePhase.COMPLETE, 1, snapshot)
        return snapshot

    capture.capture_coordinator.run.side_effect = capture_result
    from rclpy.action import GoalResponse
    assert capture._accept(goal.request) is GoalResponse.ACCEPT
    result = _execute_callback(capture._execute(goal))

    assert result.state.state == ProbeSetupState.STATE_SUCCEEDED
    goal.succeed.assert_called_once()
    goal.abort.assert_not_called()
    goal.canceled.assert_not_called()
    worker.close()


@pytest.mark.parametrize("shutdown", [False, True])
def test_finalization_cancellation_during_preparation_prevents_submission(shutdown):
    runner, context, _ = _runner()
    cancelled = Event()
    operation = SimpleNamespace(request_id="motion")

    def prepare(*args, **kwargs):
        if shutdown:
            runner.request_shutdown()
        else:
            cancelled.set()
        return operation

    runner.coordinator.prepare_motion.side_effect = prepare
    with pytest.raises(RuntimeError, match="cancelled during preparation"):
        runner._execute_retraction(
            context.context_id, context.client_id, "request",
            ProbeMotionKind.MOVE_SAFE_APPROACH, cancelled.is_set, 0.01, 0.1,
        )
    runner.coordinator.cancel_motion.assert_called_once_with(context, "motion")
    runner.coordinator.submit_motion.assert_not_called()
    assert not runner._waiters


@pytest.mark.parametrize("api_name", ["capture", "finalization"])
def test_context_cancelled_pending_goal_cannot_recreate_admission(monkeypatch, api_name):
    from rclpy.action import GoalResponse
    from fault_detector_msgs.msg import ProbeSetupState
    capture, finalization, worker = _api_pair(monkeypatch)
    api = capture if api_name == "capture" else finalization
    request = _capture_goal() if api_name == "capture" else _finalization_goal()
    goal = Mock(request=request, is_cancel_requested=False)
    finalization.runner.run = Mock()
    assert api._accept(request) is GoalResponse.ACCEPT
    worker.cancel_context(request.context_id)
    assert worker.shutdown_status()[0]
    # End-refinement cancels pending work while its setup context still exists.
    # A late ROS execute callback must consume the old reservation, never create
    # a fresh one merely because context lookup and validation still succeed.
    assert not finalization.coordinator.is_context_closing(object())
    result = _execute_callback(api._execute(goal))
    assert result.state.state == ProbeSetupState.STATE_FAILED
    goal.abort.assert_called_once()
    goal.succeed.assert_not_called()
    goal.canceled.assert_not_called()
    capture.capture_coordinator.run.assert_not_called()
    finalization.runner.run.assert_not_called()
    assert worker.shutdown_status()[0]
    worker.close()
