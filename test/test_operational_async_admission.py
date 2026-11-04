"""Operational admission and cancellation are independent of workflow workers."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent
from rclpy.action import GoalResponse

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity
from fault_detector_spot.application.controllers.application_controller import ApplicationController
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)


@pytest.fixture
def rig():
    node = ApplicationApiNode.__new__(ApplicationApiNode)
    node._lock = RLock()
    node._shutdown = Event()
    node._operation_admission = OperationAdmission(1)
    node._shutdown_callbacks = ShutdownCallbackActivity()
    node._executions = {}
    node._state_publisher = Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    node.get_logger = lambda: Mock()
    node.application_controller = Mock()
    operation = SimpleNamespace(request_id='request', intent=0,
                                request=SimpleNamespace(client_id='client', context_id='context'))
    node.application_controller.prepare_operation.return_value = operation
    handle = Mock(request=ExecuteOperation.Goal(client_id='client', context_id='context'),
                  is_cancel_requested=False)
    return node, handle, operation


def finish(coroutine):
    with pytest.raises(StopIteration) as completed:
        coroutine.send(None)
    return completed.value.value


@pytest.fixture
def command_rig(rig):
    node, _, _ = rig
    dispatched = []
    node.command_controller = CommandController(dispatch_request=dispatched.append)
    node.application_controller = ApplicationController(node.command_controller)
    node.application_controller.add_status_listener(node._handle_application_status)
    return node, dispatched


def start_operation(node, intent):
    handle = Mock(request=ExecuteOperation.Goal(
        client_id='client', context_id='context',
        intent=OperationalIntent(intent=intent),
    ), is_cancel_requested=False)
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT
    callback = node._execute_operation(handle)
    waiting = callback.send(None)
    assert not waiting.done()
    return handle, callback, waiting


def report_execution(node, request_id, state, *, sequence=1, buffered_count=0):
    assert node.command_controller.handle_execution_status(CommandExecutionStatus(
        request_id=request_id, state=state, status_sequence=sequence,
        execution_instance_id='test-owner', buffered_command_count=buffered_count,
        execution_idle=state is CommandControllerState.SUCCEEDED,
    ))


def test_operations_buffer_while_moving_and_dispatch_in_order(command_rig):
    node, dispatched = command_rig
    first = start_operation(node, OperationalIntent.INTENT_STAND_UP)
    first_id = node.command_controller.active_request_id
    report_execution(node, first_id, CommandControllerState.RUNNING)

    second = start_operation(node, OperationalIntent.INTENT_READY_ARM)
    third = start_operation(node, OperationalIntent.INTENT_STOW_ARM)
    second_id, third_id = node.command_controller.queued_request_ids
    assert [request.request_id for request in dispatched] == [first_id]
    assert third[0].publish_feedback.call_args.args[0].state.state == (
        ApplicationCommandState.STATE_QUEUED
    )
    assert third[0].publish_feedback.call_args.args[0].state.buffered_command_count == 2

    report_execution(node, first_id, CommandControllerState.SUCCEEDED,
                     sequence=2, buffered_count=1)
    assert not first[2].done()
    assert [request.request_id for request in dispatched] == [first_id]

    report_execution(node, first_id, CommandControllerState.SUCCEEDED, sequence=3)
    assert finish(first[1]).state.state == ApplicationCommandState.STATE_SUCCEEDED
    assert [request.request_id for request in dispatched] == [first_id, second_id]
    assert node.command_controller.queued_request_ids == (third_id,)
    assert not second[2].done() and not third[2].done()

    report_execution(node, second_id, CommandControllerState.SUCCEEDED)
    finish(second[1])
    assert [request.request_id for request in dispatched] == [first_id, second_id, third_id]
    report_execution(node, third_id, CommandControllerState.SUCCEEDED)
    finish(third[1])
    assert node._operation_admission.shutdown_status()[0]
    assert not node._executions
    for handle, _, _ in (first, second, third):
        handle.succeed.assert_called_once()


def test_cancelling_buffered_operation_preserves_active_and_other_queued_operations(command_rig):
    node, dispatched = command_rig
    first = start_operation(node, OperationalIntent.INTENT_STAND_UP)
    first_id = node.command_controller.active_request_id
    report_execution(node, first_id, CommandControllerState.RUNNING)
    second = start_operation(node, OperationalIntent.INTENT_READY_ARM)
    third = start_operation(node, OperationalIntent.INTENT_STOW_ARM)
    second_id, third_id = node.command_controller.queued_request_ids

    second[0].is_cancel_requested = True
    node._accept_cancellation(second[0])
    assert finish(second[1]).state.state == ApplicationCommandState.STATE_CANCELLED
    second[0].canceled.assert_called_once()
    assert node.command_controller.active_request_id == first_id
    assert node.command_controller.queued_request_ids == (third_id,)
    assert [request.request_id for request in dispatched] == [first_id]
    assert not node.command_controller.software_stop_status.latched
    assert not first[2].done() and not third[2].done()
    assert second_id not in node._executions
    assert not node._operation_admission.shutdown_status()[0]
    assert not node._operation_admission.cancel_requested(id(first[0].request))
    assert not node._operation_admission.cancel_requested(id(third[0].request))
    assert not node._operation_admission.reserve('setup', context_id='setup', lane='physical')

    report_execution(node, first_id, CommandControllerState.SUCCEEDED, sequence=2)
    finish(first[1])
    assert [request.request_id for request in dispatched] == [first_id, third_id]
    report_execution(node, third_id, CommandControllerState.SUCCEEDED)
    finish(third[1])
    assert node._operation_admission.shutdown_status()[0]


def test_cancellation_during_preparation_survives_ros_cancel_transition(rig):
    node, handle, operation = rig
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT

    def prepare(**kwargs):
        node._accept_cancellation(handle)
        return operation

    node.application_controller.prepare_operation.side_effect = prepare
    callback = node._execute_operation(handle)
    waiting = callback.send(None)
    assert not waiting.done()
    node.application_controller.cancel.assert_called_once_with('client', 'request')
    node._accept_cancellation(handle)
    node.application_controller.cancel.assert_called_once()
    handle.is_cancel_requested = True
    waiting.set_result(ApplicationCommandState(state=ApplicationCommandState.STATE_STOP_CONFIRMED))
    finish(callback)
    handle.canceled.assert_called_once()


def test_workflow_occupancy_rejects_motion_but_does_not_block_emergency(rig):
    node, handle, _ = rig
    worker = node._operation_admission
    assert worker.reserve('capture', context_id='capture-context', lane='probe')
    assert node._accept_operation(handle.request) == GoalResponse.REJECT
    response = node._emergency_stop(SimpleNamespace(client_id='client'), SimpleNamespace())
    assert response.accepted
    node.application_controller.emergency_stop.assert_called_once_with('client')
    worker.release('capture')
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT
    node._operation_admission.release_pending(id(handle.request))
    worker.close()


def test_terminal_status_finishes_even_when_state_publication_fails(rig):
    node, handle, operation = rig
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT
    callback = node._execute_operation(handle)
    waiting = callback.send(None)
    state = ApplicationCommandState(state=ApplicationCommandState.STATE_SUCCEEDED)
    node._state_message = Mock(return_value=state)
    node._state_publisher.publish.side_effect = RuntimeError('publisher closed')
    node._handle_application_status(SimpleNamespace(
        operation=operation, state=CommandControllerState.SUCCEEDED,
    ))
    assert waiting.done()
    assert finish(callback).state.state == state.state
    handle.succeed.assert_called_once()
    assert not node._executions
    assert node._operation_admission.shutdown_status()[0]


def test_cancellation_send_error_retains_ownership_and_allows_retry(rig):
    node, handle, _ = rig
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT
    callback = node._execute_operation(handle)
    waiting = callback.send(None)
    node.application_controller.cancel.side_effect = [RuntimeError('transport unavailable'), 'cancel']
    node._accept_cancellation(handle)
    assert not waiting.done()
    assert not node._operation_admission.shutdown_status()[0]
    node._accept_cancellation(handle)
    assert node.application_controller.cancel.call_count == 2
    waiting.set_result(ApplicationCommandState(state=ApplicationCommandState.STATE_STOP_CONFIRMED))
    finish(callback)


def test_context_cancelled_accepted_goal_cannot_recreate_admission(rig):
    node, handle, _ = rig
    assert node._accept_operation(handle.request) == GoalResponse.ACCEPT
    node._operation_admission.cancel_context('context')
    callback = node._execute_operation(handle)
    assert finish(callback).state.state == ApplicationCommandState.STATE_FAILED
    node.application_controller.prepare_operation.assert_not_called()
    node.application_controller.submit.assert_not_called()
    handle.abort.assert_called_once()
