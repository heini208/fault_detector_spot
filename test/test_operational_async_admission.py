"""Operational admission and cancellation are independent of workflow workers."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState
from rclpy.action import GoalResponse

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity
from fault_detector_spot.application.controllers.command_controller import CommandControllerState


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
