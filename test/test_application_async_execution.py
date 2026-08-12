import inspect
from types import SimpleNamespace

from rclpy.clock import ClockType
from rclpy.task import Future

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
    _OperationExecution,
)
from fault_detector_spot.application.behaviour_tree.runner import (
    start_tree_ticking,
)


class FakeNode:
    def __init__(self):
        self.period = None
        self.callback = None
        self.clock = None
        self.timer = object()

    def create_timer(self, period, callback, clock=None):
        self.period = period
        self.callback = callback
        self.clock = clock
        return self.timer


def test_operation_execute_callback_is_non_blocking_coroutine():
    assert inspect.iscoroutinefunction(ApplicationApiNode._execute_operation)


def test_operation_completion_uses_rclpy_future():
    execution = _OperationExecution(
        goal_handle=object(),
        operation=SimpleNamespace(),
    )

    assert isinstance(execution.finished, Future)


def test_behavior_tree_tick_uses_steady_clock():
    node = FakeNode()
    tree = SimpleNamespace(
        node=node,
        tick=lambda: None,
        timer=None,
    )

    timer = start_tree_ticking(tree, period_ms=50.0)

    assert timer is node.timer
    assert tree.timer is node.timer
    assert node.period == 0.05
    assert node.clock.clock_type is ClockType.STEADY_TIME

