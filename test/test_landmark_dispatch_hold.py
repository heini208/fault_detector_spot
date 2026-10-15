"""Landmark pose resets briefly hold normal dispatch without blocking cancel."""

from types import SimpleNamespace

import py_trees
import pytest
from py_trees.common import Access, Status

from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import (
    CommandManager,
)
from fault_detector_spot.application.commanding.command_ids import CommandID


def command(command_id):
    return SimpleNamespace(command_id=command_id, request_id="", stamp=None)


@pytest.fixture
def rig():
    py_trees.blackboard.Blackboard.clear()
    steady = [100.0]
    ros = [42_000_000_000]
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=ros[0]),
    ))
    manager = CommandManager(monotonic_clock=lambda: steady[0])
    manager.setup(node=node)
    publisher = py_trees.blackboard.Client(name="landmark reset hold test")
    publisher.register_key("localization_reset_hold_until", access=Access.WRITE)
    manager.blackboard.command_tree_status = Status.SUCCESS
    previous = command(CommandID.SIT_DOWN)
    manager.blackboard.last_command = previous
    pending = [command(CommandID.STAND_UP), command(CommandID.READY_ARM)]
    manager.blackboard.command_buffer = pending.copy()

    yield SimpleNamespace(
        manager=manager, publisher=publisher, steady=steady, ros=ros,
        pending=pending, previous=previous,
    )

    manager.blackboard.unregister_all_keys()
    publisher.unregister_all_keys()
    py_trees.blackboard.Blackboard.clear()


def test_hold_preserves_queue_previous_command_and_status(rig):
    rig.publisher.localization_reset_hold_until = 102.0
    for stamp in (100.0, 100.5, 101.999):
        rig.steady[0] = stamp
        assert rig.manager.update() is Status.SUCCESS
        assert rig.manager.blackboard.command_buffer == rig.pending
        assert rig.manager.blackboard.last_command is rig.previous
        assert rig.manager.blackboard.command_tree_status is Status.SUCCESS
        assert all(item.stamp is None for item in rig.pending)
        assert "reset to settle" in rig.manager.feedback_message


def test_hold_expires_on_steady_deadline_even_with_paused_ros_clock(rig):
    rig.publisher.localization_reset_hold_until = 102.0
    rig.manager.update()
    rig.steady[0] = 102.0

    assert rig.manager.update() is Status.SUCCESS
    assert rig.manager.blackboard.last_command is rig.pending[0]
    assert rig.manager.blackboard.command_buffer == rig.pending[1:]
    assert rig.manager.blackboard.command_tree_status is None
    assert rig.pending[0].stamp.sec == 42
    assert rig.pending[0].stamp.nanosec == 0
    assert rig.manager.feedback_message == ""
    assert rig.ros == [42_000_000_000]


@pytest.mark.parametrize("status", [Status.SUCCESS, Status.RUNNING])
def test_emergency_cancel_bypasses_active_reset_hold(rig, status):
    rig.publisher.localization_reset_hold_until = 102.0
    emergency = command(CommandID.EMERGENCY_CANCEL)
    rig.manager.blackboard.command_buffer.append(emergency)
    rig.manager.blackboard.command_tree_status = status

    assert rig.manager.update() is Status.SUCCESS
    assert rig.manager.blackboard.command_buffer == []
    assert rig.manager.blackboard.last_command is emergency
    assert rig.manager.blackboard.command_tree_status is None
    assert emergency.stamp is not None
    assert all(item.stamp is None for item in rig.pending)


@pytest.mark.parametrize("deadline", [None, 0.0, 99.0, 100.0])
def test_missing_or_expired_hold_allows_normal_dispatch(rig, deadline):
    if deadline is not None:
        rig.publisher.localization_reset_hold_until = deadline

    assert rig.manager.update() is Status.SUCCESS
    assert rig.manager.blackboard.last_command is rig.pending[0]
    assert rig.manager.blackboard.command_buffer == rig.pending[1:]
    assert rig.manager.blackboard.command_tree_status is None
