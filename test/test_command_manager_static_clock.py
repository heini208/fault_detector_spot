"""Tests for command dispatch with a static or rewound ROS clock."""

from types import SimpleNamespace

from py_trees.common import Status

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import (
    CommandManager,
)
from fault_detector_spot.application.behaviour_tree.behaviours.new_command_guard import (
    NewCommandGuard,
)


class FakeClock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class FakeNode:
    def __init__(self, nanoseconds):
        self.clock = FakeClock(nanoseconds)

    def get_clock(self):
        return self.clock


class FakeManagerBlackboard:
    def __init__(self, commands):
        self.command_buffer = list(commands)
        self.command_tree_status = Status.SUCCESS
        self.last_command = None


class FakeGuardBlackboard:
    def __init__(self):
        self.last_command = None
        self.last_processed_command = None

    def exists(self, key):
        return getattr(self, key, None) is not None


def make_command(command_id):
    return SimpleNamespace(
        command_id=command_id,
        stamp=None,
    )


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def test_static_clock_dispatches_distinct_command_stamps():
    commands = [
        make_command(CommandID.CREATE_INSPECTION_OBJECT),
        make_command(CommandID.DELETE_INSPECTION_OBJECT),
    ]
    manager = CommandManager()
    manager.node = FakeNode(42_000_000_000)
    manager.blackboard = FakeManagerBlackboard(commands)
    manager._last_dispatch_stamp_nanoseconds = -1

    assert manager.update() == Status.SUCCESS
    first_command = manager.blackboard.last_command
    assert manager.blackboard.command_tree_status is None
    manager.blackboard.command_tree_status = Status.SUCCESS

    assert manager.update() == Status.SUCCESS
    second_command = manager.blackboard.last_command
    assert manager.blackboard.command_tree_status is None

    assert stamp_nanoseconds(second_command.stamp) == (
        stamp_nanoseconds(first_command.stamp) + 1
    )


def test_rewound_clock_keeps_command_stamps_increasing():
    commands = [
        make_command(CommandID.CREATE_INSPECTION_OBJECT),
        make_command(CommandID.CREATE_INSPECTION_ROUTINE),
    ]
    manager = CommandManager()
    manager.node = FakeNode(100_000_000_000)
    manager.blackboard = FakeManagerBlackboard(commands)
    manager._last_dispatch_stamp_nanoseconds = -1

    manager.update()
    first_stamp = manager.blackboard.last_command.stamp
    manager.node.clock.nanoseconds = 20_000_000_000
    manager.blackboard.command_tree_status = Status.SUCCESS
    manager.update()
    second_stamp = manager.blackboard.last_command.stamp

    assert stamp_nanoseconds(second_stamp) == (
        stamp_nanoseconds(first_stamp) + 1
    )


def test_new_command_guard_accepts_static_clock_dispatches():
    commands = [
        make_command(CommandID.CREATE_INSPECTION_OBJECT),
        make_command(CommandID.DELETE_INSPECTION_OBJECT),
    ]
    manager = CommandManager()
    manager.node = FakeNode(42_000_000_000)
    manager.blackboard = FakeManagerBlackboard(commands)
    manager._last_dispatch_stamp_nanoseconds = -1

    guard = NewCommandGuard()
    guard.blackboard = FakeGuardBlackboard()

    manager.update()
    guard.blackboard.last_command = manager.blackboard.last_command
    assert guard.update() == Status.SUCCESS

    manager.blackboard.command_tree_status = Status.SUCCESS
    manager.update()
    guard.blackboard.last_command = manager.blackboard.last_command
    assert guard.update() == Status.SUCCESS
