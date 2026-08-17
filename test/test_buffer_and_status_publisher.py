"""Tests for correlated, transition-only robot command statuses."""

from enum import Enum
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.application.commanding.request_identity import new_request_id


class FakePublisher:
    """Record published ROS messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def behavior(status=Status.SUCCESS, command_id="move_to_tag"):
    """Create a publisher with isolated blackboard data."""
    result = BufferStatusPublisher()
    result.blackboard = SimpleNamespace(
        command_buffer=[],
        command_tree_status=status,
        last_command=SimpleNamespace(
            command_id=command_id,
            request_id=new_request_id(),
        ),
        command_failure_request_id="",
        command_failure_detail="",
    )
    result.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=7, nanosec=11)
            )
        )
    )
    result.structured_status_pub = FakePublisher()
    return result


def test_unchanged_terminal_status_is_published_only_once():
    result = behavior(Status.SUCCESS, "move_to_tag")

    assert result.update() == Status.SUCCESS
    assert result.update() == Status.SUCCESS

    assert len(result.structured_status_pub.messages) == 1


def test_new_running_command_produces_a_new_correlated_transition():
    result = behavior(Status.SUCCESS, "move_to_tag")
    result.update()
    result.blackboard.command_tree_status = Status.RUNNING
    result.blackboard.last_command = SimpleNamespace(
        command_id="cancel_all",
        request_id=new_request_id(),
    )

    result.update()

    messages = result.structured_status_pub.messages
    assert [message.command_id for message in messages] == [
        "move_to_tag",
        "cancel_all",
    ]


def test_string_enum_command_ids_publish_their_wire_value():
    class TestCommandID(str, Enum):
        MOVE = "move_to_tag"

    result = behavior(Status.SUCCESS, TestCommandID.MOVE)

    result.update()

    assert result.structured_status_pub.messages[0].command_id == (
        "move_to_tag"
    )


def test_internal_success_is_running_until_request_buffer_is_empty():
    result = behavior(Status.SUCCESS, "move_to_tag")
    request_id = result.blackboard.last_command.request_id
    result.blackboard.command_buffer = [
        SimpleNamespace(
            command_id="wait_time",
            request_id=request_id,
        ),
        SimpleNamespace(
            command_id="stow_arm",
            request_id=new_request_id(),
        ),
    ]

    result.update()

    message = result.structured_status_pub.messages[0]
    assert message.state == message.STATE_RUNNING
    assert message.buffered_command_count == 1


def test_correlated_leaf_failure_detail_is_published():
    result = behavior(Status.FAILURE, "move_close_to_surface")
    request_id = result.blackboard.last_command.request_id
    result.blackboard.command_failure_request_id = request_id
    result.blackboard.command_failure_detail = (
        "Unable to establish stationary end-effector force baseline"
    )

    result.update()

    message = result.structured_status_pub.messages[0]
    assert message.state == message.STATE_FAILED
    assert message.detail == (
        "move_close_to_surface failed: Unable to establish stationary "
        "end-effector force baseline"
    )


def test_failure_detail_from_another_request_is_not_reused():
    result = behavior(Status.FAILURE, "move_close_to_surface")
    result.blackboard.command_failure_request_id = new_request_id()
    result.blackboard.command_failure_detail = "stale failure"

    result.update()

    message = result.structured_status_pub.messages[0]
    assert message.detail == (
        "BT execution failed for move_close_to_surface"
    )
