"""Tests for correlated, transition-only robot command statuses."""

from enum import Enum
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from py_trees.common import Status

from fault_detector_spot.behaviour_tree.nodes.sensing.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.request_identity import new_request_id


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
    )
    result.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=7, nanosec=11)
            )
        )
    )
    result.buffer_pub = FakePublisher()
    result.status_pub = FakePublisher()
    result.structured_status_pub = FakePublisher()
    return result


def test_terminal_status_contains_the_completed_command_id():
    result = behavior(Status.SUCCESS, "move_to_tag")

    message = result.get_status_message()

    assert message.data == "SUCCESS: move_to_tag"


def test_unchanged_terminal_status_is_published_only_once():
    result = behavior(Status.SUCCESS, "move_to_tag")

    assert result.update() == Status.SUCCESS
    assert result.update() == Status.SUCCESS

    assert [message.data for message in result.status_pub.messages] == [
        "SUCCESS: move_to_tag"
    ]


def test_new_running_command_produces_a_new_correlated_transition():
    result = behavior(Status.SUCCESS, "move_to_tag")
    result.update()
    result.blackboard.command_tree_status = Status.RUNNING
    result.blackboard.last_command = SimpleNamespace(
        command_id="cancel_all",
        request_id=new_request_id(),
    )

    result.update()

    assert [message.data for message in result.status_pub.messages] == [
        "SUCCESS: move_to_tag",
        "Running: cancel_all",
    ]


def test_string_enum_command_ids_publish_their_wire_value():
    class TestCommandID(str, Enum):
        MOVE = "move_to_tag"

    result = behavior(Status.SUCCESS, TestCommandID.MOVE)

    assert result.get_status_message().data == "SUCCESS: move_to_tag"
