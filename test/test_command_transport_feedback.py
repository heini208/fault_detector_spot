"""Tests for command transport diagnostics."""

import inspect

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
)
from fault_detector_spot.application.ros.command_transport import (
    RosCommandTransport,
)


def test_bt_transport_acknowledges_received_requests():
    source = inspect.getsource(CommandSubscriber.append_request_to_buffer)

    assert "BT received" in source
    assert "_publish_request_status" in source


def test_bt_transport_rejection_publishes_correlated_failure():
    source = inspect.getsource(CommandSubscriber.append_request_to_buffer)

    assert "_publish_raw_request_failure" in source


def test_controller_has_transport_independent_bt_ack_timeout():
    source = inspect.getsource(CommandController._retry_dispatch_locked)

    assert "did not acknowledge" in source
    assert "_ack_timeout_sec" in source
    assert "CommandStatus" not in source


def test_ros_transport_owns_ros_feedback_conversion():
    source = inspect.getsource(RosCommandTransport.handle_command_status)

    assert "CommandStatus.STATE_RUNNING" in source
    assert "CommandExecutionStatus" in source
