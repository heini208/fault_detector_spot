"""Tests for command transport diagnostics."""

import inspect

from fault_detector_spot.application.ros.command_transport import (
    RosCommandTransport,
)


def test_ros_transport_owns_ros_feedback_conversion():
    source = inspect.getsource(RosCommandTransport.handle_command_status)

    assert "CommandStatus.STATE_RUNNING" in source
    assert "CommandExecutionStatus" in source
