"""Callback delivery for context-scoped operations."""

import pytest

from fault_detector_spot.application.setup.setup_operation_registry import SetupOperationRegistry


def test_listener_failure_does_not_drop_other_terminal_notifications():
    registry, received = SetupOperationRegistry(), []

    def fail(_value):
        raise RuntimeError("Listener failed")

    registry.add_listener(fail)
    registry.add_listener(received.append)
    with pytest.raises(RuntimeError, match="Listener failed"):
        registry.emit("terminal")
    assert received == ["terminal"]
