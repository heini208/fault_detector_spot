"""Tests for command-controller process ownership."""

import inspect

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)


def test_behavior_tree_runner_does_not_create_command_controller():
    source = inspect.getsource(runner.main)

    assert "CommandController(" not in source
    assert "tree.node.command_controller" not in source


def test_application_api_remains_command_controller_owner():
    source = inspect.getsource(ApplicationApiNode.__init__)

    assert "self.command_controller = CommandController(" in source
    assert (
        "self.command_transport = RosCommandTransport(\n"
        "            self,\n"
        "            self.command_controller"
    ) in source
