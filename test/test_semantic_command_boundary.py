"""Lock the application command model outside ROS wire ownership."""

import inspect
from pathlib import Path

from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_semantic_command_model_has_no_ros_message_imports():
    source = inspect.getsource(SemanticCommand)
    module = (
        ROOT
        / "application/commanding/semantic_command.py"
    ).read_text(encoding="utf-8")

    assert "fault_detector_msgs" not in module
    assert "geometry_msgs" not in module
    assert "ComplexCommand" not in source


def test_command_controller_queue_is_semantic_not_complex():
    source = inspect.getsource(CommandController)

    assert "CommandRequest[SemanticCommand]" in source
    assert "ComplexCommand" not in source


def test_setup_coordinator_does_not_import_complex_command():
    module = (
        ROOT
        / "application/coordinators/setup_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "ComplexCommand" not in module


def test_complex_command_is_confined_to_transport_or_bt_files():
    paths = (
        ROOT / "application/ros/semantic_command_adapter.py",
        ROOT / "application/ros/command_request_adapter.py",
        ROOT / "application/behaviour_tree/behaviours/command_subscriber.py",
        ROOT / "application/recording/record_manager_node.py",
    )
    assert all(path.exists() for path in paths)
