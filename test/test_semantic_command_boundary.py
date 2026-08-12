"""Lock the application command model outside ROS wire ownership."""

import inspect
from pathlib import Path

from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.controllers.command_controller import CommandController
from fault_detector_spot.application.coordinators.setup_coordinator import SetupCoordinator


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_semantic_command_model_has_no_ros_message_imports():
    source = inspect.getsource(SemanticCommand)
    module = (
        ROOT / "application/commanding/semantic_command.py"
    ).read_text(encoding="utf-8")

    assert "fault_detector_msgs" not in module
    assert "geometry_msgs" not in module
    assert "ComplexCommand" not in source


def test_command_controller_queue_is_semantic_not_complex():
    source = inspect.getsource(CommandController)

    assert "CommandRequest[SemanticCommand]" in source
    assert "ComplexCommand" not in source


def test_command_controller_has_no_ros_transport_dependency():
    module = (
        ROOT / "application/controllers/command_controller.py"
    ).read_text(encoding="utf-8")

    assert "fault_detector_msgs" not in module
    assert "rclpy" not in module
    assert "create_publisher" not in module
    assert "create_subscription" not in module
    assert "command_request_adapter" not in module
    assert "fault_detector/_internal" not in module


def test_setup_coordinator_does_not_import_complex_command():
    module = (
        ROOT / "application/coordinators/setup_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "ComplexCommand" not in module


def test_ros_command_transport_exists_at_ros_boundary():
    transport = ROOT / "application/ros/command_transport.py"
    source = transport.read_text(encoding="utf-8")

    assert transport.is_file()
    assert "CommandStatus" in source
    assert "command_request_from_message" in source
    assert "command_request_to_message" in source


def test_behaviour_tree_consumes_semantic_commands():
    subscriber = (
        ROOT
        / "application/behaviour_tree/behaviours/command_subscriber.py"
    ).read_text(encoding="utf-8")

    assert "ComplexCommand" not in subscriber
    assert "CommandRequest[SemanticCommand]" in subscriber
    assert "command_request_from_message" in subscriber


def test_recording_storage_is_ros_independent():
    manager = (
        ROOT / "application/recording/record_manager_node.py"
    ).read_text(encoding="utf-8")
    codec = (
        ROOT / "application/recording/semantic_command_codec.py"
    ).read_text(encoding="utf-8")

    assert "ComplexCommand" not in manager
    assert "rosidl_runtime_py" not in manager
    assert "ComplexCommand" not in codec
    assert "rosidl_runtime_py" not in codec
    assert "SemanticCommand" in codec


def test_complex_command_is_confined_to_semantic_ros_wire_adapter():
    semantic_adapter = ROOT / "application/ros/semantic_command_adapter.py"
    request_adapter = ROOT / "application/ros/command_request_adapter.py"

    assert semantic_adapter.exists()
    assert request_adapter.exists()
    assert "ComplexCommand" in semantic_adapter.read_text(encoding="utf-8")
    assert "ComplexCommand" not in request_adapter.read_text(
        encoding="utf-8"
    )
