"""Lock the application command model outside ROS wire ownership."""

import inspect
from pathlib import Path

from fault_detector_spot.application.commanding.semantic_command import SemanticCommand


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_semantic_command_model_has_no_ros_message_imports():
    source = inspect.getsource(SemanticCommand)
    module = (
        ROOT / "application/commanding/semantic_command.py"
    ).read_text(encoding="utf-8")

    assert "fault_detector_msgs" not in module
    assert "geometry_msgs" not in module
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


def test_runtime_python_has_no_legacy_ros_command_types():
    offenders = []
    for path in ROOT.rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        if "ComplexCommand" in source or "BasicCommand" in source:
            offenders.append(str(path.relative_to(ROOT)))

    assert offenders == []
