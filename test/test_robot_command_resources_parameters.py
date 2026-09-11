"""Regression tests for RobotCommandResources parameter wiring."""

import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
RESOURCES = (
    ROOT
    / "fault_detector_spot/application/behaviour_tree/behaviours/"
    "robot_command_resources.py"
)


def test_positive_parameter_calls_have_node_name_and_default():
    tree = ast.parse(RESOURCES.read_text(encoding="utf-8"))

    calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if (
            isinstance(node.func, ast.Attribute)
            and node.func.attr == "_positive_parameter"
        ):
            calls.append(node)

    assert calls
    assert all(len(call.args) == 3 for call in calls)


def test_arm_executor_has_no_removed_guarded_speed_keyword():
    tree = ast.parse(RESOURCES.read_text(encoding="utf-8"))

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if (
            isinstance(node.func, ast.Name)
            and node.func.id == "ArmMovementExecutor"
        ):
            keywords = {
                keyword.arg
                for keyword in node.keywords
                if keyword.arg is not None
            }
            assert "guarded_linear_speed_mps" not in keywords
            return

    raise AssertionError("ArmMovementExecutor construction not found")
