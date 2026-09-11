"""Lock the arm executor boundary between guarded and low-level probe motion."""

import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
EXECUTOR = (
    ROOT
    / "fault_detector_spot"
    / "manipulation"
    / "arm_movement_executor.py"
)
GUARD = (
    ROOT
    / "fault_detector_spot"
    / "manipulation"
    / "guarded_probe_execution.py"
)
RESOURCES = (
    ROOT
    / "fault_detector_spot"
    / "application"
    / "behaviour_tree"
    / "behaviours"
    / "robot_command_resources.py"
)


def _source(path):
    return path.read_text(encoding="utf-8")


def _method_source(source, method_name):
    marker = f"    def {method_name}("
    body = source.split(marker, 1)[1]
    next_method = body.find("\n    def ")
    if next_method >= 0:
        body = body[:next_method]
    return body


def test_application_arm_entries_route_through_guarded_probe():
    source = _source(EXECUTOR)

    for method_name in (
        "relative",
        "pose",
        "probe_pose",
        "tag_probe",
        "probe_relative",
        "guarded_probe",
    ):
        method = _method_source(source, method_name)
        assert (
            "_start_guarded_probe(" in method
            or "self.guarded_probe(" in method
        )


def test_low_level_probe_is_explicitly_unguarded():
    source = _source(EXECUTOR)
    method = _method_source(source, "probe")

    assert "_submit_probe(" in method
    assert "_start_guarded_probe(" not in method


def test_ready_arm_uses_low_level_probe_submission_not_force_guard():
    source = _source(EXECUTOR)
    method = _method_source(source, "_advance_prepare_start")

    assert "_submit_probe(" in method
    assert "_start_guarded_probe(" not in method
    assert "guarded_probe_execution" not in method


def test_force_guard_state_machine_is_not_inside_arm_executor():
    executor_source = _source(EXECUTOR)
    guard_source = _source(GUARD)

    for name in (
        "_check_force_guard",
        "_begin_contact",
        "_begin_retreat",
        "_handle_stop_settling",
    ):
        assert f"def {name}(" not in executor_source
        assert f"def {name}(" in guard_source


def test_robot_command_resource_parameter_calls_have_three_arguments():
    tree = ast.parse(_source(RESOURCES))

    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_positive_parameter"
    ]

    assert calls
    assert all(len(call.args) == 3 for call in calls)


def test_guard_does_not_define_a_separate_movement_speed():
    executor_source = _source(EXECUTOR)
    resources_source = _source(RESOURCES)

    assert "guarded_linear_speed" not in executor_source
    assert "GUARDED_LINEAR_SPEED" not in executor_source
    assert "guarded_linear_speed" not in resources_source
    assert "GUARDED_LINEAR_SPEED" not in resources_source


def test_guard_uses_executor_default_or_explicit_speed():
    source = _source(EXECUTOR)
    method = _method_source(source, "_effective_guarded_speed")

    assert "self.speed_policy.default_speed" in method
    assert "return speed" in method
