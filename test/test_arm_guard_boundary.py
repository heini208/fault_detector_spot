"""Lock the arm boundary between orchestration, planning, and force guard."""

import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
EXECUTOR = (
    ROOT
    / "fault_detector_spot"
    / "manipulation"
    / "arm_movement_executor.py"
)
PLANNER = (
    ROOT
    / "fault_detector_spot"
    / "manipulation"
    / "probe_motion_planner.py"
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


def test_executor_delegates_probe_geometry_to_planner():
    executor = _source(EXECUTOR)
    planner = _source(PLANNER)

    for name in (
        "resolve_relative",
        "resolve_tag",
        "resolve_probe_relative",
        "build_plan",
        "build_probe_goal",
        "probe_target_to_hand_target",
        "normalize_target",
    ):
        assert f"def {name}(" not in executor
        assert f"def {name}(" in planner


def test_force_guard_state_machine_is_not_inside_arm_executor():
    executor = _source(EXECUTOR)
    guard = _source(GUARD)

    for name in (
        "_check_force_guard",
        "_begin_contact",
        "_begin_retreat",
        "_handle_stop_settling",
    ):
        assert f"def {name}(" not in executor
        assert f"def {name}(" in guard


def test_force_threshold_policy_belongs_to_guard_not_motion_planner():
    planner = _source(PLANNER)
    guard = _source(GUARD)

    assert "force_contact_policy" not in planner
    assert "threshold_for(" not in planner
    assert "force_contact_policy" in guard
    assert "threshold_for(" in guard


def test_executor_keeps_only_robot_command_translation():
    executor = _source(EXECUTOR)

    assert "RobotCommandBuilder.arm_pose_command" in executor
    assert "RobotCommandBuilder.arm_stow_command" in executor
    assert "tf2_geometry_msgs" not in executor
    assert "MovementGeometryResolver" not in executor


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
    executor = _source(EXECUTOR)
    planner = _source(PLANNER)
    resources = _source(RESOURCES)

    for source in (executor, planner, resources):
        assert "guarded_linear_speed" not in source
        assert "GUARDED_LINEAR_SPEED" not in source
