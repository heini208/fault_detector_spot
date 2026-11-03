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


def test_executor_delegates_probe_geometry_to_planner():
    executor = _source(EXECUTOR)
    planner = _source(PLANNER)
    executor_class = next(
        node for node in ast.parse(executor).body
        if isinstance(node, ast.ClassDef)
        and node.name == "ArmMovementExecutor"
    )
    executor_methods = {
        node.name for node in executor_class.body
        if isinstance(node, ast.FunctionDef)
    }

    for name in (
        "resolve_relative",
        "resolve_tag",
        "resolve_probe_relative",
        "build_plan",
        "build_probe_plan",
        "probe_target_to_hand_target",
        "normalize_target",
    ):
        assert name not in executor_methods
        assert f"def {name}(" in planner


def test_force_guard_state_machine_is_not_inside_arm_executor():
    executor = _source(EXECUTOR)
    guard = _source(GUARD)

    for name in (
        "_check_force_guard",
        "_begin_contact",
        "_begin_retreat",
        "_begin_arm_stop",
        "_poll_arm_stop",
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


def test_command_builder_owns_robot_command_translation():
    executor = _source(EXECUTOR)

    builder = _source(EXECUTOR.with_name("arm_command_builder.py"))
    assert "RobotCommandBuilder.arm_pose_command" in builder
    assert "RobotCommandBuilder.arm_stow_command" in builder
    assert "ArmStopCommand.Request" in builder
    assert "RobotCommandBuilder" not in executor
    assert "tf2_geometry_msgs" not in executor
    assert "MovementGeometryResolver" not in executor


def test_guard_does_not_define_a_separate_movement_speed():
    executor = _source(EXECUTOR)
    planner = _source(PLANNER)
    resources = _source(RESOURCES)

    for source in (executor, planner, resources):
        assert "guarded_linear_speed" not in source
        assert "GUARDED_LINEAR_SPEED" not in source


def test_only_probe_moveit_completion_and_native_stow_submit_arm_motion():
    tree = ast.parse(_source(EXECUTOR))
    executor = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                    and node.name == "ArmMovementExecutor")
    callers = {}
    for method in executor.body:
        if not isinstance(method, ast.FunctionDef):
            continue
        for call in ast.walk(method):
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute):
                callers.setdefault(call.func.attr, set()).add(method.name)

    assert callers["_submit_goal"] == {
        "_stage_probe_goal", "_start_prepared_guarded_motion", "_poll_state_confirmation",
    }
    assert callers["_build_pose_goal"] == {"probe"}
    assert callers["probe"] == {"_continue_probe", "_continue_contact_retreat"}
    assert "prepare_motion=self._prepare_guarded_motion" in _source(EXECUTOR)
    assert "poll_preparation=self._poll_guarded_motion_preparation" in _source(EXECUTOR)
    assert "start_motion=self._start_prepared_guarded_motion" in _source(EXECUTOR)
    assert "start_retreat=self._continue_contact_retreat" in _source(EXECUTOR)
    assert "self._start_motion(plan)" in _source(GUARD)
    assert "self._start_retreat(retreat_plan)" in _source(GUARD)
