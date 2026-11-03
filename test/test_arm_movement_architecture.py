"""Lock the centralized arm movement architecture."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_common_movement_behaviour_only_adapts_tree_to_executor():
    move = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )

    assert "self.executor.poll()" in move
    assert "self.executor.cancel()" in move
    assert "send_goal_async" not in move
    assert "get_result_async" not in move
    assert "cancel_goal_async" not in move
    assert "RobotCommandBuilder" not in move
    assert "_prepare_move_command" not in move
    assert "_resolve_and_transform_offset_if_tag" not in move
    assert "_resolve_tag_alias" not in move
    assert "_can_transform" not in move


def test_arm_goal_behaviour_only_dispatches_to_executor():
    goal = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_goal_behaviour.py"
    )

    assert "_prepare_operation" not in goal
    assert "_prepare_move_command" not in goal
    assert "GRAV_ALIGNED_BODY_FRAME_NAME" not in goal
    assert "executor.relative(command, speed=speed)" in goal
    assert "executor.tag_probe(command, speed=speed)" in goal


def test_arm_executor_inherits_shared_robot_command_lifecycle():
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    move = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )

    assert "class ArmMovementExecutor(MovementExecutor)" in executor
    assert "send_goal_async(" in shared
    assert "get_result_async(" in shared
    assert "cancel_goal_async(" in shared

    assert "send_goal_async(" not in executor
    assert "get_result_async(" not in executor
    assert "cancel_goal_async(" not in executor
    assert "send_goal_async(" not in move
    assert "get_result_async(" not in move
    assert "cancel_goal_async(" not in move


def test_probe_motion_planner_owns_probe_to_hand_conversion():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    planner = read(
        "fault_detector_spot/manipulation/probe_motion_planner.py"
    )
    command = read(
        "fault_detector_spot/manipulation/commands/"
        "manipulator_to_tag_command.py"
    )

    assert "def probe_pose(" in executor
    assert "def tag_probe(" in executor
    assert "def probe_relative(" in executor
    assert "ProbeMotionPlanner(" in executor
    assert "def probe_target_to_hand_target(" in planner
    assert "def hand_to_probe_pose(" in planner

    assert "_probe_target_to_hand_target" not in command
    assert "sensor_probe_frame" not in command
    assert "hand_to_probe" not in command


def test_probe_planner_owns_live_arm_geometry_preparation():
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    geometry = read(
        "fault_detector_spot/shared/geometry/movement_geometry.py"
    )
    planner = read(
        "fault_detector_spot/manipulation/probe_motion_planner.py"
    )
    base_planner = read(
        "fault_detector_spot/navigation/base_motion_planner.py"
    )
    base = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "MovementGeometryResolver(" in shared
    assert "def _prepare_move_command(" in shared
    assert "def resolve_and_transform_offset_if_tag(" in geometry
    assert "MovementGeometryResolver(" in planner
    assert "prepare_move_command(" in planner
    assert "MovementGeometryResolver(" in base_planner
    assert "prepare_move_command(" in base_planner
    assert "_prepare_move_command(" not in base
