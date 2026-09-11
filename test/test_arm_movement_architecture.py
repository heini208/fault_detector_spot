"""Lock the centralized arm movement architecture."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_runner_uses_one_arm_goal_behaviour_for_relative_and_tag():
    runner = read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )

    assert runner.count("ArmGoalBehaviour(") == 2
    assert "ArmMovementAction" not in runner
    assert "ReadyArmActionSimple" not in runner
    assert "StowArmActionSimple" not in runner


def test_arm_behavior_hierarchy_has_one_common_move_adapter():
    move = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )
    arm = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_movement_behaviour.py"
    )
    goal = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_goal_behaviour.py"
    )
    ready = read(
        "fault_detector_spot/manipulation/behaviours/"
        "ready_arm_behaviour.py"
    )
    stow = read(
        "fault_detector_spot/manipulation/behaviours/"
        "stow_arm_behaviour.py"
    )

    assert "class MovementBehaviour(" in move
    assert "class ArmMovementBehaviour(MovementBehaviour)" in arm
    assert "class ArmGoalBehaviour(ArmMovementBehaviour)" in goal
    assert "class ReadyArmBehaviour(ArmMovementBehaviour)" in ready
    assert "class StowArmBehaviour(ArmMovementBehaviour)" in stow


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


def test_ready_and_stow_are_small_executor_dispatchers():
    ready = read(
        "fault_detector_spot/manipulation/behaviours/"
        "ready_arm_behaviour.py"
    )
    stow = read(
        "fault_detector_spot/manipulation/behaviours/"
        "stow_arm_behaviour.py"
    )

    assert "return self.executor.prepare()" in ready
    assert "return self.executor.stow()" in stow
    assert "RobotCommandBuilder" not in ready
    assert "RobotCommandBuilder" not in stow
    assert "def update(" not in ready
    assert "def update(" not in stow
    assert "def terminate(" not in ready
    assert "def terminate(" not in stow


def test_arm_goal_behaviour_only_dispatches_to_executor():
    goal = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_goal_behaviour.py"
    )

    assert "_prepare_operation" not in goal
    assert "_prepare_move_command" not in goal
    assert "GRAV_ALIGNED_BODY_FRAME_NAME" not in goal
    assert "executor.relative(command)" in goal
    assert "executor.tag_probe(command)" in goal


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



def test_renamed_movement_behaviour_files_replace_old_action_names():
    removed = (
        "fault_detector_spot/application/behaviour_tree/behaviours/move_action.py",
        "fault_detector_spot/manipulation/behaviours/move_arm_action.py",
        "fault_detector_spot/manipulation/behaviours/move_arm_goal_action.py",
        "fault_detector_spot/manipulation/behaviours/ready_arm_action.py",
        "fault_detector_spot/manipulation/behaviours/stow_arm_action.py",
        "fault_detector_spot/manipulation/behaviours/arm_movement_action.py",
    )

    for relative_path in removed:
        assert not (ROOT / relative_path).exists()


def test_executor_owns_probe_to_hand_conversion():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    command = read(
        "fault_detector_spot/manipulation/commands/"
        "manipulator_to_tag_command.py"
    )

    assert "def probe_pose(" in executor
    assert "def tag_probe(" in executor
    assert "def probe_relative(" in executor
    assert "_probe_target_to_hand_target" in executor

    assert "_probe_target_to_hand_target" not in command
    assert "sensor_probe_frame" not in command
    assert "hand_to_probe" not in command


def test_public_arm_api_exposes_speed_not_duration():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    goal = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_goal_behaviour.py"
    )

    for signature in (
        "def relative(",
        "def pose(",
        "def probe_pose(",
        "def tag_probe(",
        "def probe_relative(",
    ):
        assert signature in executor

    assert "duration_sec" not in goal
    assert "executor.relative(command)" in goal
    assert "executor.tag_probe(command)" in goal



def test_arm_and_base_executors_share_only_the_lifecycle_parent():
    arm = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    base = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )

    assert "class MovementExecutor:" in shared
    assert "class ArmMovementExecutor(MovementExecutor)" in arm
    assert "class BaseMovementExecutor(MovementExecutor)" in base
    assert "_handle_successful_result" in arm
    assert "_build_probe_motion_goal" in arm
    assert "_build_se2_goal" in base



def test_shared_executor_owns_live_movement_geometry_preparation():
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    geometry = read(
        "fault_detector_spot/shared/geometry/movement_geometry.py"
    )
    arm = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    base = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "MovementGeometryResolver(" in shared
    assert "def _prepare_move_command(" in shared
    assert "def resolve_and_transform_offset_if_tag(" in geometry
    assert "_prepare_move_command(" in arm
    assert "_prepare_move_command(" in base
