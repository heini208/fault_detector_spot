"""Lock the centralized base movement architecture."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_base_behaviour_hierarchy_matches_arm_structure():
    common = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )
    base = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_movement_behaviour.py"
    )
    goal = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_goal_behaviour.py"
    )
    stand = read(
        "fault_detector_spot/navigation/behaviours/"
        "stand_up_behaviour.py"
    )

    assert "class MovementBehaviour(" in common
    assert "class BaseMovementBehaviour(MovementBehaviour)" in base
    assert "class BaseGoalBehaviour(BaseMovementBehaviour)" in goal
    assert "class StandUpBehaviour(BaseMovementBehaviour)" in stand


def test_runner_uses_one_base_goal_behaviour_for_relative_and_tag():
    runner = read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )

    assert runner.count("BaseGoalBehaviour(") == 2
    assert "BaseMoveRelativeAction" not in runner
    assert "BaseMoveToTagAction" not in runner
    assert "BaseGetGoalTag" not in runner
    assert "StandUpActionSimple" not in runner
    assert "StandUpBehaviour(" in runner


def test_base_behaviours_only_delegate_to_shared_executor():
    base = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_movement_behaviour.py"
    )
    goal = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_goal_behaviour.py"
    )
    stand = read(
        "fault_detector_spot/navigation/behaviours/"
        "stand_up_behaviour.py"
    )

    assert "get_base_movement_executor(" in base
    assert "return self.executor.relative(command)" in goal
    assert "return self.executor.tag(command)" in goal
    assert "return self.executor.stand()" in stand
    assert "RobotCommandBuilder" not in base
    assert "RobotCommandBuilder" not in goal
    assert "RobotCommandBuilder" not in stand


def test_base_executor_inherits_lifecycle_and_owns_goal_building():
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "class BaseMovementExecutor(MovementExecutor)" in executor
    assert "send_goal_async(" in shared
    assert "get_result_async(" in shared
    assert "cancel_goal_async(" in shared
    assert "send_goal_async(" not in executor
    assert "get_result_async(" not in executor
    assert "cancel_goal_async(" not in executor

    assert "RobotCommandBuilder.synchro_stand_command()" in executor
    assert (
        "RobotCommandBuilder.synchro_se2_trajectory_point_command("
        in executor
    )
    assert "def relative(" in executor
    assert "def tag(" in executor
    assert "def stand(" in executor


def test_base_tag_movement_uses_authoritative_live_visible_tag_state():
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )
    goal = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_goal_behaviour.py"
    )

    assert "visible_snapshot()" in executor
    assert "visible_snapshot()" not in goal
    assert "command.tag_pose = deepcopy(tag.pose)" in executor
    assert "command.tag_pose = deepcopy(tag.pose)" not in goal
    assert "_prepare_operation" not in goal