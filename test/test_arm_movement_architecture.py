"""Lock the single arm movement behavior and executor path."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_runner_uses_one_arm_movement_behavior_for_relative_and_tag():
    runner = read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )

    assert runner.count("ArmMovementAction(") == 2
    assert "ManipulatorGetGoalTag" not in runner
    assert "ManipulatorMoveArmAction" not in runner
    assert "ManipulatorMoveRelativeAction" not in runner
    assert "build_manipulator_goal_tree" not in runner


def test_executor_owns_live_reachable_tag_resolution():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )

    assert "def tag_pose(" in executor
    assert "reachable_tag(tag_id)" in executor
    assert "command.tag_pose = deepcopy(tag.pose)" in executor
    assert "return self.pose(target, duration_sec)" in executor
