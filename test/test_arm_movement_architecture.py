"""Lock the centralized hand/probe arm movement architecture."""

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
    assert "sensor_probe_frame" in executor
    assert "compose_poses" in executor
    assert "inverse_pose" in executor

    assert "_probe_target_to_hand_target" not in command
    assert "sensor_probe_frame" not in command
    assert "hand_to_probe" not in command


def test_tag_probe_uses_live_tag_then_probe_pose():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )

    assert "reachable_tag(tag_id)" in executor
    assert "command.tag_pose = deepcopy(tag.pose)" in executor
    assert "return self.probe_pose(" in executor


def test_arm_behavior_routes_tag_motion_through_tag_probe():
    action = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_movement_action.py"
    )

    assert "executor.relative(" in action
    assert "executor.tag_probe(" in action
    assert "executor.tag_pose(" not in action
