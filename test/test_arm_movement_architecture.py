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
    assert "_probe_target_to_hand_target" in executor

    assert "_probe_target_to_hand_target" not in command
    assert "sensor_probe_frame" not in command
    assert "hand_to_probe" not in command


def test_public_arm_api_exposes_speed_not_duration():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    action = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_movement_action.py"
    )

    for signature in (
        "def relative(",
        "def pose(",
        "def probe_pose(",
        "def tag_probe(",
        "def probe_relative(",
    ):
        assert signature in executor

    assert "relative_duration_sec" not in action
    assert "tag_duration_sec" not in action
    assert "DEFAULT_RELATIVE_DURATION_SEC" not in action
    assert "DEFAULT_TAG_DURATION_SEC" not in action
    assert "executor.relative(command)" in action
    assert "executor.tag_probe(command)" in action


def test_duration_is_private_spot_translation_detail():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )

    assert "duration_between(" in executor
    assert "def _build_pose_goal(" in executor
    assert "duration_sec: float" in executor
    assert "RobotCommandBuilder.arm_pose_command(" in executor


def test_executor_owns_robot_command_lifecycle():
    executor = read(
        "fault_detector_spot/manipulation/arm_movement_executor.py"
    )
    action = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_movement_action.py"
    )
    resources = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "robot_command_resources.py"
    )

    assert "send_goal_async(" in executor
    assert "get_result_async(" in executor
    assert "cancel_goal_async(" in executor
    assert "def poll(" in executor
    assert "def cancel(" in executor

    assert "send_goal_async(" not in action
    assert "get_result_async(" not in action
    assert "cancel_goal_async(" not in action
    assert "arm_movement_executor.poll()" in action
    assert "arm_movement_executor.cancel()" in action

    assert "action_client=self.get_action_client(" in resources
    assert "executor.shutdown" in resources


def test_arm_action_reuses_tf_preparation_without_generic_action_lifecycle():
    action = read(
        "fault_detector_spot/manipulation/behaviours/"
        "arm_movement_action.py"
    )
    move_action = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "move_command_action.py"
    )

    assert "_prepare_move_command(command)" in action
    assert "def _prepare_move_command(" in move_action
    assert "super().update()" not in action
    assert "super().terminate(" not in action
