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
    sit = read(
        "fault_detector_spot/navigation/behaviours/"
        "sit_down_behaviour.py"
    )

    assert "class MovementBehaviour(" in common
    assert "class BaseMovementBehaviour(MovementBehaviour)" in base
    assert "class BaseGoalBehaviour(BaseMovementBehaviour)" in goal
    assert "class StandUpBehaviour(BaseMovementBehaviour)" in stand
    assert "class SitDownBehaviour(BaseMovementBehaviour)" in sit


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
    assert "SitDownBehaviour(" in runner
    assert "CommandID.SIT_DOWN" in runner


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
    sit = read(
        "fault_detector_spot/navigation/behaviours/"
        "sit_down_behaviour.py"
    )

    assert "get_base_movement_executor(" in base
    assert "return self.executor.relative(command)" in goal
    assert "return self.executor.tag(command)" in goal
    assert "return self.executor.stand()" in stand
    assert "return self.executor.sit()" in sit
    assert "RobotCommandBuilder" not in base
    assert "RobotCommandBuilder" not in goal
    assert "RobotCommandBuilder" not in stand
    assert "RobotCommandBuilder" not in sit


def test_base_executor_inherits_lifecycle_and_planner_builds_planar_goals():
    shared = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )
    planner = read(
        "fault_detector_spot/navigation/base_motion_planner.py"
    )

    assert "class BaseMovementExecutor(MovementExecutor)" in executor
    assert "send_goal_async(" in shared
    assert "get_result_async(" in shared
    assert "cancel_goal_async(" in shared
    assert "send_goal_async(" not in executor
    assert "def cancel(" in executor
    assert "handle.get_result_async()" in executor
    assert "cancel_goal_async(" in executor

    assert "RobotCommandBuilder.synchro_stand_command(" in executor
    assert "RobotCommandBuilder.synchro_sit_command()" in executor
    assert (
        "RobotCommandBuilder.synchro_se2_trajectory_point_command("
        in planner
    )
    assert (
        "RobotCommandBuilder.synchro_se2_trajectory_point_command("
        not in executor
    )
    assert "SE2VelocityLimit" in planner
    assert "SE2VelocityLimit" not in executor
    assert "def relative(" in executor
    assert "def tag(" in executor
    assert "def stand(" in executor
    assert "def sit(" in executor


def test_base_motion_planner_owns_target_resolution():
    planner = read(
        "fault_detector_spot/navigation/base_motion_planner.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )
    goal = read(
        "fault_detector_spot/navigation/behaviours/"
        "base_goal_behaviour.py"
    )

    assert "class BaseMotionPlanner:" in planner
    assert "def resolve_relative(" in planner
    assert "def resolve_tag(" in planner
    assert "def normalize_target(" in planner
    assert "def planar_target(" in planner
    assert "def build_goal(" in planner
    assert "visible_snapshot()" in planner
    assert "do_transform_pose_stamped(" in planner
    assert "visible_snapshot()" not in executor
    assert "do_transform_pose_stamped(" not in executor
    assert "visible_snapshot()" not in goal
    assert "self.motion_planner.resolve_relative(command)" in executor
    assert "self.motion_planner.resolve_tag(" in executor
    assert "self.motion_planner.resolve_tag_observation(" in executor
    assert "def resolve_tag_observation(" in planner
    assert "_movement_plan_builder" in executor
    assert "_movement_goal_builder" not in executor
    assert "def _build_relative_goal(" not in executor
    assert "def _build_tag_goal(" not in executor


def test_base_pose_source_owns_measured_pose_resolution():
    source = read(
        "fault_detector_spot/navigation/base_pose_source.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "class BasePoseSample:" in source
    assert "class BasePoseSource:" in source
    assert "lookup_a_tform_b(" in source
    assert "ODOM_FRAME_NAME" in source
    assert "BODY_FRAME_NAME" in source
    assert "lookup_a_tform_b(" not in executor
    assert "self.base_pose_source.sample()" in executor


def test_correction_policy_owns_correction_decisions():
    policy = read(
        "fault_detector_spot/navigation/base_correction_policy.py"
    )
    verifier = read(
        "fault_detector_spot/navigation/base_goal_verifier.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "class BaseCorrectionPolicy:" in policy
    assert "CORRECT" in policy
    assert "RETRY_FROZEN_PLAN" not in policy
    assert "maximum_attempts" in policy
    assert "minimum_progress_ratio" in policy
    assert "maximum_attempts" not in verifier
    assert "minimum_progress_ratio" not in verifier
    assert "_correction_attempts" not in executor
    assert "_previous_correction_error" not in executor
    assert "self.correction_policy.decide(" in executor


def test_base_executor_uses_explicit_execution_phases():
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "class _BasePhase(Enum):" in executor
    assert "WAITING_FOR_POSTURE" in executor
    assert "EXECUTING_STAND" in executor
    assert "CONFIRMING_STANDING" in executor
    assert "EXECUTING_MOVEMENT" in executor
    assert "VERIFYING_ENDPOINT" in executor
    assert "WAITING_FOR_FRESH_TAG" in executor
    assert "TAG_OBSERVATION_TIMEOUT" in executor
    assert "CORRECTING" in executor
    assert "CANCELLING" in executor
    assert "FROZEN_TARGET" in executor
    assert "FRESH_TAG_TARGET" in executor
    assert "EXECUTING_SIT" in executor
    assert "MOVEMENT_STAND" not in executor
    assert "_verification_started" not in executor
    assert "_state_wait_started" not in executor
    assert "if self._phase is _BasePhase.VERIFYING_ENDPOINT:" in executor
    assert "tag_observation_timeout_sec" in executor
    assert (
        "if self._phase is _BasePhase.WAITING_FOR_FRESH_TAG:"
        in executor
    )
    assert "if self._goal_verifier is not None:" not in executor


def test_direct_planar_moves_share_one_submission_choke_point():
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )

    assert "def _submit_movement_plan(" in executor

    initial = executor.split(
        "def _submit_movement_goal",
        1,
    )[1].split(
        "def _submit_movement_plan",
        1,
    )[0]
    submission = executor.split(
        "def _submit_movement_plan",
        1,
    )[1].split(
        "def _handle_successful_result",
        1,
    )[0]
    fresh_correction = executor.split(
        "def _poll_fresh_tag_target",
        1,
    )[1].split(
        "def _correct_frozen_base_goal",
        1,
    )[0]
    frozen_correction = executor.split(
        "def _correct_frozen_base_goal",
        1,
    )[1].split(
        "def _begin_timeout_cancellation_if_needed",
        1,
    )[0]

    assert "self._submit_movement_plan(" in initial
    assert "self._submit_movement_plan(" in fresh_correction
    assert "self._submit_movement_plan(" in frozen_correction
    assert "self.motion_planner.build_goal(" in submission
    assert "self._submit_goal(build_goal)" in submission
    assert "_build_absolute_base_goal" not in executor
    assert "self._submit_goal(" not in fresh_correction
    assert "self._submit_goal(" not in frozen_correction


def test_legacy_base_movement_behaviours_are_removed():
    legacy = (
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "move_command_action.py",
        "fault_detector_spot/navigation/behaviours/move_base/"
        "base_get_goal_tag.py",
        "fault_detector_spot/navigation/behaviours/move_base/"
        "base_move_relative_action.py",
        "fault_detector_spot/navigation/behaviours/move_base/"
        "base_move_to_tag_action.py",
        "fault_detector_spot/manipulation/behaviours/"
        "stand_up_action.py",
    )

    for relative_path in legacy:
        assert not (ROOT / relative_path).exists()
