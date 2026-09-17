"""Offline checks for MoveIt planning failure diagnostics and Cartesian paths."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import pytest
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from fault_detector_spot.manipulation.moveit_arm_planner import (
    ARM_JOINT_NAMES,
    DEFAULT_CARTESIAN_JUMP_THRESHOLD,
    DEFAULT_CARTESIAN_MAX_STEP_M,
    DEFAULT_CARTESIAN_MIN_FRACTION,
    DEFAULT_POSITION_TOLERANCE_M,
    MoveItArmPlanner,
    MoveItPlanOutcome,
)


def planner_shell():
    planner = object.__new__(MoveItArmPlanner)
    planner.planning_frame = "body"
    planner.group_name = "arm"
    planner.end_effector_link = "hand"
    planner.cartesian_max_step_m = DEFAULT_CARTESIAN_MAX_STEP_M
    planner.cartesian_jump_threshold = DEFAULT_CARTESIAN_JUMP_THRESHOLD
    planner.cartesian_min_fraction = DEFAULT_CARTESIAN_MIN_FRACTION
    planner.velocity_scaling = 1.0
    planner.acceleration_scaling = 1.0
    planner.response_timeout_sec = 7.0
    planner._started_at = 0.0
    planner._planning_mode = None
    planner._future = None
    planner._logger = SimpleNamespace(info=lambda *_args, **_kwargs: None)
    return planner


def valid_trajectory():
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ARM_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(ARM_JOINT_NAMES)
    point.time_from_start.sec = 1
    trajectory.points = [point]
    return trajectory


def test_default_position_tolerance_is_two_millimeters():
    assert DEFAULT_POSITION_TOLERANCE_M == pytest.approx(0.002)


def test_cartesian_defaults_require_dense_nearly_complete_path():
    assert DEFAULT_CARTESIAN_MAX_STEP_M == pytest.approx(0.002)
    assert DEFAULT_CARTESIAN_JUMP_THRESHOLD == pytest.approx(2.0)
    assert DEFAULT_CARTESIAN_MIN_FRACTION == pytest.approx(0.999)


def test_cartesian_request_is_straight_collision_checked_hand_path():
    planner = planner_shell()
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.42
    target.pose.orientation.w = 1.0

    request = planner._build_cartesian_request(target)

    assert request.header.frame_id == "body"
    assert request.start_state.is_diff
    assert request.group_name == "arm"
    assert request.link_name == "hand"
    assert len(request.waypoints) == 1
    assert request.waypoints[0].position.x == pytest.approx(0.42)
    assert request.max_step == pytest.approx(0.002)
    assert request.jump_threshold == pytest.approx(2.0)
    assert request.avoid_collisions


def test_complete_cartesian_response_returns_joint_trajectory():
    planner = planner_shell()
    planner._planning_mode = "cartesian"
    planner._future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(
            error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
            fraction=1.0,
            solution=SimpleNamespace(joint_trajectory=valid_trajectory()),
        ),
    )

    update = planner.poll()

    assert update.outcome is MoveItPlanOutcome.SUCCESS
    assert update.trajectory is not None
    assert "fraction 1.000000" in update.detail
    assert not planner.active


def test_partial_cartesian_response_is_rejected():
    planner = planner_shell()
    planner._planning_mode = "cartesian"
    planner._future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(
            error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
            fraction=0.95,
            solution=SimpleNamespace(joint_trajectory=JointTrajectory()),
        ),
    )

    update = planner.poll()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert "incomplete" in update.detail
    assert "0.950000" in update.detail
    assert not planner.active


@pytest.mark.parametrize("code, name", [
    (-27, "GOAL_STATE_INVALID"),
    (-26, "START_STATE_INVALID"),
    (-31, "NO_IK_SOLUTION"),
    (-12345, "UNKNOWN"),
])
def test_failed_plan_reports_symbolic_code_and_releases_future(code, name):
    planner = planner_shell()
    planner._planning_mode = "motion"
    planner._future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(motion_plan_response=SimpleNamespace(
            error_code=MoveItErrorCodes(val=code),
        )),
    )

    update = planner.poll()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert name in update.detail
    assert str(code) in update.detail
    assert not planner.active
    if code == -27:
        assert "check move_group logs" in update.detail
