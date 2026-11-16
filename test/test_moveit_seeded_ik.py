"""Prefer a current-state-seeded IK goal without constraining the path."""

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import pytest

from fault_detector_spot.manipulation.moveit_arm_planner import ARM_JOINT_NAMES, MoveItPlanOutcome
from test_moveit_collision_planning import Rig, READ, APPLY, PLAN

IK = "/compute_ik"


@pytest.fixture
def rig():
    value = Rig()
    value.node.clients[IK].available = True
    yield value
    value.planner.destroy()


def begin(rig):
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    rig.read()
    rig.applied()
    assert rig.node.events == [READ, APPLY, IK]


def response(code=MoveItErrorCodes.SUCCESS):
    result = GetPositionIK.Response()
    result.error_code.val = code
    result.solution.joint_state.name = list(reversed(ARM_JOINT_NAMES)) + ["gripper"]
    result.solution.joint_state.position = [0.1, 0.2, 0.3, 0.4, -1.5, 0.6, 0.0]
    return result


def test_seeded_goal_keeps_pose_collision_and_scene_checks(rig):
    begin(rig)
    request = rig.node.clients[IK].requests[0].ik_request
    assert request.robot_state.is_diff and not request.robot_state.joint_state.name
    assert request.group_name == "arm" and request.ik_link_name == "hand"
    assert request.pose_stamped == rig.target
    assert request.avoid_collisions
    assert request.timeout.sec == request.timeout.nanosec == 0
    result = response()
    assert rig.reply(IK, result).outcome is MoveItPlanOutcome.RUNNING
    motion = rig.node.clients[PLAN].requests[0].motion_plan_request
    goal, = motion.goal_constraints
    assert [joint.joint_name for joint in goal.joint_constraints] == list(ARM_JOINT_NAMES)
    values = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
    assert [joint.position for joint in goal.joint_constraints] == [values[name] for name in ARM_JOINT_NAMES]
    assert goal.position_constraints and goal.orientation_constraints
    assert not motion.path_constraints.orientation_constraints
    assert not motion.path_constraints.joint_constraints
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.planner._pending_joint_goal is None
    begin_again = rig.planner.start(rig.target)
    assert begin_again.outcome is MoveItPlanOutcome.RUNNING
    rig.read()
    assert len(rig.node.clients[IK].requests) == 2


def test_no_exact_ik_retains_original_pose_tolerance_search(rig):
    begin(rig)
    rig.reply(IK, response(MoveItErrorCodes.NO_IK_SOLUTION))
    motion = rig.node.clients[PLAN].requests[0].motion_plan_request
    assert not motion.goal_constraints[0].joint_constraints
    assert motion.goal_constraints[0].orientation_constraints


@pytest.mark.parametrize("problem", ["missing", "nan", "duplicate", "length"])
def test_invalid_ik_never_submits_motion_plan(rig, problem):
    begin(rig)
    result = response()
    state = result.solution.joint_state
    if problem == "missing":
        state.name[0] = "unknown"
    elif problem == "nan":
        state.position[0] = float("nan")
    elif problem == "duplicate":
        state.name[-1] = state.name[0]
    else:
        state.position.pop()
    assert rig.reply(IK, result).outcome is MoveItPlanOutcome.ERROR
    assert not rig.node.clients[PLAN].requests


def test_policy_change_during_ik_invalidates_goal(rig):
    begin(rig)
    rig.toggle()
    assert rig.reply(IK, response()).outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.clients[PLAN].requests


def test_cancel_during_ik_does_not_submit_plan(rig):
    begin(rig)
    future = rig.node.clients[IK].future
    rig.planner.cancel()
    future.set_result(response())
    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert not rig.node.clients[PLAN].requests


def test_cartesian_requests_skip_ik(rig):
    rig.planner.start_cartesian(rig.target)
    rig.read()
    rig.applied()
    assert not rig.node.clients[IK].requests
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
