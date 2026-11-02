"""Stop proof survives lifecycle resets and requires fresh physical telemetry."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus

from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from fault_detector_spot.navigation.base_movement_executor import BaseMovementExecutor
from fault_detector_spot.navigation.base_pose_source import BasePoseSample
from test_base_movement_executor import FakeActionClient, FakeGoalHandle, ManualClock


def base_executor():
    clock = ManualClock()
    source = SimpleNamespace(value=None)
    source.sample = lambda: source.value
    sent = Future()
    client = FakeActionClient(sent)
    owner = BaseMovementExecutor(object(), action_client=client,
                                 monotonic_clock=clock, ros_time_sec=clock)
    owner.base_pose_source = source
    owner._build_sit_goal = lambda: object()
    return owner, sent, clock, source


def stationary_sample(owner, clock, source, at, x=2.0, stamp=None):
    clock.now = at
    source.value = BasePoseSample(x, 3.0, 0.4, at if stamp is None else stamp)
    return owner.poll_stop_confirmation()[0]


def test_late_goal_delivery_cannot_be_hidden_by_operation_reset():
    owner, sent, clock, source = base_executor()
    owner.sit()
    owner.begin_stop_confirmation()
    assert not owner.execution_idle
    result = Future()
    handle = FakeGoalHandle(result)
    sent.set_result(handle)
    assert handle.cancel_calls == 1
    assert not owner.execution_idle
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not owner.active
    assert not owner.execution_idle  # Action cancellation alone is insufficient.
    assert not stationary_sample(owner, clock, source, 0.1)
    assert stationary_sample(owner, clock, source, 0.7)
    assert owner.execution_idle
    assert owner.action_client.send_calls == 1


@pytest.mark.parametrize("failure", ["send", "result", "nonterminal", "missing_handle"])
def test_failed_or_nonterminal_action_evidence_never_confirms_idle(failure):
    owner, sent, clock, source = base_executor()
    owner.sit()
    owner.begin_stop_confirmation()
    if failure == "send":
        sent.set_exception(RuntimeError("lost goal response"))
    elif failure == "missing_handle":
        sent.set_result(None)
    else:
        result = Future()
        sent.set_result(FakeGoalHandle(result))
        if failure == "result":
            result.set_exception(RuntimeError("lost action result"))
        else:
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_EXECUTING))
    assert not stationary_sample(owner, clock, source, 0.1)
    assert not stationary_sample(owner, clock, source, 0.7)
    assert not owner.execution_idle


def test_stationary_pose_before_stop_or_repeated_timestamp_is_not_new_stop_proof():
    owner, _, clock, source = base_executor()
    clock.now = 1.0
    owner.begin_stop_confirmation()
    assert not stationary_sample(owner, clock, source, 1.1, stamp=1.0)
    assert not stationary_sample(owner, clock, source, 1.2)
    assert not stationary_sample(owner, clock, source, 1.6, stamp=1.2)
    assert not stationary_sample(owner, clock, source, 1.7, x=2.1)
    assert not stationary_sample(owner, clock, source, 2.0, x=2.1)
    assert stationary_sample(owner, clock, source, 2.3, x=2.1)


def test_stale_pose_cannot_confirm_stop_even_after_waiting():
    owner, _, clock, source = base_executor()
    owner.begin_stop_confirmation()
    assert not stationary_sample(owner, clock, source, 0.1)
    assert not stationary_sample(owner, clock, source, 4.0, stamp=0.2)


def test_direct_base_timeout_does_not_turn_action_terminal_into_physical_stop_proof():
    owner, sent, clock, source = base_executor()
    owner.sit()
    result = Future()
    sent.set_result(FakeGoalHandle(result))
    owner.poll()
    clock.now = owner.result_timeout_sec + 0.1
    owner.poll()
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    owner.poll()
    assert not owner.active
    assert not owner.execution_idle
    owner.begin_stop_confirmation()
    assert not stationary_sample(owner, clock, source, 31.0)
    assert stationary_sample(owner, clock, source, 31.6)
    assert owner.execution_idle


def test_arm_stop_unavailable_survives_reset_as_nonidle():
    owner = ArmMovementExecutor(object())
    result = owner.confirm_stop()
    assert result.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert not owner.active
    assert not owner.execution_idle
    # An unrelated lifecycle reset cannot erase failed stop confirmation.
    owner._reset_operation()
    assert not owner.execution_idle


def resource_harness(monkeypatch):
    clock = ManualClock()
    resources = RobotCommandResources(monotonic_clock=clock)
    arm = Mock(active=False, execution_idle=True)
    arm.goals_settled.return_value = True
    arm.confirm_stop.return_value = ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "measured arm stop")
    base = Mock(execution_idle=True)
    base.poll_stop_confirmation.return_value = (True, "measured base stop")
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    monkeypatch.setattr(resources, "get_arm_movement_executor", lambda node: arm)
    monkeypatch.setattr(resources, "get_base_movement_executor", lambda node: base)
    return resources, arm, base, clock


@pytest.mark.parametrize("unconfirmed_owner", ["arm", "base", "navigation"])
def test_resource_stop_requires_every_motion_owner(monkeypatch, unconfirmed_owner):
    resources, arm, base, _ = resource_harness(monkeypatch)
    if unconfirmed_owner == "arm":
        arm.confirm_stop.return_value = ArmMovementUpdate(ArmMovementOutcome.STOP_UNCONFIRMED, "stale arm")
    elif unconfirmed_owner == "base":
        base.poll_stop_confirmation.return_value = (False, "base still moving")
    else:
        resources._waypoint_executor = Mock(active=True)
        resources._waypoint_executor.poll.return_value = SimpleNamespace(detail="Nav2 result missing")
    resources.begin_stop_confirmation(object())
    assert resources.poll_stop_confirmation()[0] is False
    arm.cancel.assert_called_once()
    base.begin_stop_confirmation.assert_called_once()


def test_resource_retries_failed_arm_stop_without_flooding_stop_service(monkeypatch):
    resources, arm, base, clock = resource_harness(monkeypatch)
    arm.confirm_stop.side_effect = [
        ArmMovementUpdate(ArmMovementOutcome.STOP_UNCONFIRMED, "service unavailable"),
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "fresh stationary arm"),
    ]
    resources.begin_stop_confirmation(object())
    assert not resources.poll_stop_confirmation()[0]
    assert not resources.poll_stop_confirmation()[0]
    assert arm.confirm_stop.call_count == 1
    clock.now = 1.1
    assert resources.poll_stop_confirmation()[0]
    assert arm.confirm_stop.call_count == 2


def test_resource_idle_does_not_treat_inactive_uncertain_owner_as_stopped(monkeypatch):
    resources, arm, base, _ = resource_harness(monkeypatch)
    arm.execution_idle = False
    assert not resources.execution_idle()
    arm.execution_idle = True
    base.execution_idle = False
    assert not resources.execution_idle()


@pytest.mark.parametrize("failure", ["raises", "no_future", "no_handle"])
def test_nav2_ambiguous_submission_is_never_nonexecution_evidence(failure):
    from test_waypoint_navigation_executor import rig, prepare
    from fault_detector_spot.navigation.waypoint_navigation_executor import WaypointNavigationOutcome

    owner, arm, base, client, sent, clock, pose, _ = rig()
    if failure == "raises":
        client.send_goal_async.side_effect = RuntimeError("lost dispatch acknowledgement")
    elif failure == "no_future":
        client.send_goal_async.side_effect = lambda goal: None
    prepare(owner, arm, base, pose)
    if failure == "no_handle":
        sent.set_result(None)
    owner.cancel()
    clock.now = 10.0
    assert owner.poll().outcome is WaypointNavigationOutcome.RUNNING
    assert not owner.execution_idle
    assert owner.navigate(pose).outcome is WaypointNavigationOutcome.BUSY
    assert client.send_goal_async.call_count == 1
