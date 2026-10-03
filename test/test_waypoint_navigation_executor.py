"""Direct waypoint callers cannot bypass arm/height preparation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import py_trees
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from fault_detector_spot.navigation.waypoint_navigation_executor import (
    WaypointNavigationExecutor, WaypointNavigationOutcome as Outcome,
)
from fault_detector_spot.navigation.base_movement_executor import BaseMovementOutcome as BaseOutcome
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementOutcome as ArmOutcome
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from test_base_movement_executor import ManualFuture, FakeGoalHandle, ManualClock


class Preparation:
    def __init__(self, outcomes, events, name):
        self.outcomes = outcomes
        self.events = events
        self.name = name
        self.active = False
        self.next = outcomes.RUNNING
        self.cancel_calls = 0
        self.state = ArmStowState.STOWED
        self.arm_state_source = SimpleNamespace(stow_state=lambda: self.state)

    def _update(self):
        self.active = self.next is self.outcomes.RUNNING
        return SimpleNamespace(outcome=self.next, detail=self.next.value)

    def stow(self):
        self.events.append("stow")
        return self._update()

    def prepare_for_navigation(self):
        self.events.append("height")
        return self._update()

    def poll(self):
        self.events.append(self.name + "_poll")
        return self._update()

    def cancel(self):
        self.cancel_calls += 1
        self.active = False


def rig():
    events = []
    arm = Preparation(ArmOutcome, events, "arm")
    base = Preparation(BaseOutcome, events, "base")
    client = Mock()
    send = ManualFuture()
    def dispatch(goal):
        events.append("nav2")
        return send
    client.send_goal_async.side_effect = dispatch
    client.wait_for_server.return_value = True
    clock = ManualClock()
    executor = WaypointNavigationExecutor(arm, base, client, lambda: Time(sec=12), clock)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    return executor, arm, base, client, send, clock, pose, events


def prepare(executor, arm, base, pose):
    arm.next = ArmOutcome.SUCCESS
    base.next = BaseOutcome.SUCCESS
    return executor.navigate(pose)


def test_direct_caller_waits_for_stow_then_height_before_dispatch():
    executor, arm, base, client, send, clock, pose, events = rig()
    pose.pose.position.x = 2.0
    assert executor.navigate(pose).outcome is Outcome.RUNNING
    pose.pose.position.x = 99.0
    assert events == ["stow"]
    executor.poll()
    client.send_goal_async.assert_not_called()
    arm.next = ArmOutcome.SUCCESS
    executor.poll()
    assert events[-2:] == ["arm_poll", "height"]
    client.send_goal_async.assert_not_called()
    base.next = BaseOutcome.SUCCESS
    executor.poll()
    assert events[-2:] == ["base_poll", "nav2"]
    sent = client.send_goal_async.call_args.args[0]
    assert sent.pose.pose.position.x == 2.0
    assert sent.pose.header.stamp.sec == 12


@pytest.mark.parametrize("phase", ["arm", "base"])
def test_preparation_failure_blocks_nav2(phase):
    executor, arm, base, client, send, clock, pose, events = rig()
    arm.next = ArmOutcome.MOTION_FAILED if phase == "arm" else ArmOutcome.SUCCESS
    base.next = BaseOutcome.HEIGHT_RESET_TIMEOUT
    assert executor.navigate(pose).outcome is Outcome.FAILURE
    client.send_goal_async.assert_not_called()
    assert not executor.active
    if phase == "arm":
        assert "height" not in events


@pytest.mark.parametrize("state", [None, ArmStowState.UNKNOWN, ArmStowState.DEPLOYED])
def test_arm_rechecked_after_height_preparation(state):
    executor, arm, base, client, send, clock, pose, events = rig()
    arm.next = ArmOutcome.SUCCESS
    executor.navigate(pose)
    arm.state = state
    base.next = BaseOutcome.SUCCESS
    assert executor.poll().outcome is Outcome.FAILURE
    client.send_goal_async.assert_not_called()


@pytest.mark.parametrize("phase", ["arm", "base", "pending_nav2", "nav2"])
def test_cancellation_reaches_owned_operation_in_every_phase(phase):
    executor, arm, base, client, send, clock, pose, events = rig()
    result = ManualFuture()
    handle = FakeGoalHandle(result)
    if phase != "arm":
        arm.next = ArmOutcome.SUCCESS
    if phase in ("pending_nav2", "nav2"):
        base.next = BaseOutcome.SUCCESS
    executor.navigate(pose)
    if phase == "nav2":
        send.set_result(handle)
        executor.poll()
    executor.cancel()
    assert not executor.active
    if phase == "pending_nav2":
        send.set_result(handle)
    assert arm.cancel_calls == (1 if phase == "arm" else 0)
    assert base.cancel_calls == (1 if phase == "base" else 0)
    assert handle.cancel_calls == (1 if phase in ("pending_nav2", "nav2") else 0)


def test_goal_response_timeout_cancels_late_acceptance():
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    clock.now = 2.1
    assert executor.poll().outcome is Outcome.FAILURE
    handle = FakeGoalHandle(ManualFuture())
    send.set_result(handle)
    assert handle.cancel_calls == 1


@pytest.mark.parametrize("success", [False, True])
def test_nav2_result_is_propagated(success):
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    handle = FakeGoalHandle(result)
    send.set_result(handle)
    assert executor.poll().outcome is Outcome.RUNNING
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED))
    assert executor.poll().outcome is (Outcome.SUCCESS if success else Outcome.FAILURE)
    assert not executor.active
    if success:
        assert handle.cancel_calls == 0


def test_rejection_fails_instead_of_waiting_forever():
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    send.set_result(FakeGoalHandle(ManualFuture(), accepted=False))
    assert executor.poll().outcome is Outcome.FAILURE


def test_arm_deployment_during_navigation_cancels_goal():
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    handle = FakeGoalHandle(ManualFuture())
    send.set_result(handle)
    executor.poll()
    arm.state = ArmStowState.DEPLOYED
    assert executor.poll().outcome is Outcome.FAILURE
    assert handle.cancel_calls == 1


def test_busy_shared_executor_is_not_preempted():
    executor, arm, base, client, send, clock, pose, events = rig()
    arm.active = True
    assert executor.navigate(pose).outcome is Outcome.BUSY
    assert arm.cancel_calls == 0
    assert events == []


def test_unavailable_server_does_not_start_preparation():
    executor, arm, base, client, send, clock, pose, events = rig()
    client.wait_for_server.return_value = False
    assert executor.navigate(pose).outcome is Outcome.FAILURE
    assert events == []


def test_behaviour_direct_invocation_uses_prepared_executor():
    from fault_detector_spot.navigation.behaviours.navigate_to_goal_pose import NavigateToGoalPose
    executor, arm, base, client, send, clock, pose, events = rig()
    behaviour = NavigateToGoalPose()
    behaviour.executor = executor
    behaviour._last_command = lambda: SimpleNamespace(goal_pose=pose)
    assert behaviour.update() is py_trees.common.Status.RUNNING
    assert events == ["stow"]
    behaviour.terminate(py_trees.common.Status.INVALID)
    assert arm.cancel_calls == 1


def test_waypoint_tree_only_resolves_then_calls_prepared_navigation(monkeypatch):
    from fault_detector_spot.application.behaviour_tree import runner
    resources = object()
    monkeypatch.setattr(runner, "get_helper_container", lambda _: SimpleNamespace(robot_command_resources=resources))
    tree = runner.build_navigate_to_goal_pose_tree(object())
    assert [child.name for child in tree.children] == ["SetWaypointAsGoal", "NavigateToGoalPose"]
    assert tree.children[1].robot_command_resources is resources
