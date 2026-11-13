"""Direct waypoint callers cannot bypass arm/height preparation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import py_trees
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatus

from fault_detector_spot.navigation.waypoint_navigation_executor import (
    WaypointNavigationExecutor, WaypointNavigationOutcome as Outcome,
)
from fault_detector_spot.navigation.base_movement_executor import BaseMovementOutcome as BaseOutcome
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementOutcome as ArmOutcome
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from test_base_movement_executor import ManualFuture, FakeGoalHandle, ManualClock
from fault_detector_spot.navigation.base_pose_source import BasePoseSample
from fault_detector_spot.navigation.base_goal_verifier import BaseGoalVerificationConfig


class Preparation:
    def __init__(self, outcomes, events, name):
        self.outcomes = outcomes
        self.events = events
        self.name = name
        self.active = False
        self.next = outcomes.RUNNING
        self.finish_outcome = outcomes.RUNNING
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

    def finish_walking(self):
        self.events.append("finish_walking")
        self.next = self.finish_outcome
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
    base.goal_verification_config = BaseGoalVerificationConfig()
    base.base_pose_source = SimpleNamespace(sample=lambda: BasePoseSample(0.0, 0.0, 0.0, 12.0 + clock.now))
    def stamp():
        value = 12.0 + clock.now
        return Time(sec=int(value), nanosec=round((value % 1) * 1e9))
    executor = WaypointNavigationExecutor(arm, base, client, stamp, clock)
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
    assert executor.active
    assert executor.navigate(pose).outcome is Outcome.BUSY
    if phase == "pending_nav2":
        send.set_result(handle)
    assert arm.cancel_calls == (1 if phase == "arm" else 0)
    assert base.cancel_calls == (1 if phase == "base" else 0)
    assert handle.cancel_calls == (1 if phase in ("pending_nav2", "nav2") else 0)
    if phase in ("pending_nav2", "nav2"):
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        settle_cancel(executor, clock)
    else:
        executor.poll()
    assert not executor.active


def test_goal_response_timeout_cancels_late_acceptance():
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    clock.now = 2.1
    assert executor.poll().outcome is Outcome.RUNNING
    assert executor.active
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
    if success:
        assert executor.poll().outcome is Outcome.SUCCESS
        assert executor.execution_idle
        assert not base.active
        assert handle.cancel_calls == 0
        client.send_goal_async.assert_called_once()
    else:
        assert executor.poll().outcome is Outcome.RUNNING
        settle_cancel(executor, clock)
    assert "finish_walking" not in events
    assert not executor.active


@pytest.mark.parametrize("feedback", [None, "stale"])
def test_nav2_success_does_not_require_hold_or_pose_feedback(feedback):
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    base.base_pose_source.sample = Mock(return_value=(
        None if feedback is None else BasePoseSample(0.0, 0.0, 0.0, 0.0)
    ))
    base.finish_walking = Mock(side_effect=AssertionError("No hold after Nav2 success"))
    result = ManualFuture()
    send.set_result(FakeGoalHandle(result))
    executor._receive_velocity(Twist())
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert executor.poll().outcome is Outcome.SUCCESS
    assert executor.execution_idle
    base.finish_walking.assert_not_called()
    base.base_pose_source.sample.assert_not_called()


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
    assert executor.poll().outcome is Outcome.RUNNING
    assert executor.active
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


def settle_cancel(executor, clock):
    assert executor.poll().outcome is Outcome.RUNNING
    for delta in (0.31, 0.1, 0.6):
        clock.now += delta
        update = executor.poll()
    assert update.outcome is Outcome.FAILURE
    return update


@pytest.mark.parametrize("external_cancel", [False, True])
def test_retry_permission_requires_confirmed_failure_stop(external_cancel):
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    send.set_result(FakeGoalHandle(result))
    executor.poll()
    if external_cancel:
        executor.cancel()
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    first = executor.poll()
    assert first.outcome is Outcome.RUNNING
    assert not first.retry_safe
    stopped = settle_cancel(executor, clock)
    assert stopped.retry_safe is (not external_cancel)


def test_cancel_without_result_or_fresh_pose_keeps_ownership():
    executor, arm, base, client, send, clock, pose, events = rig()
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    send.set_result(FakeGoalHandle(result))
    executor.poll()
    executor.cancel()
    clock.now = 10.0
    assert executor.poll().outcome is Outcome.RUNNING
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    base.base_pose_source.sample = lambda: None
    assert executor.poll().outcome is Outcome.RUNNING
    assert executor.active
    assert executor.navigate(pose).outcome is Outcome.BUSY


def test_cancel_monitor_finishes_without_further_bt_ticks():
    executor, arm, base, client, send, clock, pose, events = rig()
    timer = SimpleNamespace(cancel=Mock(), reset=Mock())
    def create_timer(_period, callback, **_options):
        timer.callback = callback
        return timer
    executor._node = SimpleNamespace(create_timer=create_timer)
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    send.set_result(FakeGoalHandle(result))
    executor.poll()
    executor.cancel()
    timer.callback()
    assert executor.active
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    timer.callback()
    for value in (0.31, 0.41, 1.01):
        clock.now = value
        timer.callback()
    assert not executor.active
    timer.cancel.assert_called_once()


def attach_monitor(executor):
    timer = SimpleNamespace(cancel=Mock(), reset=Mock())

    def create_timer(_period, callback, **options):
        from rclpy.clock import ClockType
        assert options["clock"].clock_type is ClockType.STEADY_TIME
        timer.callback = callback
        return timer

    executor._node = SimpleNamespace(create_timer=create_timer, destroy_timer=Mock())
    return timer


@pytest.mark.parametrize("state", [ArmStowState.DEPLOYED, ArmStowState.UNKNOWN, None])
@pytest.mark.parametrize("accepted", [False, True])
def test_stow_guard_cancels_without_bt_ticks_and_preserves_failure(state, accepted):
    executor, arm, base, client, send, clock, pose, events = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    handle = FakeGoalHandle(result)
    if accepted:
        send.set_result(handle)
        executor.poll()
    arm.state = state
    timer.callback()
    assert executor.cancelling
    if not accepted:
        send.set_result(handle)
    assert handle.cancel_calls == 1
    assert executor.active
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    timer.callback()
    for value in (0.31, 0.41, 1.01):
        clock.now = value
        timer.callback()
    assert not executor.active
    assert executor.poll().outcome is Outcome.FAILURE
    assert "Stowed-arm feedback lost" in executor.poll().detail
    timer.cancel.assert_called_once()
    # A new request must not inherit the preceding terminal failure.
    arm.state = ArmStowState.STOWED
    client.send_goal_async.side_effect = lambda _goal: ManualFuture()
    assert executor.navigate(pose).outcome is Outcome.RUNNING
    assert executor.poll().outcome is Outcome.RUNNING


def test_goal_response_watchdog_runs_without_bt_ticks():
    executor, arm, base, client, send, clock, pose, events = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    timer.callback()
    assert not executor.cancelling
    clock.now = 2.1
    timer.callback()
    assert executor.cancelling
    handle = FakeGoalHandle(ManualFuture())
    send.set_result(handle)
    assert handle.cancel_calls == 1


def test_navigation_monitor_stops_on_success_and_shutdown():
    executor, arm, base, client, send, clock, pose, events = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    handle = FakeGoalHandle(result)
    send.set_result(handle)
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert executor.poll().outcome is Outcome.SUCCESS
    timer.cancel.assert_called_once()
    arm.state = ArmStowState.DEPLOYED
    timer.callback()
    assert handle.cancel_calls == 0
    executor.shutdown()
    executor._node.destroy_timer.assert_called_once_with(timer)
    timer.callback()
    assert executor.navigate(pose).outcome is Outcome.FAILURE


def test_guard_error_cancels_and_shutdown_disables_queued_monitor_callback():
    executor, arm, base, client, send, clock, pose, events = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    handle = FakeGoalHandle(ManualFuture())
    send.set_result(handle)
    arm.arm_state_source.stow_state = Mock(side_effect=RuntimeError("feedback unavailable"))
    timer.callback()
    assert executor.cancelling
    assert handle.cancel_calls == 1
    assert executor.poll().outcome is Outcome.RUNNING
    assert "feedback unavailable" in executor.poll().detail
    executor.shutdown()
    assert not executor.execution_idle
    executor._node.destroy_timer.assert_called_once_with(timer)
    timer.callback()
    assert handle.cancel_calls == 1
    assert executor._monitor_timer is None


def completed_nav2():
    value = rig()
    executor, arm, base, client, send, clock, pose, events = value
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    send.set_result(FakeGoalHandle(result))
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    return value


def test_success_stops_forwarding_navigation_velocity_without_sending_hold():
    from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources

    executor, _, base, client, _, clock, pose, events = completed_nav2()
    resources = RobotCommandResources(monotonic_clock=clock)
    resources._waypoint_executor = executor
    resources._velocity_publisher = Mock()
    resources._velocity_publisher.publish.side_effect = executor._receive_velocity
    resources._forward_navigation_velocity(Twist())
    assert executor.poll().outcome is Outcome.SUCCESS
    resources._forward_navigation_velocity(Twist())
    resources._velocity_publisher.publish.assert_called_once()
    assert "finish_walking" not in events
    assert executor.execution_idle


def test_cancel_after_success_does_not_start_base_completion():
    executor, _, base, _, send, clock, _, events = completed_nav2()
    assert executor.poll().outcome is Outcome.SUCCESS
    executor.cancel()
    assert base.cancel_calls == 0
    assert send.result().cancel_calls == 0
    assert executor.execution_idle
    assert "finish_walking" not in events


def test_velocity_resuming_during_cancellation_restarts_stationary_confirmation():
    executor, _, _, _, _, clock, _, events = completed_nav2()
    executor.cancel()
    for value in (0.31, 0.41):
        clock.now = value
        assert executor.poll().outcome is Outcome.RUNNING
    clock.now = 0.8
    executor._receive_velocity(Twist())
    # No poll occurs during this new command. Its receipt still invalidates the
    # previous stable window, even when the next poll finds the stream quiet.
    for value in (1.2, 1.31):
        clock.now = value
        assert executor.poll().outcome is Outcome.RUNNING
        assert executor.active
    clock.now = 1.92
    assert executor.poll().outcome is Outcome.FAILURE
    assert not executor.active
    assert "finish_walking" not in events


def test_stowed_arm_guard_runs_until_result_is_consumed_without_bt_ticks():
    executor, arm, base, _, _, _, _, events = completed_nav2()
    timer = attach_monitor(executor)
    executor._start_monitor()
    arm.state = ArmStowState.DEPLOYED
    timer.callback()
    assert executor.cancelling
    assert "Stowed-arm feedback lost" in executor._cancel_detail
    assert base.cancel_calls == 0
    assert "finish_walking" not in events


def test_velocity_monitor_observes_driver_topic_and_is_destroyed_once():
    _, arm, base, client, _, clock, _, _ = rig()
    node = Mock()
    executor = WaypointNavigationExecutor(
        arm, base, client, lambda: Time(sec=12), clock, node=node,
    )
    message_type, topic, callback, depth = node.create_subscription.call_args.args
    assert (message_type, topic, depth) == (Twist, "/cmd_vel", 1)
    clock.now = 0.25
    callback(Twist())
    assert executor._last_velocity_at == 0.25
    executor.shutdown()
    executor.shutdown()
    node.destroy_subscription.assert_called_once_with(node.create_subscription.return_value)


@pytest.mark.parametrize("quiet", [0.0, float("nan"), float("inf")])
def test_invalid_velocity_quiet_interval_is_rejected(quiet):
    with pytest.raises(ValueError, match="Velocity quiet interval"):
        WaypointNavigationExecutor(
            Mock(), Mock(), Mock(), Mock(), velocity_quiet_sec=quiet,
        )


@pytest.mark.parametrize("pending_cancel", [False, True])
def test_shutdown_navigation_does_not_cancel_late_acceptance_or_continue_monitoring(pending_cancel):
    executor, arm, base, client, send, clock, pose, _ = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    if pending_cancel:
        executor.request_cancel()
        executor.cancel()
    executor.begin_shutdown()
    executor.begin_shutdown()
    assert executor.navigate(pose).outcome is Outcome.FAILURE
    assert not executor.poll_shutdown()[0]
    executor.shutdown()
    executor._node.destroy_timer.assert_called_once_with(timer)
    client.destroy.assert_called_once()

    result = ManualFuture()
    handle = FakeGoalHandle(result)
    send.set_result(handle)
    executor.cancel()
    executor.request_cancel()
    assert handle.cancel_calls == 0
    assert not executor.poll_shutdown()[0]
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    timer.callback()
    assert not executor.poll_shutdown()[0]
    for value in (0.31, 0.41, 1.01):
        clock.now = value
        timer.callback()
    assert not executor.poll_shutdown()[0]
    executor.shutdown()
    executor.shutdown()
    client.destroy.assert_called_once()
    executor._node.destroy_timer.assert_called_once_with(timer)


@pytest.mark.parametrize("missing", ["acceptance", "result", "physical_feedback"])
def test_shutdown_timeout_never_certifies_missing_nav2_evidence(missing):
    executor, arm, base, client, send, clock, pose, _ = rig()
    timer = attach_monitor(executor)
    prepare(executor, arm, base, pose)
    result = ManualFuture()
    if missing != "acceptance":
        send.set_result(FakeGoalHandle(result))
    if missing == "physical_feedback":
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        base.base_pose_source.sample = lambda: None
    executor.begin_shutdown()
    for value in (0.0, 1.0, 10.0):
        clock.now = value
        timer.callback()
        assert not executor.poll_shutdown()[0]
    executor.shutdown(stop_unconfirmed=True)
    executor.shutdown(stop_unconfirmed=True)
    assert not executor.execution_idle
    client.destroy.assert_called_once()


def test_shutdown_retries_failed_nav2_cleanup_and_releases_independent_handles():
    executor, _, _, client, _, _, _, _ = rig()
    timer = attach_monitor(executor)
    executor._monitor_timer = timer
    subscription = object()
    executor._velocity_subscription = subscription
    executor._node.destroy_timer.side_effect = [RuntimeError("timer cleanup failed"), None]
    executor._node.destroy_subscription = Mock()
    with pytest.raises(RuntimeError, match="timer cleanup failed"):
        executor.shutdown()
    executor._node.destroy_subscription.assert_called_once_with(subscription)
    client.destroy.assert_called_once()
    assert executor._monitor_timer is timer
    executor.shutdown()
    executor.shutdown()
    assert executor._node.destroy_timer.call_count == 2
    executor._node.destroy_subscription.assert_called_once()
    client.destroy.assert_called_once()


def test_shutdown_navigation_does_not_poll_preparation_owned_by_coordinator():
    executor, arm, _, _, _, _, pose, _ = rig()
    executor.navigate(pose)
    executor.begin_shutdown()
    arm.active = True
    arm._shutdown_started = True
    arm.poll = Mock(side_effect=AssertionError("Coordinator owns preparation polling"))
    assert not executor.poll_shutdown()[0]
    arm.poll.assert_not_called()
    arm.active = False
    assert not executor.poll_shutdown()[0]
