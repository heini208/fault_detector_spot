"""Offline tests for measured base arrival and settling."""

from test_base_movement_executor import ReadyHeight

import math

import pytest

from fault_detector_spot.navigation.base_goal_verifier import (
    BaseGoalVerifier, BaseGoalVerificationConfig,
)


def verifier(target=(0, 0, 0)):
    return BaseGoalVerifier(target, BaseGoalVerificationConfig(), 0)


def test_success_requires_new_samples_and_settling():
    check = verifier()
    assert check.update((0, 0, 0), 10, 10, 0) is None
    assert check.update((0, 0, 0), 10, 10.5, 0.5) is None
    assert check.update((0, 0, 0), 10.6, 10.6, 0.6) is True


def test_goal_error_does_not_succeed_even_after_action_success():
    check = verifier()
    assert check.update((0.1, 0, 0), 10, 10, 0) is None
    assert check.update((0.1, 0, 0), 15, 15, 5) is False
    assert "0.1000 m" in check.detail


def test_settling_is_independent_of_goal_accuracy():
    check = verifier()

    assert check.update((0.1, 0, 0), 10, 10, 0) is None
    assert check.within_tolerance is False
    assert not check.settled

    assert check.update((0.1, 0, 0), 10.6, 10.6, 0.6) is None
    assert check.within_tolerance is False
    assert check.settled
    assert check.settled_stamp == pytest.approx(10.6)
    assert "base settled" in check.detail


def test_stale_pose_clears_independent_settled_state():
    check = verifier()

    check.update((0.1, 0, 0), 10, 10, 0)
    check.update((0.1, 0, 0), 10.6, 10.6, 0.6)
    assert check.settled

    assert check.update((0.1, 0, 0), 10.6, 12, 0.7) is None
    assert not check.settled
    assert check.settled_stamp is None
    assert check.within_tolerance is None


def test_yaw_error_is_wrapped():
    check = verifier((0, 0, math.pi - 0.01))
    assert check.update((0, 0, -math.pi + 0.01), 10, 10, 0) is None
    assert check.update((0, 0, -math.pi + 0.01), 11, 11, 1) is True


def test_motion_inside_goal_tolerance_restarts_settling():
    check = verifier()
    assert check.update((0, 0, 0), 10, 10, 0) is None
    assert check.update((0.02, 0, 0), 10.5, 10.5, 0.5) is None
    assert check.update((0.02, 0, 0), 11.1, 11.1, 1.1) is True


@pytest.mark.parametrize("pose,stamp", [(None, None), ((0, 0, 0), 1), ((float('nan'), 0, 0), 10)])
def test_invalid_or_stale_pose_cannot_complete(pose, stamp):
    check = verifier()
    assert check.update(pose, stamp, 10, 0) is None
    assert check.update(pose, stamp, 15, 5) is False


def test_leaving_tolerance_resets_settling():
    check = verifier()
    assert check.update((0, 0, 0), 10, 10, 0) is None
    assert check.update((0.2, 0, 0), 10.4, 10.4, 0.4) is None
    assert check.update((0, 0, 0), 10.6, 10.6, 0.6) is None
    assert check.update((0, 0, 0), 11.2, 11.2, 1.2) is True


@pytest.mark.parametrize("value", [0, -1, float('nan'), float('inf')])
def test_invalid_tolerance_rejected(value):
    with pytest.raises(ValueError):
        BaseGoalVerificationConfig(position_tolerance_m=value)


def test_executor_verifies_actual_goal_and_cancellation_during_settling():
    from types import SimpleNamespace
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from fault_detector_spot.navigation.base_motion_planner import (
        BaseMovementPlan,
    )
    from fault_detector_spot.navigation.base_movement_executor import (
        BaseMovementExecutor, BaseMovementOutcome,
    )
    from fault_detector_spot.navigation.posture_state_source import PostureState
    from test_base_movement_executor import (
        ManualClock, ManualFuture, FakeActionClient, FakeGoalHandle,
        FakePostureStateSource,
    )

    clock = ManualClock()
    send, result = ManualFuture(), ManualFuture()
    transform = TransformStamped()
    transform.header.stamp.sec = 10
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 1.0
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=SimpleNamespace(lookup_a_tform_b=lambda *a, **k: transform),
        action_client=FakeActionClient(send),
        posture_state_source=FakePostureStateSource(PostureState.STANDING),
        monotonic_clock=clock,
        ros_time_sec=lambda: 10 + clock.now,
    )
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    target.pose.position.x = 1.0
    profile = executor.walking_profiles.for_move()
    plan = BaseMovementPlan(target, 0.1, profile)
    executor.motion_planner.resolve_relative = lambda _: plan

    executor.relative(object())
    handle = FakeGoalHandle(result)
    send.set_result(handle)
    executor.poll()
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=True)))
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    assert executor.active
    clock.now = 0.6
    transform.header.stamp.nanosec = 600_000_000
    assert executor.poll().outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active

    executor.motion_planner.resolve_relative = lambda _: plan
    executor.relative(object())
    executor.poll()
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    executor.cancel()
    assert not executor.active
    assert executor._goal_verifier is None
    assert handle.cancel_calls == 0


def test_late_sample_cannot_succeed_after_deadline():
    check = verifier()
    check.update((0, 0, 0), 10, 10, 0)
    assert check.update((0, 0, 0), 16, 16, 6) is False


def test_position_alone_is_not_enough_when_yaw_is_wrong():
    check = verifier()
    assert check.update((0, 0, 0.2), 10, 10, 0) is None
    assert check.update((0, 0, 0.2), 11, 11, 1) is None
    assert check.update((0, 0, 0.2), 15, 15, 5) is False


def test_clock_rewind_restarts_settling():
    check = verifier()
    check.update((0, 0, 0), 10, 10, 0)
    assert check.update((0, 0, 0), 9, 9, 0.6) is None
    assert check.update((0, 0, 0), 9.1, 9.1, 0.7) is None


def test_slow_approach_continues_past_five_seconds_then_settles():
    check = BaseGoalVerifier(
        (1, 0, 0), BaseGoalVerificationConfig(), 0,
        motion_timeout_sec=30.0,
    )
    for second in range(11):
        assert check.update(
            (0.5 + second * 0.05, 0, 0), 100 + second,
            100 + second, second,
        ) is None
        assert not check.settled
    assert check.update((1, 0, 0), 110.6, 110.6, 10.6) is True


def test_approach_stall_times_out_after_last_progress():
    check = BaseGoalVerifier(
        (1, 0, 0), BaseGoalVerificationConfig(), 0,
        motion_timeout_sec=30.0,
    )
    check.update((0.5, 0, 0), 100, 100, 0)
    check.update((0.6, 0, 0), 102, 102, 2)
    assert check.update((0.6, 0, 0), 106.9, 106.9, 6.9) is None
    assert check.update((0.6, 0, 0), 107, 107, 7) is False


def test_progress_cannot_extend_hard_motion_deadline():
    check = BaseGoalVerifier(
        (10, 0, 0), BaseGoalVerificationConfig(), 0,
        motion_timeout_sec=8.0,
    )
    for second in range(8):
        assert check.update(
            (second * 0.05, 0, 0), 100 + second, 100 + second, second,
        ) is None
    assert check.update((0.4, 0, 0), 108, 108, 8) is False


def test_late_progress_cannot_revive_expired_verification():
    check = BaseGoalVerifier(
        (1, 0, 0), BaseGoalVerificationConfig(), 0,
        motion_timeout_sec=30.0,
    )
    check.update((0.5, 0, 0), 100, 100, 0)
    assert check.update((0.8, 0, 0), 106, 106, 6) is False
