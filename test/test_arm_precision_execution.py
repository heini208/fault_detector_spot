"""Opt-in precision uses the existing guarded movement and verification owner."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget
from test_arm_movement_executor import FakeGoalHandle, ManualFuture, transform
from test_guarded_arm_movement_executor import ImmediateBaseline
from test_guarded_planning_frame_freshness import rig, stamped_transform


def precision_rig(monkeypatch, *, relative=False, ignore=False, tolerance_m=0.005):
    value = rig(monkeypatch, ignore_environment_collisions=ignore)
    value.state.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round((10 + value.clock.now) * 1e9)),
    ))

    class Baseline(ImmediateBaseline):
        def start(self):
            result = super().start()
            return replace(result, baseline=replace(result.baseline, last_received_at=value.clock.now))

    value.executor.guarded_probe_execution.force_baseline_sampler = Baseline()
    value.resolve_calls = []

    def resolve(*_args):
        value.resolve_calls.append(deepcopy(value.target))
        return ResolvedProbeTarget(deepcopy(value.target), "hand")

    monkeypatch.setattr(value.executor.probe_motion_planner,
                        "resolve_relative" if relative else "resolve_tag", resolve)
    value.command = SimpleNamespace(
        precision_mode=True, tag_position_tolerance_m=tolerance_m,
        precision_orientation_tolerance_rad=math.radians(2.0),
        arm_speed_scale=1.0, ignore_environment_collisions=ignore,
    )
    start = value.executor.relative if relative else value.executor.tag_probe
    assert start(value.command).outcome is ArmMovementOutcome.RUNNING
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(value.client.sent_goals) == 1
    return value


def finish_motion(value):
    result = ManualFuture()
    value.client.send_future.set_result(FakeGoalHandle(result_future=result))
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=True)))
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    value.client.send_future = ManualFuture()


def feedback(value, stamp, *, position=1.0, yaw=0.0, acquisition=None):
    value.clock.now = stamp
    actual_stamp = 10 + (stamp if acquisition is None else acquisition)
    pose = transform("odom", "hand", x=position, yaw=yaw)
    pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(round(actual_stamp * 1e9), 10**9)
    value.tf.transforms[("odom", "hand")] = pose
    value.tf.transforms[("body", "odom")] = stamped_transform(stamp=10 + stamp)
    return value.executor.poll()


def stable_feedback(value, *, position=1.0, yaw=0.0):
    start = value.clock.now + 0.5
    update = None
    for time in (start, start + 0.15, start + 0.30):
        update = feedback(value, time, position=position, yaw=yaw)
    return update


@pytest.mark.parametrize("relative", [False, True])
@pytest.mark.parametrize("ignore", [False, True])
def test_precision_freezes_target_then_reverifies_every_slow_correction(monkeypatch, relative, ignore):
    value = precision_rig(monkeypatch, relative=relative, ignore=ignore)
    finish_motion(value)
    value.target.pose.position.x = 2.0  # Neither a changed tag nor relative origin retargets correction.
    assert stable_feedback(value, position=0.988).outcome is ArmMovementOutcome.RUNNING
    assert len(value.starts) == 2
    assert value.executor.guarded_probe_execution._plan.target_hand.pose.position.x == pytest.approx(1.0)
    assert value.executor.guarded_probe_execution._plan.linear_speed_mps <= 0.025
    assert value.executor._planning_frame_tolerances == pytest.approx((0.001, math.radians(0.2)))
    value.executor.poll()  # Fresh collision planning and baseline finish before dispatch.
    assert len(value.client.sent_goals) == 2
    finish_motion(value)
    assert stable_feedback(value, position=0.993).outcome is ArmMovementOutcome.RUNNING
    value.executor.poll()
    assert len(value.client.sent_goals) == 3
    finish_motion(value)
    update = stable_feedback(value, position=0.997)
    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert "2 slow correction" in update.detail
    assert len(value.resolve_calls) == 1
    assert value.stop.requests == []


def test_unimproved_return_fails_without_another_correction(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    stable_feedback(value, position=0.990)
    value.executor.poll()
    finish_motion(value)
    update = stable_feedback(value, position=0.9905)
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert "did not improve enough" in update.detail
    assert len(value.client.sent_goals) == 2
    assert value.stop.requests == []


@pytest.mark.parametrize("relative", [False, True])
@pytest.mark.parametrize("error_m", [0.02646, 0.050, 0.200])
def test_large_precision_errors_allow_correction_and_keep_requested_accuracy(monkeypatch, relative, error_m):
    value = precision_rig(monkeypatch, relative=relative, tolerance_m=0.010)
    finish_motion(value)
    update = stable_feedback(value, position=1.0 - error_m)
    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(value.starts) == 2
    assert value.executor.guarded_probe_execution._plan.target_hand.pose.position.x == pytest.approx(1.0)
    value.executor.poll()
    assert len(value.client.sent_goals) == 2
    finish_motion(value)
    update = stable_feedback(value, position=0.995)
    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert "limit 10.00 mm" in update.detail
    assert value.stop.requests == []


@pytest.mark.parametrize("relative", [False, True])
def test_precision_error_above_200_mm_still_refuses_correction(monkeypatch, relative):
    value = precision_rig(monkeypatch, relative=relative)
    finish_motion(value)
    update = stable_feedback(value, position=0.7999)
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert "outside the bounded correction range" in update.detail
    assert len(value.client.sent_goals) == 1
    assert len(value.starts) == 1


def test_precision_total_correction_allowance_is_400_mm(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    for position in (0.800, 0.850):
        assert stable_feedback(value, position=position).outcome is ArmMovementOutcome.RUNNING
        value.executor.poll()
        finish_motion(value)
    update = stable_feedback(value, position=0.880)
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert "Total pose correction travel budget exceeded" in update.detail
    assert len(value.client.sent_goals) == 3
    assert value.stop.requests == []


def test_orientation_is_verified_even_when_position_is_correct(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    assert stable_feedback(value, yaw=math.radians(3)).outcome is ArmMovementOutcome.RUNNING
    assert len(value.starts) == 2
    value.executor.poll()
    finish_motion(value)
    assert stable_feedback(value, yaw=math.radians(1)).outcome is ArmMovementOutcome.SUCCESS


def test_old_duplicate_and_moving_pose_cannot_complete_precision(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    assert feedback(value, 0.5, acquisition=0.1).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 0.6).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 0.7, acquisition=0.6).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 0.8, position=1.002).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 0.95, position=1.002).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 1.1, position=1.002).outcome is ArmMovementOutcome.SUCCESS
    assert len(value.client.sent_goals) == 1


def test_missing_tf_invalidates_previously_stable_samples(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    feedback(value, 0.5)
    feedback(value, 0.65)
    del value.tf.transforms[("odom", "hand")]
    value.clock.now = 0.7
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert value.executor._tag_accuracy["stability"].sample_count == 0
    assert feedback(value, 0.8).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 0.95).outcome is ArmMovementOutcome.RUNNING
    assert feedback(value, 1.1).outcome is ArmMovementOutcome.SUCCESS


def test_polling_old_tf_after_it_ages_out_preserves_valid_samples(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    feedback(value, 0.5)
    feedback(value, 0.65)
    # The already counted pose ages past .25 s between arrivals. It is not a
    # new invalid observation and cannot erase the two accepted acquisitions.
    update = feedback(value, 0.92, acquisition=0.65)
    assert update.outcome is ArmMovementOutcome.RUNNING
    assert "distinct, advancing" in update.detail
    window = value.executor._tag_accuracy["stability"]
    assert window.sample_count == 2
    assert window.last_reset_reason == ""
    # New delayed feedback remains fresh, with .15 s acquisition spacing.
    assert feedback(value, 0.94, acquisition=0.8).outcome is ArmMovementOutcome.SUCCESS


def test_new_stale_tf_still_invalidates_the_precision_window(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    feedback(value, 0.5)
    feedback(value, 0.65)
    assert feedback(value, 1.1, acquisition=0.8).outcome is ArmMovementOutcome.RUNNING
    window = value.executor._tag_accuracy["stability"]
    assert window.sample_count == 0
    assert "stale" in window.last_reset_reason
    # A subsequent fresh acquisition cannot reuse the samples from before
    # invalid feedback, even though the robot pose did not change.
    assert feedback(value, 1.2).outcome is ArmMovementOutcome.RUNNING
    assert window.sample_count == 1


@pytest.mark.parametrize("acquisition", [-10.0, -11.0])
def test_invalid_tf_stamp_clears_window_instead_of_being_ignored(monkeypatch, acquisition):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    feedback(value, 0.5)
    feedback(value, 0.65)
    assert feedback(value, 0.8, acquisition=acquisition).outcome is ArmMovementOutcome.RUNNING
    window = value.executor._tag_accuracy["stability"]
    assert window.sample_count == 0
    assert "timestamp is invalid" in window.last_reset_reason


def test_precision_timeout_and_cancel_do_not_start_correction(monkeypatch):
    value = precision_rig(monkeypatch)
    finish_motion(value)
    update = feedback(value, 6.6, acquisition=0.0)
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert len(value.client.sent_goals) == 1
    assert value.stop.requests == []

    value = precision_rig(monkeypatch)
    finish_motion(value)
    value.executor.cancel()
    assert value.executor.cancelling
    assert len(value.stop.requests) == 1
    assert len(value.client.sent_goals) == 1
    assert stable_feedback(value, position=0.990).outcome is ArmMovementOutcome.RUNNING
    assert len(value.client.sent_goals) == 1
