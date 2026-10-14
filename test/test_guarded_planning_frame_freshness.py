"""Frozen odom targets must still match the planned body target at dispatch."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.arm_state_source import ArmStowState, HandForceSample
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome, MoveItPlanUpdate
from test_arm_movement_executor import (
    FakeArmStateSource, FakeArmStopServiceClient, FakeGoalHandle, FakeTransformer,
    ManualClock, ManualFuture, capture_builder, confirm_physical_stop,
    executor_with_client, transform,
)
from test_guarded_arm_movement_executor import ImmediateBaseline, FixedForcePolicy


OPTIONS = dict(
    planning_frame_position_tolerance_m=0.001,
    planning_frame_orientation_tolerance_rad=math.radians(0.2),
)


def stamped_transform(x=0.0, *, yaw=0.0, stamp=10.0):
    result = transform("body", "odom", x=x, yaw=yaw)
    stamp_ns = round(stamp * 1e9)
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    return result


def rig(monkeypatch, *, stowed=False, ignore_environment_collisions=False):
    events = []
    clock = ManualClock()
    state = FakeArmStateSource(ArmStowState.STOWED if stowed else ArmStowState.DEPLOYED)
    state.hand_force_sample = lambda: HandForceSample(clock.now, 1.0, 2.0, 3.0)
    tf = FakeTransformer({
        ("body", "odom"): stamped_transform(),
        ("odom", "hand"): transform("odom", "hand"),
        ("flat_body", "hand"): transform("flat_body", "hand"),
        ("body", "hand"): transform("body", "hand"),
    })
    starts = []
    baseline_actions = []

    class Baseline(ImmediateBaseline):
        def start(self):
            events.append("baseline")
            if baseline_actions:
                baseline_actions.pop(0)()
            return super().start()

    def plan_start(target, **kwargs):
        events.append("plan")
        starts.append(deepcopy(target))
        assert kwargs == {"ignore_environment_collisions": ignore_environment_collisions}
        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "Planning")

    planner = SimpleNamespace(
        planning_frame="body", start=plan_start, start_cartesian=plan_start,
        poll=lambda: MoveItPlanUpdate(MoveItPlanOutcome.SUCCESS, "Planned", object()),
        cancel=lambda: None,
        validate_prepared_scene=lambda: events.append("scene check"),
    )
    stop = FakeArmStopServiceClient()
    executor, client = executor_with_client(
        tf, arm_state_source=state, arm_stop_service_client=stop,
        moveit_arm_planner=planner, monotonic_clock=clock,
        force_baseline_sampler=Baseline(), force_contact_policy=FixedForcePolicy(),
        config=ArmMotionParameters(),
    )
    # Drive the execution synchronously; only the TF age check needs ROS time.
    state.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000),
    ))
    monkeypatch.setattr(executor, "_build_moveit_joint_goal",
                        lambda *_args, **_kwargs: events.append("dispatch") or object())
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.position.x = 1.0
    target.pose.orientation.w = 1.0
    return SimpleNamespace(
        executor=executor, client=client, stop=stop, state=state, clock=clock,
        tf=tf, target=target, events=events, starts=starts,
        baseline_actions=baseline_actions,
    )


def begin(value):
    return value.executor.guarded_probe(value.target, "hand", cartesian_path=True, **OPTIONS)


def test_replan_rebuilds_current_hand_and_baseline_before_only_dispatch(monkeypatch):
    value = rig(monkeypatch)

    def drift():
        value.tf.transforms[("body", "odom")] = stamped_transform(0.003)
        value.tf.transforms[("odom", "hand")] = transform("odom", "hand", x=0.02)

    value.baseline_actions.append(drift)
    assert begin(value).outcome is ArmMovementOutcome.RUNNING
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert value.events == ["plan", "baseline", "plan"]
    assert value.client.sent_goals == value.stop.requests == []
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert value.events == ["plan", "baseline", "plan", "baseline", "scene check", "dispatch"]
    assert [p.pose.position.x for p in value.starts] == pytest.approx([1.0, 1.003])
    guard_plan = value.executor.guarded_probe_execution._plan
    assert guard_plan.current_hand.pose.position.x == pytest.approx(0.02)
    assert guard_plan.target_hand.pose.position.x == pytest.approx(1.0)
    assert len(value.client.sent_goals) == 1
    assert value.stop.requests == []


def test_continuous_drift_exhausts_two_replans_without_command_or_stop(monkeypatch):
    value = rig(monkeypatch)
    for offset in (0.003, 0.006, 0.009):
        value.baseline_actions.append(lambda offset=offset: value.tf.transforms.update({
            ("body", "odom"): stamped_transform(offset),
        }))
    begin(value)
    for _ in range(3):
        update = value.executor.poll()
    assert update.outcome is ArmMovementOutcome.PLANNING_FAILED
    assert "after 2 replans" in update.detail
    assert value.events == ["plan", "baseline"] * 3
    assert value.client.sent_goals == value.stop.requests == []
    assert not value.executor.active


@pytest.mark.parametrize("yaw, replans", [(0.03, 0), (0.1, 1), (0.3, 1)])
def test_rotation_drift_includes_target_lever_arm(monkeypatch, yaw, replans):
    value = rig(monkeypatch)
    value.baseline_actions.append(lambda: value.tf.transforms.update({
        ("body", "odom"): stamped_transform(yaw=math.radians(yaw)),
    }))
    begin(value)
    value.executor.poll()
    assert len(value.starts) == 1 + replans
    if replans:
        assert value.client.sent_goals == []
        value.executor.poll()
    assert len(value.client.sent_goals) == 1
    assert value.stop.requests == []


@pytest.mark.parametrize("bad_transform, detail", [
    (stamped_transform(stamp=9.7), "not fresh"),
    (stamped_transform(stamp=0.0), "timestamp is empty"),
    (stamped_transform(stamp=10.1), "not fresh"),
    (stamped_transform(stamp=9.99), "moved backwards"),
    (None, "Unexpected TF lookup"),
])
def test_missing_or_invalid_dispatch_tf_fails_without_movement(monkeypatch, bad_transform, detail):
    value = rig(monkeypatch)
    begin(value)
    if bad_transform is None:
        del value.tf.transforms[("body", "odom")]
    else:
        value.tf.transforms[("body", "odom")] = bad_transform
    update = value.executor.poll()
    assert update.outcome is ArmMovementOutcome.PLANNING_FAILED
    assert detail in update.detail
    assert value.client.sent_goals == value.stop.requests == []
    assert not value.executor.active


def test_snapshot_uses_the_same_tf_as_target_conversion(monkeypatch):
    value = rig(monkeypatch)
    sampled = stamped_transform(0.012)
    value.tf.transforms[("body", "odom")] = sampled
    begin(value)
    original, planned, snapshot = value.executor._planning_frame_snapshot
    assert value.tf.calls.count(("body", "odom", 0.0)) == 1
    assert original.pose.position.x == pytest.approx(1.0)
    assert planned.pose.position.x == pytest.approx(1.012)
    assert snapshot == sampled
    sampled.transform.translation.x = 0.05
    assert snapshot.transform.translation.x == pytest.approx(0.012)


def test_default_opt_out_accepts_unstamped_tf(monkeypatch):
    value = rig(monkeypatch)
    value.tf.transforms[("body", "odom")] = stamped_transform(stamp=0.0)
    update = value.executor.guarded_probe(value.target, "hand", cartesian_path=True)
    assert update.outcome is ArmMovementOutcome.RUNNING
    value.executor.poll()
    assert len(value.client.sent_goals) == 1


def test_stowed_ready_lift_preserves_opt_in_for_primary_plan(monkeypatch):
    value = rig(monkeypatch, stowed=True)
    capture_builder(monkeypatch)
    assert begin(value).outcome is ArmMovementOutcome.RUNNING
    assert len(value.client.sent_goals) == 1
    assert value.starts == []
    result = ManualFuture()
    value.client.send_future.set_result(FakeGoalHandle(result_future=result))
    value.executor.poll()
    value.state.state = ArmStowState.DEPLOYED
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=True)))
    value.executor.poll()
    assert len(value.stop.requests) == 1
    value.stop.future.set_result(SimpleNamespace(success=True))
    value.executor.poll()
    value.client.send_future = ManualFuture()
    assert confirm_physical_stop(value.executor, value.clock, 0.1).outcome is ArmMovementOutcome.RUNNING
    assert len(value.starts) == 1
    # The deployment stop is expected; frame verification remains active for
    # the pending primary trajectory and cannot send it with stale TF.
    value.tf.transforms[("body", "odom")] = stamped_transform(stamp=9.5)
    update = value.executor.poll()
    assert update.outcome is ArmMovementOutcome.PLANNING_FAILED
    assert len(value.client.sent_goals) == len(value.stop.requests) == 1
