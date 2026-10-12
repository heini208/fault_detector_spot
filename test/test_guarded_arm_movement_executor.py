"""Focused tests for guarded probe execution."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineOutcome,
    ForceBaselineSampler,
    ForceBaselineUpdate,
    HandForceBaseline,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import (
    HandForceSample,
    HandVelocitySample,
)
from fault_detector_spot.manipulation.guarded_probe_execution import (
    GuardedProbeExecution,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlan,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeArmStateSource:

    def __init__(self):
        self.sample = HandForceSample(
            received_at=0.0,
            x_n=1.0,
            y_n=2.0,
            z_n=3.0,
        )
        self.velocity = None
        self.last_received_at = 0.0

    def hand_force_sample(self):
        return self.sample

    def hand_velocity_sample(self):
        return self.velocity

    def is_stale(self):
        return False


class ImmediateBaseline:

    def start(self):
        return ForceBaselineUpdate(
            ForceBaselineOutcome.READY,
            "baseline ready",
            HandForceBaseline(
                x_n=1.0,
                y_n=2.0,
                z_n=3.0,
                sample_count=10,
                sample_span_sec=0.5,
                maximum_component_span_n=0.1,
                last_received_at=0.0,
            ),
        )

    def poll(self):
        raise AssertionError("Immediate baseline should not need polling")

    def reset(self):
        pass


class FixedForcePolicy:

    consecutive_samples = 2

    @staticmethod
    def threshold_for(_linear_speed_mps, _angular_speed_rad_s=0.0):
        return 5.0


class HighForcePolicy:

    consecutive_samples = 2

    @staticmethod
    def threshold_for(_linear_speed_mps, _angular_speed_rad_s=0.0):
        return 12.0


class GoalDriver:

    def __init__(self):
        self.updates = []
        self.started_goals = []
        self.cancel_count = 0
        self.stop_count = 0
        self.stop_updates = []

    def start(self, goal):
        self.started_goals.append(goal)
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        )

    def poll(self):
        if not self.updates:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "moving",
            )
        return self.updates.pop(0)

    def cancel(self):
        self.cancel_count += 1

    def start_stop(self):
        self.stop_count += 1
        self.started_goals.append(("arm_stop", self.stop_count))
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "stop sent",
        )

    def poll_stop(self):
        if not self.stop_updates:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "waiting for stop service",
            )
        return self.stop_updates.pop(0)


def pose(x):
    result = PoseStamped()
    result.header.frame_id = "body"
    result.pose.position.x = x
    result.pose.orientation.w = 1.0
    return result


def plan():
    return ProbeMotionPlan(
        duration_sec=2.0,
        current_hand=pose(0.0),
        target_hand=pose(0.01),
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.005,
        angular_speed_rad_s=0.0,
        translational_motion=True,
        direction_frame="body",
    )


def velocity_sample(received_at, linear=0.0, angular=0.0):
    return HandVelocitySample(
        received_at=received_at,
        linear_x_mps=linear,
        linear_y_mps=0.0,
        linear_z_mps=0.0,
        angular_x_rad_s=angular,
        angular_y_rad_s=0.0,
        angular_z_rad_s=0.0,
    )


def confirm_physical_stop(guard, state, clock, first_sample_at):
    state.velocity = velocity_sample(first_sample_at)
    clock.now = first_sample_at
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(first_sample_at + 0.4)
    clock.now = first_sample_at + 0.4
    return guard.poll()


def execution(
    state,
    driver,
    clock,
    current_pose,
    force_policy=None,
    contact_evidence_analyzer=None,
    start_retreat=None,
):
    return GuardedProbeExecution(
        arm_state_source=state,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=(
            force_policy
            if force_policy is not None
            else FixedForcePolicy()
        ),
        contact_evidence_analyzer=contact_evidence_analyzer,
        start_motion=driver.start,
        start_retreat=start_retreat,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        start_stop=driver.start_stop,
        poll_stop=driver.poll_stop,
        current_hand_pose=lambda _frame: deepcopy(
            current_pose() if callable(current_pose) else current_pose
        ),
        build_motion_plan=lambda current, target, speed: (
            "retreat",
            current,
            target,
            speed,
        ),
        default_angular_speed_rad_s=0.5,
        force_stale_timeout_sec=0.25,
        hard_force_delta_limit_n=20.0,
        stop_confirmation_linear_velocity_threshold_mps=0.01,
        stop_confirmation_angular_velocity_threshold_rad_s=0.05,
        stop_confirmation_stable_duration_sec=0.4,
        stop_confirmation_timeout_sec=3.0,
        retreat_distance_m=0.01,
        retreat_speed_mps=0.01,
        monotonic_clock=clock,
    )


@pytest.mark.parametrize("stable_retry", [True, False])
def test_unstable_baseline_stops_once_then_requires_a_new_baseline(stable_retry):
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))
    guard.force_baseline_sampler = ForceBaselineSampler(
        state, minimum_samples=3, minimum_span_sec=0.2,
        maximum_component_span_n=3.0, timeout_sec=0.5,
        monotonic_clock=clock,
    )
    plans = []

    def build_plan():
        value = plan()
        value.current_hand.pose.position.x = clock.now
        plans.append(value)
        return value

    state.sample = HandForceSample(0.0, 0.0, 0.0, 0.0)
    assert guard.start(build_plan).outcome is ArmMovementOutcome.RUNNING
    for stamp, force in ((0.1, 9.0), (0.2, 0.0), (0.5, 9.0)):
        clock.now = stamp
        state.sample = HandForceSample(stamp, force, 0.0, 0.0)
        update = guard.poll()
    assert update.outcome is ArmMovementOutcome.RUNNING
    assert "ArmStopCommand sent" in update.detail
    assert driver.started_goals == [("arm_stop", 1)]

    driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped"))
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(0.5)
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(plans) == 1  # Pre-acceptance feedback cannot confirm the stop.
    assert confirm_physical_stop(guard, state, clock, 0.6).outcome is ArmMovementOutcome.RUNNING
    assert len(plans) == 2
    assert plans[1].current_hand.pose.position.x == pytest.approx(1.0)
    assert driver.started_goals == [("arm_stop", 1)]

    # The pre-stop force sample must not contribute to the restarted baseline.
    for stamp, force in ((1.1, 2.0), (1.2, 2.1 if stable_retry else 11.0)):
        clock.now = stamp
        state.sample = HandForceSample(stamp, force, 0.0, 0.0)
        assert guard.poll().outcome is ArmMovementOutcome.RUNNING
        assert driver.started_goals == [("arm_stop", 1)]
    clock.now = 1.3
    state.sample = HandForceSample(1.3, 2.0, 0.0, 0.0)
    update = guard.poll()
    if stable_retry:
        assert update.outcome is ArmMovementOutcome.RUNNING
        assert driver.started_goals == [("arm_stop", 1), plans[1]]
        assert guard._force_threshold_n == 5.0
    else:
        clock.now = 1.5
        update = guard.poll()
        assert update.outcome is ArmMovementOutcome.FORCE_UNSTABLE
        assert "retry after ArmStop failed" in update.detail
        assert not guard.active
        assert driver.started_goals == [("arm_stop", 1)]
    assert driver.stop_count == 1


class FailedBaseline(ImmediateBaseline):

    def __init__(self, outcome):
        self.outcome = outcome
        self.starts = 0

    def start(self):
        self.starts += 1
        return ForceBaselineUpdate(self.outcome, "Baseline failed")


@pytest.mark.parametrize("outcome", [
    ForceBaselineOutcome.FORCE_STALE,
    ForceBaselineOutcome.FORCE_UNAVAILABLE,
    ForceBaselineOutcome.TIMEOUT,
])
def test_missing_baseline_data_does_not_attempt_stop_recovery(outcome):
    driver = GoalDriver()
    guard = execution(FakeArmStateSource(), driver, ManualClock(), pose(0.0))
    guard.force_baseline_sampler = FailedBaseline(outcome)

    assert guard.start(plan).outcome is not ArmMovementOutcome.RUNNING
    assert not guard.active
    assert driver.started_goals == []


@pytest.mark.parametrize("stop_failure", ["service", "settling"])
def test_baseline_recovery_never_moves_after_unconfirmed_stop(stop_failure):
    clock = ManualClock()
    driver = GoalDriver()
    guard = execution(FakeArmStateSource(), driver, clock, pose(0.0))
    sampler = FailedBaseline(ForceBaselineOutcome.FORCE_UNSTABLE)
    guard.force_baseline_sampler = sampler
    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING
    driver.stop_updates.append(ArmMovementUpdate(
        ArmMovementOutcome.EXECUTION_ERROR if stop_failure == "service"
        else ArmMovementOutcome.SUCCESS,
        "Stop response",
    ))
    update = guard.poll()
    if stop_failure == "settling":
        assert update.outcome is ArmMovementOutcome.RUNNING
        clock.now = 3.0
        update = guard.poll()
    assert update.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert not guard.active
    assert sampler.starts == 1
    assert driver.started_goals == [("arm_stop", 1)]


@pytest.mark.parametrize("settling", [False, True])
def test_cancellation_during_baseline_recovery_cannot_resume_motion(settling):
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))
    sampler = FailedBaseline(ForceBaselineOutcome.FORCE_UNSTABLE)
    guard.force_baseline_sampler = sampler
    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING
    if settling:
        driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped"))
        assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert guard.cancel().outcome is ArmMovementOutcome.RUNNING
    driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped"))
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    update = confirm_physical_stop(guard, state, clock, 0.1)

    assert update.outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
    assert not guard.active
    assert sampler.starts == 1
    assert driver.started_goals == [("arm_stop", 1), ("arm_stop", 2)]


@pytest.mark.parametrize("override,expected", [(None, 0.008), (0.003, 0.003), (0.02, 0.008)])
@pytest.mark.parametrize("separate_retreat", [False, True])
def test_contact_cancels_stops_retreats_and_returns_contact(override, expected, separate_retreat):
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    retreats = []

    def start_retreat(plan):
        retreats.append(plan)
        return driver.start(plan)

    guard = execution(
        state, driver, clock, pose(0.008),
        start_retreat=start_retreat if separate_retreat else None,
    )

    assert guard.start(plan, retreat_distance_m=override).outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.1
    state.sample = HandForceSample(0.1, -5.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    clock.now = 0.2
    state.sample = HandForceSample(0.2, -5.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.started_goals[-1] == ("arm_stop", 1)

    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    settling = guard.poll()

    assert settling.outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1] == ("arm_stop", 1)

    state.velocity = velocity_sample(0.3, linear=0.02)
    clock.now = 0.3
    moving = guard.poll()
    assert moving.outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1] == ("arm_stop", 1)

    state.velocity = velocity_sample(0.4)
    clock.now = 0.4
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(0.8)
    clock.now = 0.8
    retreating = guard.poll()

    assert retreating.outcome is ArmMovementOutcome.RUNNING
    retreat_goal = driver.started_goals[-1]
    assert retreat_goal[0] == "retreat"
    assert retreat_goal[2].pose.position.x == pytest.approx(0.008 - expected)
    assert retreat_goal[3].linear_speed_mps == pytest.approx(0.01)
    assert len(retreats) == int(separate_retreat)

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert f"retreated {expected:.4f} m" in finished.detail
    assert "ArmStopCommand accepted" not in finished.detail
    assert driver.stop_count == 1
    assert not guard.active


    guard.start(plan)
    assert guard._retreat_distance_override_m is None
    assert guard.retreat_distance_m == pytest.approx(0.01)


def test_primary_success_finishes_without_arm_stop():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    guard.start(plan)
    state.sample = HandForceSample(0.1, 2.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )

    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.SUCCESS
    assert "Succeeded" in finished.detail
    assert driver.cancel_count == 0
    assert driver.stop_count == 0
    assert not guard.active


def test_stale_force_cancels_and_finishes_after_arm_stop_confirmation():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    guard.start(plan)
    clock.now = 0.25
    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.started_goals[-1] == ("arm_stop", 1)

    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    settling = guard.poll()

    assert settling.outcome is ArmMovementOutcome.RUNNING
    failed = confirm_physical_stop(guard, state, clock, 0.3)

    assert failed.outcome is ArmMovementOutcome.FORCE_STALE
    assert "physical stop confirmed" in failed.detail
    assert not guard.active


def test_stop_confirmation_requires_new_post_acceptance_samples():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    guard.start(plan)
    clock.now = 0.25
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    state.velocity = velocity_sample(0.2)
    clock.now = 0.3
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(0.3)
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.8
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(0.7)
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.FORCE_STALE
    assert "physical stop confirmed" in finished.detail


def test_stop_confirmation_motion_resets_stability_before_retreat():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.008))

    guard.start(plan)
    for received_at in (0.1, 0.2):
        clock.now = received_at
        state.sample = HandForceSample(received_at, -5.0, 2.0, 3.0)
        guard.observe_force_sample(state.sample)
    assert driver.stop_count == 1
    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    for received_at in (0.3, 0.5):
        state.velocity = velocity_sample(received_at)
        clock.now = received_at
        assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(0.6, linear=0.02)
    clock.now = 0.6
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1] == ("arm_stop", 1)

    state.velocity = velocity_sample(0.7)
    clock.now = 0.7
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.velocity = velocity_sample(1.1)
    clock.now = 1.1
    retreating = guard.poll()

    assert retreating.outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1][0] == "retreat"


def test_stop_confirmation_times_out_without_fresh_velocity():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    guard.start(plan)
    clock.now = 0.25
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    clock.now = 3.25
    failed = guard.poll()

    assert failed.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert "physical stop was not confirmed" in failed.detail
    assert not guard.active


def test_explicit_threshold_override_replaces_generic_policy():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(
        state,
        driver,
        clock,
        pose(0.008),
        force_policy=HighForcePolicy(),
    )

    assert guard.start(
        plan,
        force_threshold_n=5.0,
    ).outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(0.1, -5.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    state.sample = HandForceSample(0.2, -5.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    contact = guard.poll()

    assert contact.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.started_goals[-1] == ("arm_stop", 1)


def test_hard_force_limit_stops_on_first_fresh_sample():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.1
    state.sample = HandForceSample(0.1, -20.0, 2.0, 3.0)
    update = guard.observe_force_sample(state.sample)

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.stop_count == 1
    assert guard._force_contact_count == 0


def test_hard_force_limit_is_not_suppressed_by_self_motion():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    current = {"pose": pose(0.0)}
    guard = execution(state, driver, clock, lambda: current["pose"])

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.1
    current["pose"] = pose(0.01)
    current["pose"].pose.position.y = 0.01
    state.sample = HandForceSample(0.1, -20.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)

    assert driver.cancel_count == 1
    assert driver.stop_count == 1
    assert guard._self_motion_suppression_count == 0


def test_large_sideways_force_does_not_trigger_directional_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(0.1, 1.0, 21.0, 3.0)
    guard.observe_force_sample(state.sample)
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0


def test_force_in_commanded_direction_is_not_obstacle_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(0.1, 8.0, 2.0, 3.0)
    guard.observe_force_sample(state.sample)
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0


def test_sustained_high_off_axis_self_motion_is_suppressed():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    current = {"pose": pose(0.0)}
    guard = execution(
        state,
        driver,
        clock,
        lambda: current["pose"],
    )

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    samples = (
        (0.1, 0.01, 0.005),
        (0.2, 0.02, 0.010),
        (0.3, 0.03, 0.015),
    )
    for received_at, x, y in samples:
        clock.now = received_at
        current["pose"] = pose(x)
        current["pose"].pose.position.y = y
        state.sample = HandForceSample(
            received_at,
            -5.0,
            2.0,
            3.0,
        )
        guard.observe_force_sample(state.sample)
        assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    assert driver.cancel_count == 0
    assert driver.stop_count == 0
    assert guard._self_motion_suppression_count == 2
