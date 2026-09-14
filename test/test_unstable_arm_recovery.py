"""Focused tests for bounded local recovery from an unstable arm stop."""

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineOutcome,
    ForceBaselineUpdate,
    HandForceBaseline,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import HandForceSample
from fault_detector_spot.manipulation.guarded_probe_execution import (
    GuardedProbeExecution,
)
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingOutcome,
    HandSettlingUpdate,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlan,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ArmState:

    def __init__(self):
        self.sample = HandForceSample(
            received_at=0.0,
            x_n=1.0,
            y_n=2.0,
            z_n=3.0,
        )
        self.last_received_at = 0.0

    def hand_force_sample(self):
        return self.sample


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
        raise AssertionError("baseline should already be ready")

    def reset(self):
        pass


class ScriptedSettling:

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.start_count = 0

    def start(self):
        self.start_count += 1
        if not self.outcomes:
            raise AssertionError("unexpected settling start")
        outcome = self.outcomes.pop(0)
        return HandSettlingUpdate(outcome, outcome.value)

    def poll(self):
        raise AssertionError("scripted settling completes on start")

    def reset(self):
        pass


class ForcePolicy:

    consecutive_samples = 2

    @staticmethod
    def threshold_for(_speed):
        return 5.0


class GoalDriver:

    def __init__(self):
        self.started_goals = []
        self.updates = []
        self.cancel_count = 0

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


class PoseSource:

    def __init__(self, positions):
        self.positions = list(positions)
        self.last = self.positions[-1]

    def __call__(self, _frame):
        if self.positions:
            self.last = self.positions.pop(0)
        return pose(self.last)


def pose(x):
    value = PoseStamped()
    value.header.frame_id = "body"
    value.pose.position.x = x
    value.pose.orientation.w = 1.0
    return value


def plan():
    return ProbeMotionPlan(
        goal=object(),
        current_hand=pose(0.0),
        target_hand=pose(0.05),
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.005,
        direction_frame="body",
    )


def execution(state, driver, settling, current_pose, clock):
    return GuardedProbeExecution(
        arm_state_source=state,
        settling_detector=settling,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=ForcePolicy(),
        start_goal=driver.start,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        current_hand_pose=current_pose,
        build_motion_goal=lambda current, target, speed: (
            current,
            target,
            speed,
        ),
        default_angular_speed_rad_s=0.5,
        force_stale_timeout_sec=0.25,
        retreat_distance_m=0.01,
        retreat_speed_mps=0.01,
        monotonic_clock=clock,
    )


def trigger_contact(guard, state):
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=-5.0,
        y_n=2.0,
        z_n=3.0,
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.2,
        x_n=-5.0,
        y_n=2.0,
        z_n=3.0,
    )
    return guard.poll()


def test_settling_timeout_recovers_one_step_toward_pre_movement_pose():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.TIMEOUT,
        HandSettlingOutcome.SETTLED,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03]),
        clock,
    )

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING
    update = trigger_contact(guard, state)

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert len(driver.started_goals) == 2

    _, target, speed = driver.started_goals[-1]
    assert target.pose.position.x == pytest.approx(0.02)
    assert speed.linear_speed_mps == pytest.approx(0.01)

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "recovered 0.0100 m" in finished.detail
    assert "pre-movement pose" in finished.detail
    assert settling.start_count == 2
    assert not guard.active


def test_recovery_steps_repeat_only_while_arm_remains_unstable():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.TIMEOUT,
        HandSettlingOutcome.TIMEOUT,
        HandSettlingOutcome.SETTLED,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03, 0.03, 0.03, 0.02]),
        clock,
    )

    guard.start(plan)
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    assert len(driver.started_goals) == 3
    _, second_target, _ = driver.started_goals[-1]
    assert second_target.pose.position.x == pytest.approx(0.01)

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "recovered 0.0200 m" in finished.detail
    assert settling.start_count == 3


def test_recovery_stops_when_a_step_makes_no_measurable_progress():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.TIMEOUT,
        HandSettlingOutcome.TIMEOUT,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03, 0.03, 0.03, 0.03]),
        clock,
    )

    guard.start(plan)
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.RECOVERY_FAILED
    assert "no measurable progress" in finished.detail
    assert len(driver.started_goals) == 2
    assert not guard.active


def test_missing_settling_sensing_does_not_start_recovery_motion():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.SENSING_UNAVAILABLE,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03]),
        clock,
    )

    guard.start(plan)
    finished = trigger_contact(guard, state)

    assert finished.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert len(driver.started_goals) == 1
    assert driver.cancel_count == 1
    assert not guard.active


def test_force_abort_timeout_does_not_start_recovery_without_force_sensing():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.TIMEOUT,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03]),
        clock,
    )

    guard.start(plan)
    clock.now = 0.25
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert len(driver.started_goals) == 1
    assert driver.cancel_count == 1
    assert not guard.active


def test_post_retreat_settling_timeout_uses_local_recovery():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.SETTLED,
        HandSettlingOutcome.TIMEOUT,
        HandSettlingOutcome.SETTLED,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03, 0.03, 0.03, 0.02]),
        clock,
    )

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING
    assert len(driver.started_goals) == 2

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    recovering = guard.poll()

    assert recovering.outcome is ArmMovementOutcome.RUNNING
    assert len(driver.started_goals) == 3
    _, recovery_target, speed = driver.started_goals[-1]
    assert recovery_target.pose.position.x == pytest.approx(0.01)
    assert speed.linear_speed_mps == pytest.approx(0.01)

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "retreated 0.0100 m" in finished.detail
    assert "arm remained unstable after retreat" in finished.detail
    assert "recovered 0.0100 m" in finished.detail
    assert settling.start_count == 3
    assert not guard.active


def test_post_retreat_missing_sensing_does_not_start_recovery_motion():
    clock = ManualClock()
    state = ArmState()
    driver = GoalDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.SETTLED,
        HandSettlingOutcome.SENSING_UNAVAILABLE,
    ])
    guard = execution(
        state,
        driver,
        settling,
        PoseSource([0.03, 0.03, 0.03]),
        clock,
    )

    guard.start(plan)
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING
    assert len(driver.started_goals) == 2

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.RETREAT_FAILED
    assert "physical stability could not be confirmed" in finished.detail
    assert len(driver.started_goals) == 2
    assert not guard.active
