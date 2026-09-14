"""Focused tests for guarded probe execution."""

from copy import deepcopy
from types import SimpleNamespace

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
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlan,
)
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingOutcome,
    HandSettlingUpdate,
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


class ImmediateSettling:

    def __init__(self):
        self.start_count = 0

    def start(self):
        self.start_count += 1
        return HandSettlingUpdate(
            HandSettlingOutcome.SETTLED,
            "settled",
        )

    def poll(self):
        raise AssertionError("Immediate settling should not need polling")

    def reset(self):
        pass


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
    def threshold_for(_linear_speed_mps):
        return 5.0


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
        goal="primary",
        current_hand=pose(0.0),
        target_hand=pose(0.01),
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.005,
        direction_frame="body",
    )


def execution(
    state,
    driver,
    clock,
    current_pose,
    force_policy=None,
    contact_evidence_analyzer=None,
):
    settling = ImmediateSettling()
    guard = GuardedProbeExecution(
        arm_state_source=state,
        settling_detector=settling,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=(
            force_policy
            if force_policy is not None
            else FixedForcePolicy()
        ),
        contact_evidence_analyzer=contact_evidence_analyzer,
        start_goal=driver.start,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        start_stop=driver.start_stop,
        poll_stop=driver.poll_stop,
        current_hand_pose=lambda _frame: deepcopy(current_pose),
        build_motion_goal=lambda current, target, speed: (
            "retreat",
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
    guard._test_settling = settling
    return guard


def test_contact_cancels_then_retreats_and_returns_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.008))

    started = guard.start(plan)
    assert started.outcome is ArmMovementOutcome.RUNNING
    assert len(driver.started_goals) == 1

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
    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.started_goals[-1] == ("arm_stop", 1)
    assert guard._test_settling.start_count == 0

    driver.stop_updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Stopped",
        )
    )
    retreating = guard.poll()

    assert retreating.outcome is ArmMovementOutcome.RUNNING
    assert guard._test_settling.start_count == 1
    retreat_goal = driver.started_goals[-1]
    assert retreat_goal[0] == "retreat"
    target = retreat_goal[2]
    speed = retreat_goal[3]
    assert target.pose.position.x == pytest.approx(0.0)
    assert speed.linear_speed_mps == pytest.approx(0.01)

    driver.updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Succeeded",
        )
    )
    stopping_again = guard.poll()

    assert stopping_again.outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1] == ("arm_stop", 2)

    driver.stop_updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Stopped",
        )
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "retreated 0.0080 m" in finished.detail
    assert guard._test_settling.start_count == 2
    assert not guard.active


def test_normal_movement_does_not_wait_for_pre_movement_settling():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    started = guard.start(plan)

    assert started.outcome is ArmMovementOutcome.RUNNING
    assert guard._test_settling.start_count == 0
    assert len(driver.started_goals) == 1


def test_primary_success_runs_arm_stop_and_settling_without_retreat():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    guard.start(plan)
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=2.0,
        y_n=2.0,
        z_n=3.0,
    )
    driver.updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Succeeded",
        )
    )

    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.started_goals[-1] == ("arm_stop", 1)
    assert driver.cancel_count == 0

    driver.stop_updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Stopped",
        )
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.SUCCESS
    assert guard._test_settling.start_count == 1
    assert not guard.active


def test_stale_force_cancels_then_arm_stops_before_returning_force_stale():
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
    failed = guard.poll()

    assert failed.outcome is ArmMovementOutcome.FORCE_STALE
    assert not guard.active


class HighForcePolicy:

    consecutive_samples = 2

    @staticmethod
    def threshold_for(_linear_speed_mps):
        return 12.0


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
    contact = guard.poll()

    assert contact.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.started_goals[-1] == ("arm_stop", 1)


def test_large_sideways_force_does_not_trigger_directional_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.1,
        x_n=1.0,
        y_n=22.0,
        z_n=3.0,
    )
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0


def test_force_in_commanded_direction_does_not_count_as_obstacle_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.0))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.1,
        x_n=8.0,
        y_n=2.0,
        z_n=3.0,
    )
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0



def test_sustained_high_off_axis_self_motion_does_not_trigger_contact():
    clock = ManualClock()
    state = FakeArmStateSource()
    state.velocity = SimpleNamespace(
        linear_x_mps=0.05,
        linear_y_mps=0.08,
        linear_z_mps=0.0,
    )
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.008))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

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
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.3,
        x_n=-5.0,
        y_n=2.0,
        z_n=3.0,
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0
    assert driver.stop_count == 0
    assert guard._self_motion_suppression_count == 2


def test_missing_hand_velocity_keeps_existing_force_guard_fallback():
    clock = ManualClock()
    state = FakeArmStateSource()
    state.velocity = None
    driver = GoalDriver()
    guard = execution(state, driver, clock, pose(0.008))

    guard.start(plan)
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
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.stop_count == 1


def test_evidence_analyzer_failure_keeps_existing_force_guard_fallback():
    class BrokenAnalyzer:
        def begin_movement(self):
            pass

        def analyze(self, **_kwargs):
            raise RuntimeError("classifier unavailable")

    clock = ManualClock()
    state = FakeArmStateSource()
    state.velocity = SimpleNamespace(
        linear_x_mps=0.05,
        linear_y_mps=0.08,
        linear_z_mps=0.0,
    )
    driver = GoalDriver()
    guard = execution(
        state,
        driver,
        clock,
        pose(0.008),
        contact_evidence_analyzer=BrokenAnalyzer(),
    )

    guard.start(plan)
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
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.stop_count == 1
