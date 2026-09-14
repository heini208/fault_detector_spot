"""Regression tests for fail-open guarded contact telemetry."""

from copy import deepcopy

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


class Baseline:

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


class Settling:

    def start(self):
        return HandSettlingUpdate(
            HandSettlingOutcome.SETTLED,
            "settled",
        )

    def poll(self):
        raise AssertionError("settling should already be complete")

    def reset(self):
        pass


class Policy:

    consecutive_samples = 2

    @staticmethod
    def threshold_for(_speed):
        return 5.0


class Driver:

    def __init__(self):
        self.cancel_count = 0

    def start(self, _goal):
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        )

    def poll(self):
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "moving",
        )

    def cancel(self):
        self.cancel_count += 1


class Telemetry:

    def __init__(self, fail=False):
        self.fail = fail
        self.begin_count = 0
        self.observations = []

    def begin_movement(self):
        self.begin_count += 1
        if self.fail:
            raise RuntimeError("telemetry unavailable")
        return self.begin_count

    def observe(self, **values):
        if self.fail:
            raise RuntimeError("telemetry failed")
        self.observations.append(values)


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
        target_hand=pose(0.01),
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.005,
        direction_frame="body",
    )


def execution(state, driver, clock, telemetry):
    return GuardedProbeExecution(
        arm_state_source=state,
        settling_detector=Settling(),
        force_baseline_sampler=Baseline(),
        force_contact_policy=Policy(),
        start_goal=driver.start,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        start_stop=lambda: ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "stop sent",
        ),
        poll_stop=lambda: ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "stop accepted",
        ),
        current_hand_pose=lambda _frame: deepcopy(pose(0.004)),
        build_motion_goal=lambda current, target, speed: (
            current,
            target,
            speed,
        ),
        default_angular_speed_rad_s=0.5,
        force_stale_timeout_sec=0.25,
        retreat_distance_m=0.01,
        retreat_speed_mps=0.01,
        contact_telemetry=telemetry,
        monotonic_clock=clock,
    )


def test_guard_emits_shadow_observation_for_each_fresh_force_sample():
    clock = ManualClock()
    state = ArmState()
    driver = Driver()
    telemetry = Telemetry()
    guard = execution(state, driver, clock, telemetry)

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.1
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=2.0,
        y_n=2.0,
        z_n=3.0,
    )
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0
    assert telemetry.begin_count == 1
    assert len(telemetry.observations) == 1
    observation = telemetry.observations[0]
    assert observation["movement_sequence"] == 1
    assert observation["elapsed_sec"] == 0.1
    assert observation["phase"] == "moving"
    assert observation["force_sample"] is state.sample
    assert observation["current_hand"].pose.position.x == 0.004


def test_telemetry_failure_cannot_change_guard_decision():
    clock = ManualClock()
    state = ArmState()
    driver = Driver()
    guard = execution(state, driver, clock, Telemetry(fail=True))

    assert guard.start(plan).outcome is ArmMovementOutcome.RUNNING

    clock.now = 0.1
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=2.0,
        y_n=2.0,
        z_n=3.0,
    )
    update = guard.poll()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0
