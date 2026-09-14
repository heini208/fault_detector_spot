"""Focused tests for ArmStop-based guarded movement finalization."""

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


class StopDriver:

    def __init__(self):
        self.count = 0
        self.updates = []

    def start(self):
        self.count += 1
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "stop sent",
        )

    def poll(self):
        if not self.updates:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "waiting for stop service",
            )
        return self.updates.pop(0)


def pose(x):
    value = PoseStamped()
    value.header.frame_id = "body"
    value.pose.position.x = x
    value.pose.orientation.w = 1.0
    return value


def plan(force_guard_enabled=True):
    return ProbeMotionPlan(
        goal="primary",
        current_hand=pose(0.0),
        target_hand=pose(0.05),
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.005,
        direction_frame="body",
        force_guard_enabled=force_guard_enabled,
    )


def execution(
    state,
    driver,
    settling,
    stop_driver,
    clock,
    current_x=0.03,
):
    return GuardedProbeExecution(
        arm_state_source=state,
        settling_detector=settling,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=ForcePolicy(),
        start_goal=driver.start,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        start_stop=stop_driver.start,
        poll_stop=stop_driver.poll,
        current_hand_pose=lambda _frame: deepcopy(pose(current_x)),
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


def test_successful_cartesian_move_is_stopped_then_settled():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([HandSettlingOutcome.SETTLED])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    assert guard.start(plan()).outcome is ArmMovementOutcome.RUNNING
    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )

    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert stop_driver.count == 1

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.SUCCESS
    assert "ArmStopCommand accepted" in finished.detail
    assert settling.start_count == 1
    assert not guard.active


def test_cartesian_stall_is_stopped_then_preserved():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([HandSettlingOutcome.SETTLED])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    guard.start(plan())
    driver.updates.append(
        ArmMovementUpdate(
            ArmMovementOutcome.TRAJECTORY_STALLED,
            "Cartesian arm trajectory stalled",
        )
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.TRAJECTORY_STALLED
    assert "Cartesian arm trajectory stalled" in finished.detail
    assert not guard.active


def test_contact_uses_arm_stop_before_and_after_retreat():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.SETTLED,
        HandSettlingOutcome.SETTLED,
    ])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    assert guard.start(plan()).outcome is ArmMovementOutcome.RUNNING
    stopping = trigger_contact(guard, state)

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert stop_driver.count == 1

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    retreating = guard.poll()

    assert retreating.outcome is ArmMovementOutcome.RUNNING
    retreat_goal = driver.started_goals[-1]
    assert retreat_goal[0] == "retreat"
    assert retreat_goal[2].pose.position.x == 0.02

    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "retreat complete")
    )
    stopping_again = guard.poll()

    assert stopping_again.outcome is ArmMovementOutcome.RUNNING
    assert stop_driver.count == 2

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "retreated 0.0100 m" in finished.detail
    assert settling.start_count == 2
    assert stop_driver.count == 2
    assert not guard.active


def test_arm_stop_settling_timeout_releases_executor_as_unstable():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([HandSettlingOutcome.TIMEOUT])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    guard.start(plan())
    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.UNSTABLE_ARM
    assert "remained unstable" in finished.detail
    assert not guard.active


def test_contact_does_not_retreat_when_arm_stop_cannot_settle():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([HandSettlingOutcome.TIMEOUT])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    guard.start(plan())
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.UNSTABLE_ARM
    assert len(driver.started_goals) == 1
    assert stop_driver.count == 1
    assert not guard.active


def test_missing_settling_sensing_releases_executor_without_retreat():
    state = ArmState()
    driver = GoalDriver()
    stop_driver = StopDriver()
    settling = ScriptedSettling([
        HandSettlingOutcome.SENSING_UNAVAILABLE,
    ])
    guard = execution(
        state,
        driver,
        settling,
        stop_driver,
        ManualClock(),
    )

    guard.start(plan())
    assert trigger_contact(guard, state).outcome is ArmMovementOutcome.RUNNING

    stop_driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop complete")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert len(driver.started_goals) == 1
    assert not guard.active
