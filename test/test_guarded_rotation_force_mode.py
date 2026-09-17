"""Guarded rotation-only force handling regression tests."""

from copy import deepcopy
import math

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


class FakeArmStateSource:

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

    def hand_velocity_sample(self):
        return None


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



class RecordingForcePolicy:

    consecutive_samples = 2

    def __init__(self, threshold_n=5.0):
        self.threshold_n = threshold_n
        self.calls = []

    def threshold_for(self, linear_speed_mps, angular_speed_rad_s=0.0):
        self.calls.append((linear_speed_mps, angular_speed_rad_s))
        return self.threshold_n


class GoalDriver:

    def __init__(self):
        self.updates = []
        self.started_goals = []
        self.cancel_count = 0
        self.stop_count = 0
        self.stop_updates = []

    def start(self, goal):
        self.started_goals.append(goal)
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "Goal sent")

    def poll(self):
        if not self.updates:
            return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "moving")
        return self.updates.pop(0)

    def cancel(self):
        self.cancel_count += 1

    def start_stop(self):
        self.stop_count += 1
        self.started_goals.append(("arm_stop", self.stop_count))
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "stop sent")

    def poll_stop(self):
        if not self.stop_updates:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "waiting for stop",
            )
        return self.stop_updates.pop(0)


def pose(yaw=0.0):
    result = PoseStamped()
    result.header.frame_id = "body"
    result.pose.orientation.z = math.sin(yaw * 0.5)
    result.pose.orientation.w = math.cos(yaw * 0.5)
    return result


def rotation_plan():
    return ProbeMotionPlan(
        duration_sec=2.0,
        current_hand=pose(),
        target_hand=pose(yaw=0.25),
        direction_x=0.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.0,
        angular_speed_rad_s=0.5,
        translational_motion=False,
        direction_frame="body",
    )


def execution(state, driver, policy):
    guard = GuardedProbeExecution(
        arm_state_source=state,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=policy,
        start_motion=driver.start,
        poll_goal=driver.poll,
        cancel_goal=driver.cancel,
        start_stop=driver.start_stop,
        poll_stop=driver.poll_stop,
        current_hand_pose=lambda _frame: deepcopy(pose()),
        build_motion_plan=lambda current, target, speed: (
            "retreat",
            current,
            target,
            speed,
        ),
        default_angular_speed_rad_s=0.5,
        force_stale_timeout_sec=0.25,
        retreat_distance_m=0.01,
        retreat_speed_mps=0.01,
        monotonic_clock=lambda: 0.0,
    )
    return guard


def test_rotation_threshold_receives_angular_speed():
    state = FakeArmStateSource()
    driver = GoalDriver()
    policy = RecordingForcePolicy()
    guard = execution(state, driver, policy)

    started = guard.start(rotation_plan)

    assert started.outcome is ArmMovementOutcome.RUNNING
    assert policy.calls == [(0.0, 0.5)]


def test_rotation_self_load_below_angular_threshold_does_not_trigger_contact():
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, RecordingForcePolicy(threshold_n=5.0))

    guard.start(rotation_plan)
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=5.32,
        y_n=2.0,
        z_n=3.0,
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.2,
        x_n=5.32,
        y_n=2.0,
        z_n=3.0,
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 0
    assert driver.stop_count == 0


def test_successful_rotation_finishes_without_arm_stop():
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, RecordingForcePolicy())

    guard.start(rotation_plan)
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=1.5,
        y_n=2.0,
        z_n=3.0,
    )
    driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Succeeded")
    )

    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.SUCCESS
    assert "orientation threshold 5.00 N" in finished.detail
    assert driver.stop_count == 0
    assert driver.cancel_count == 0
    assert not guard.active


def test_rotation_contact_uses_total_delta_and_does_not_retreat():
    state = FakeArmStateSource()
    driver = GoalDriver()
    guard = execution(state, driver, RecordingForcePolicy())

    guard.start(rotation_plan)
    state.sample = HandForceSample(
        received_at=0.1,
        x_n=7.0,
        y_n=2.0,
        z_n=3.0,
    )
    assert guard.poll().outcome is ArmMovementOutcome.RUNNING

    state.sample = HandForceSample(
        received_at=0.2,
        x_n=7.0,
        y_n=2.0,
        z_n=3.0,
    )
    stopping = guard.poll()

    assert stopping.outcome is ArmMovementOutcome.RUNNING
    assert driver.cancel_count == 1
    assert driver.stop_count == 1
    assert "arm_stop" in driver.started_goals[-1]

    driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Stopped")
    )
    finished = guard.poll()

    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert "total end-effector force delta" in finished.detail
    assert "orientation threshold 5.00 N" in finished.detail
    assert all(
        not isinstance(goal, tuple) or goal[0] != "retreat"
        for goal in driver.started_goals
    )
    assert not guard.active
