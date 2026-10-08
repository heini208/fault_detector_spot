"""A completed walk must relinquish its persistent mobility trajectory."""

from types import SimpleNamespace
import math

import pytest
from bosdyn.api import robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.base_motion_planner import BaseMovementPlan
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor, BaseMovementOutcome as Outcome,
)
from fault_detector_spot.navigation.base_pose_source import BasePoseSample
from fault_detector_spot.navigation.posture_state_source import PostureState
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
from test_base_movement_executor import (
    FakeGoalHandle, FakePostureStateSource, ManualClock, ManualFuture, ReadyHeight,
)


class WalkingRig:
    def __init__(self):
        self.clock = ManualClock()
        self.x = 1.0
        self.y = 0.0
        self.z = 0.5
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.frozen_stamp = None
        self.goals = []
        self.sends = []
        self.handles = []
        self.results = []
        self.executor = BaseMovementExecutor(
            object(),
            action_client=self,
            height_readiness=ReadyHeight(),
            posture_state_source=FakePostureStateSource(PostureState.STANDING),
            base_pose_source=SimpleNamespace(sample=self.sample),
            monotonic_clock=self.clock,
            ros_time_sec=lambda: 100.0 + self.clock.now,
        )
        target = PoseStamped()
        target.header.frame_id = "odom"
        target.pose.position.x = 1.0
        target.pose.orientation.w = 1.0
        self.target = target
        plan = BaseMovementPlan(target, 0.1, self.executor.walking_profiles.for_move())
        self.executor.motion_planner.resolve_relative = lambda _: plan

    def sample(self):
        stamp = 100.0 + self.clock.now if self.frozen_stamp is None else self.frozen_stamp
        pose = PoseData(
            Vector3Data(self.x, self.y, self.z),
            quaternion_from_euler("xyz", [self.roll, self.pitch, self.yaw]),
        )
        return BasePoseSample(self.x, self.y, self.yaw, stamp, body_pose=pose)

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        native = robot_command_pb2.RobotCommand()
        convert(goal.command, native)
        self.goals.append(native)
        result = ManualFuture()
        handle = FakeGoalHandle(result)
        send = ManualFuture()
        self.results.append(result)
        self.handles.append(handle)
        self.sends.append(send)
        return send

    def accept(self):
        self.sends[-1].set_result(self.handles[-1])
        return self.executor.poll()

    def complete(self, success=True):
        self.results[-1].set_result(SimpleNamespace(
            result=SimpleNamespace(success=success)
        ))
        return self.executor.poll()

    def poll(self, seconds, x=None):
        self.clock.now = seconds
        if x is not None:
            self.x = x
        return self.executor.poll()


def test_walk_waits_for_arrival_then_stand_and_new_stationary_feedback():
    rig = WalkingRig()
    executor = rig.executor
    assert executor.relative(object()).outcome is Outcome.RUNNING
    rig.accept()
    rig.x = 0.4
    # Driver success must not cut off the remaining physical approach.
    assert rig.complete().outcome is Outcome.RUNNING
    assert rig.poll(1.0, 0.8).outcome is Outcome.RUNNING
    assert rig.poll(2.0, 1.0).outcome is Outcome.RUNNING
    assert len(rig.goals) == 1
    assert rig.poll(2.6).outcome is Outcome.RUNNING
    assert len(rig.goals) == 2
    stand = rig.goals[-1].synchronized_command
    assert stand.mobility_command.HasField("stand_request")
    assert not stand.HasField("arm_command")
    assert not stand.HasField("gripper_command")
    assert executor.relative(object()).outcome is Outcome.BUSY

    rig.accept()
    assert rig.complete().outcome is Outcome.RUNNING
    # Even a recently stamped pre-stand sample cannot confirm the handoff.
    rig.frozen_stamp = 102.6
    assert rig.poll(2.8).outcome is Outcome.RUNNING
    assert rig.poll(3.0).outcome is Outcome.RUNNING
    rig.frozen_stamp = None
    assert rig.poll(3.1).outcome is Outcome.RUNNING
    assert rig.poll(3.7).outcome is Outcome.SUCCESS
    assert not executor.active
    assert len(rig.goals) == 2


def test_stand_displacement_is_checked_against_original_goal_before_success():
    rig = WalkingRig()
    rig.executor.relative(object())
    rig.accept()
    rig.complete()
    rig.poll(0.6)
    rig.accept()
    rig.complete()
    assert rig.poll(0.7, 1.1).outcome is Outcome.RUNNING
    assert rig.poll(1.3).outcome is Outcome.RUNNING
    assert rig.poll(5.7).outcome is Outcome.RUNNING
    # A correction walks to the original endpoint, not the displaced stand pose.
    assert len(rig.goals) == 3
    correction = rig.goals[-1].synchronized_command.mobility_command
    assert correction.HasField("se2_trajectory_request")
    assert correction.se2_trajectory_request.trajectory.points[-1].pose.position.x == 1.0


def test_small_turn_holds_achieved_lean_and_yaw_without_recentring_or_retry():
    rig = WalkingRig()
    rig.yaw = math.radians(10.0)
    rig.roll = math.radians(-2.0)
    rig.pitch = math.radians(4.0)
    rig.z = 0.47
    rig.y = -0.02
    rig.target.pose.position.y = rig.y
    yaw = quaternion_from_euler("z", rig.yaw)
    rig.target.pose.orientation.z = yaw.z
    rig.target.pose.orientation.w = yaw.w

    rig.executor.relative(object())
    rig.accept()
    rig.complete()
    assert rig.poll(0.6).outcome is Outcome.RUNNING
    params = spot_command_pb2.MobilityParams()
    assert rig.goals[-1].synchronized_command.mobility_command.params.Unpack(params)
    assert params.body_control.WhichOneof("param") == "body_pose"
    hold = params.body_control.body_pose
    assert hold.root_frame_name == "odom"
    pose = hold.base_offset_rt_root.points[0].pose
    expected = rig.sample().body_pose
    for axis in ("x", "y", "z"):
        assert getattr(pose.position, axis) == pytest.approx(getattr(expected.position, axis))
    for axis in ("x", "y", "z", "w"):
        assert getattr(pose.rotation, axis) == pytest.approx(getattr(expected.orientation, axis))
    assert not params.obstacle_params.disable_vision_body_obstacle_avoidance

    rig.accept()
    rig.complete()
    assert rig.poll(0.7).outcome is Outcome.RUNNING
    assert rig.poll(1.3).outcome is Outcome.SUCCESS
    assert len(rig.goals) == 2


@pytest.mark.parametrize("stamp", [99.0, 100.1])
def test_pose_hold_rejects_stale_or_future_measurement_without_recentering(stamp):
    rig = WalkingRig()
    rig.frozen_stamp = stamp
    update = rig.executor.finish_walking()
    assert update.outcome is Outcome.EXECUTION_ERROR
    assert "fresh full odom-to-body pose" in update.detail
    assert rig.goals == []


def test_pose_hold_requires_full_geometry_not_only_planar_feedback():
    rig = WalkingRig()
    rig.executor.base_pose_source.sample = lambda: BasePoseSample(1.0, 0.0, 0.0, 100.0)
    update = rig.executor.finish_walking()
    assert update.outcome is Outcome.EXECUTION_ERROR
    assert rig.goals == []


def test_finish_navigation_confirms_settling_at_any_world_position():
    rig = WalkingRig()
    rig.x = 42.0
    assert rig.executor.finish_walking().outcome is Outcome.RUNNING
    rig.accept()
    assert rig.complete().outcome is Outcome.RUNNING
    assert rig.poll(0.1).outcome is Outcome.RUNNING
    assert rig.poll(0.7).outcome is Outcome.SUCCESS
    assert not rig.executor.active
    assert len(rig.goals) == 1


@pytest.mark.parametrize("failure", ["rejected", "failed", "stale", "moving"])
def test_failed_stationary_handoff_cannot_report_success(failure):
    rig = WalkingRig()
    rig.executor.finish_walking()
    if failure == "rejected":
        rig.handles[-1].accepted = False
        update = rig.accept()
        assert update.outcome is Outcome.GOAL_REJECTED
    else:
        rig.accept()
        update = rig.complete(success=failure != "failed")
        if failure == "stale":
            rig.frozen_stamp = 100.0
            update = rig.poll(5.1)
        elif failure == "moving":
            for second in range(1, 6):
                update = rig.poll(float(second), float(second))
        assert update.outcome is Outcome.MOTION_FAILED
    assert not rig.executor.active


def test_cancel_stationary_stand_retains_ownership_until_goal_terminates():
    rig = WalkingRig()
    rig.executor.finish_walking()
    rig.accept()
    rig.executor.cancel()
    assert rig.handles[-1].cancel_calls == 1
    assert rig.executor.finish_walking().outcome is Outcome.BUSY
    rig.results[-1].set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert not rig.executor.active
