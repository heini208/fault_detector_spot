"""Verify height changes and bounded yaw recovery without robot connections."""

from types import SimpleNamespace

import pytest
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from geometry_msgs.msg import TransformStamped

from fault_detector_spot.navigation.base_correction_policy import (
    BaseCorrectionConfig,
    BaseCorrectionPolicy,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor,
    BaseMovementOutcome,
)
from fault_detector_spot.navigation.body_height_readiness import (
    BodyHeightReadiness,
    BodyHeightSample,
)
from fault_detector_spot.navigation.posture_state_source import PostureState
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
from test_base_goal_corrections import Client
from test_base_movement_executor import FakePostureStateSource, ManualClock
from test_body_height import native_goal


class HeightRig:
    def __init__(self, maximum_attempts=2):
        self.clock = ManualClock()
        self.client = Client()
        self.transform = TransformStamped()
        self.available = True
        self.set_pose(x=0.3, y=-0.2, z=0.5, yaw=0.4)
        readiness = BodyHeightReadiness(SimpleNamespace(sample=lambda: BodyHeightSample(
            self.transform.transform.translation.z, 100.0 + self.clock.now,
        )))
        readiness.nominal_height_m = 0.5
        self.executor = BaseMovementExecutor(
            tf_listener=SimpleNamespace(lookup_a_tform_b=self.lookup),
            action_client=self.client,
            posture_state_source=FakePostureStateSource(PostureState.STANDING),
            monotonic_clock=self.clock,
            ros_time_sec=lambda: 100.0 + self.clock.now,
            height_readiness=readiness,
            correction_policy=BaseCorrectionPolicy(BaseCorrectionConfig(
                maximum_attempts=maximum_attempts,
            )),
        )

    def lookup(self, *_args, **_kwargs):
        if not self.available:
            raise RuntimeError("No body pose")
        return self.transform

    def stamp(self):
        value = 100.0 + self.clock.now
        self.transform.header.stamp.sec = int(value)
        self.transform.header.stamp.nanosec = round((value - int(value)) * 1e9)

    def set_pose(self, x=0.3, y=-0.2, z=0.37, yaw=0.4, roll=0.0, pitch=0.0):
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        self.transform.transform.translation.z = z
        rotation = quaternion_from_euler("xyz", [roll, pitch, yaw])
        for field in ("x", "y", "z", "w"):
            setattr(self.transform.transform.rotation, field, getattr(rotation, field))
        self.stamp()

    def poll(self, advance=0.0, fresh=True):
        self.clock.now += advance
        if fresh:
            self.stamp()
        return self.executor.poll()

    def start(self, height=-0.13):
        update = self.executor.change_height(height)
        assert update.outcome is BaseMovementOutcome.RUNNING
        assert len(self.client.goals) == 1

    def complete(self, success=True):
        self.executor.poll()
        self.client.results[-1].set_result(SimpleNamespace(
            result=SimpleNamespace(success=success),
        ))
        return self.executor.poll()

    def settle(self):
        assert self.poll(0.1).outcome is BaseMovementOutcome.RUNNING
        return self.poll(0.6)

    def mobility(self, index=-1):
        return native_goal(self.client.goals[index]).synchronized_command.mobility_command

    def params(self, index=-1):
        result = spot_command_pb2.MobilityParams()
        assert self.mobility(index).params.Unpack(result)
        return result


def test_height_completion_requires_fresh_post_result_samples_and_settling():
    rig = HeightRig()
    rig.start()
    rig.set_pose(x=0.4, y=-0.3, yaw=0.42)
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    assert rig.poll(0.6, fresh=False).outcome is BaseMovementOutcome.RUNNING
    assert rig.settle().outcome is BaseMovementOutcome.SUCCESS
    assert len(rig.client.goals) == 1
    assert not rig.executor.active
    assert rig.executor._commanded_height_m == pytest.approx(-0.13)


def test_height_completion_requires_xy_settling_even_with_correct_yaw():
    rig = HeightRig()
    rig.start()
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    rig.set_pose(x=0.4)
    assert rig.poll(0.1).outcome is BaseMovementOutcome.RUNNING
    rig.set_pose(x=0.5)
    assert rig.poll(0.6).outcome is BaseMovementOutcome.RUNNING
    assert rig.poll(0.6).outcome is BaseMovementOutcome.SUCCESS
    assert len(rig.client.goals) == 1


def begin_yaw_correction(rig):
    rig.start()
    rig.set_pose(x=0.34, y=-0.23, yaw=0.6)
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    assert rig.settle().outcome is BaseMovementOutcome.RUNNING
    assert len(rig.client.goals) == 2


def test_yaw_correction_uses_current_xy_precision_and_selected_height_then_full_pose_hold():
    rig = HeightRig()
    begin_yaw_correction(rig)
    mobility = rig.mobility()
    target = mobility.se2_trajectory_request.trajectory.points[0].pose
    assert target.position.x == pytest.approx(0.34)
    assert target.position.y == pytest.approx(-0.23)
    assert target.angle == pytest.approx(0.4)
    params = rig.params()
    precision = rig.executor.walking_profiles.precision
    assert params.vel_limit.max_vel.angular == pytest.approx(precision.angular_speed_rad_s)
    assert params.vel_limit.max_vel.linear.x == pytest.approx(precision.relative_speed_mps)
    assert params.locomotion_hint == spot_command_pb2.HINT_SPEED_SELECT_CRAWL
    assert params.body_control.base_offset_rt_footprint.points[0].pose.position.z == pytest.approx(-0.13)

    rig.set_pose(x=0.35, y=-0.22, z=0.37, yaw=0.4, roll=0.1, pitch=-0.08)
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    assert rig.settle().outcome is BaseMovementOutcome.RUNNING
    assert len(rig.client.goals) == 3
    assert rig.mobility().HasField("stand_request")
    body_control = rig.params().body_control
    assert body_control.HasField("body_pose")
    assert not body_control.HasField("base_offset_rt_footprint")
    hold = body_control.body_pose
    assert hold.root_frame_name == "odom"
    pose = hold.base_offset_rt_root.points[0].pose
    for field in ("x", "y", "z"):
        assert getattr(pose.position, field) == pytest.approx(
            getattr(rig.transform.transform.translation, field)
        )
    for field in ("x", "y", "z", "w"):
        assert getattr(pose.rotation, field) == pytest.approx(
            getattr(rig.transform.transform.rotation, field)
        )
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    assert rig.settle().outcome is BaseMovementOutcome.SUCCESS
    assert len(rig.client.goals) == 3
    assert rig.executor._commanded_height_m == pytest.approx(-0.13)
    assert rig.executor.height_readiness.reset_required

    # The correction exception must not leak into ordinary navigation.
    assert rig.executor.prepare_for_navigation().outcome is BaseMovementOutcome.RUNNING
    assert len(rig.client.goals) == 4
    assert rig.params().body_control.base_offset_rt_footprint.points[0].pose.position.z == 0.0


@pytest.mark.parametrize("failure", ["missing", "stale"])
def test_height_change_without_fresh_initial_pose_is_bounded_and_never_submitted(failure):
    rig = HeightRig()
    if failure == "missing":
        rig.available = False
    else:
        rig.transform.header.stamp.sec = 1
    assert rig.executor.change_height(-0.13).outcome is BaseMovementOutcome.RUNNING
    assert rig.client.goals == []
    update = rig.poll(rig.executor.ready_state_timeout_sec + 0.1, fresh=False)
    assert update.outcome is not BaseMovementOutcome.RUNNING
    assert update.outcome is not BaseMovementOutcome.SUCCESS
    assert rig.client.goals == []
    assert not rig.executor.active


def test_stale_post_height_pose_never_reuses_pre_height_yaw_to_succeed_or_correct():
    rig = HeightRig()
    rig.start()
    assert rig.complete().outcome is BaseMovementOutcome.RUNNING
    update = rig.poll(rig.executor.goal_verification_config.timeout_sec + 0.1, fresh=False)
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert len(rig.client.goals) == 1
    assert not rig.executor.active


def test_cancelling_yaw_correction_retains_ownership_until_terminal_without_more_motion():
    rig = HeightRig()
    begin_yaw_correction(rig)
    rig.executor.poll()
    rig.executor.cancel()
    assert rig.client.handles[-1].cancel_calls == 1
    assert rig.executor.active
    assert rig.executor.change_height(0.0).outcome is BaseMovementOutcome.BUSY
    rig.client.results[-1].set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert not rig.executor.active
    rig.poll()
    assert len(rig.client.goals) == 2


@pytest.mark.parametrize("maximum_attempts", [0, 2])
def test_yaw_correction_fails_when_disabled_or_when_heading_makes_no_progress(maximum_attempts):
    rig = HeightRig(maximum_attempts=maximum_attempts)
    rig.start()
    rig.set_pose(yaw=0.6)
    rig.complete()
    update = rig.settle()
    if maximum_attempts:
        assert update.outcome is BaseMovementOutcome.RUNNING
        assert len(rig.client.goals) == 2
        rig.complete()
        rig.settle()
        assert len(rig.client.goals) == 3
        rig.complete()
        update = rig.settle()
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "correction" in update.detail.lower()
    assert len(rig.client.goals) == (3 if maximum_attempts else 1)
    assert not rig.executor.active


def test_yaw_correction_attempt_limit_applies_even_when_heading_improves():
    rig = HeightRig(maximum_attempts=1)
    begin_yaw_correction(rig)
    rig.set_pose(yaw=0.5)
    rig.complete()
    rig.settle()
    assert len(rig.client.goals) == 3
    rig.complete()
    update = rig.settle()
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "attempt limit" in update.detail
    assert len(rig.client.goals) == 3
    assert not rig.executor.active


@pytest.mark.parametrize("stale", [False, True])
def test_yaw_correction_requires_fresh_standing_posture(stale):
    rig = HeightRig()
    rig.start()
    rig.set_pose(yaw=0.6)
    rig.complete()
    rig.executor.posture_state_source.stale = stale
    if not stale:
        rig.executor.posture_state_source.state = PostureState.SITTING
    update = rig.settle()
    expected = (BaseMovementOutcome.POSTURE_STATE_STALE if stale
                else BaseMovementOutcome.POSTURE_STATE_UNKNOWN)
    assert update.outcome is expected
    assert len(rig.client.goals) == 1
    assert not rig.executor.active
