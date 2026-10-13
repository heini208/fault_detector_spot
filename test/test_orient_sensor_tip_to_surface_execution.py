"""Offline contracts for measuring then orienting the actual sensor tip."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.geometry.pose import (
    compose_poses, inverse_pose, relative_pose,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome, ArmMovementUpdate,
)
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceExecution,
    MeasureSensorTipDistanceOutcome,
    SensorTipDistanceResult,
)
from fault_detector_spot.manipulation.orient_sensor_tip_to_surface_execution import (
    OrientSensorTipToSurfaceExecution,
)
from fault_detector_spot.shared.geometry.models import Vector3Data
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler, rotate_vector,
)
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose, pose_to_pose_data
from test_measure_sensor_tip_distance_execution import Clock, Executor, Source, pose


class AlignmentExecutor(Executor):
    def __init__(self, clock):
        super().__init__(clock)
        self.position_error = Vector3Data.zero()
        self.keep_original_orientation = False
        self.probe_motion_planner.hand_to_probe_pose = self.hand_to_probe_pose

    def hand_to_probe_pose(self, sensor_id):
        assert sensor_id == "probe"
        planner = self.probe_motion_planner
        return pose_data_to_pose(relative_pose(planner.hand, planner.tip))

    def poll(self):
        assert self.active
        self.active = False
        outcome = self.outcomes.get(len(self.calls), ArmMovementOutcome.SUCCESS)
        if outcome is ArmMovementOutcome.SUCCESS:
            planner = self.probe_motion_planner
            hand_to_probe = relative_pose(planner.hand, planner.tip)
            target = pose_to_pose_data(self.pending.pose)
            sensor_id = self.calls[-1][1]
            assert sensor_id == "probe"
            if self.keep_original_orientation:
                target.orientation = deepcopy(planner.tip.orientation)
            for axis in ("x", "y", "z"):
                setattr(target.position, axis,
                        getattr(target.position, axis) + getattr(self.position_error, axis))
            planner.tip = target
            planner.hand = compose_poses(target, inverse_pose(hand_to_probe))
        self.pending = None
        return ArmMovementUpdate(outcome, outcome.value)


class Measurement:
    def __init__(self, result):
        self.result = result
        self.active = False
        self.starts = []
        self.cancel_count = 0
        self.feedback_message = "Measuring the intended surface"
        self.failure_outcome = ArmMovementOutcome.EXECUTION_ERROR
        self.outcome = MeasureSensorTipDistanceOutcome.SUCCESS
        self.wait = False

    def start(self, command):
        self.starts.append(command)
        self.active = True
        return MeasureSensorTipDistanceOutcome.RUNNING

    def poll(self):
        if self.wait:
            return MeasureSensorTipDistanceOutcome.RUNNING
        self.active = False
        return self.outcome

    def cancel(self):
        self.cancel_count += 1
        self.active = False


def harness(*, angle_deg=9.0, **config):
    clock = Clock()
    executor = AlignmentExecutor(clock)
    source = Source(executor, clock, visible=True)
    angle = math.radians(angle_deg)
    plane = SurfacePlane(
        point=Vector3Data(0.31, 0.0, -0.10),
        normal=Vector3Data(-math.cos(angle), math.sin(angle), 0.0),
        frame_id="odom", inlier_count=100, sample_count=100,
        inlier_ratio=1.0, rmse_m=0.001,
    )
    measured = SensorTipDistanceResult(
        surface_plane_odom=plane,
        returned_probe_pose=deepcopy(executor.probe_motion_planner.tip),
        reference_body_pose=deepcopy(executor.probe_motion_planner.body),
        distance_m=0.25 * math.cos(angle), axial_distance_m=0.25,
        axis_error_rad=angle, sample_count=5, sample_span_sec=1.0,
        peak_to_peak_m=0.001,
    )
    execution = OrientSensorTipToSurfaceExecution(
        executor, source, MeasureSensorTipDistanceConfig(**config),
        monotonic_clock=clock,
    )
    measurement = Measurement(measured)
    execution._measurement = measurement
    command = SimpleNamespace(motion_sensor_id="probe", expected_distance_m=0.25)
    return SimpleNamespace(
        clock=clock, executor=executor, source=source, execution=execution,
        measurement=measurement, command=command,
    )


def finish(case, update=None):
    update = update or case.execution.start(case.command)
    for _ in range(400):
        if update.outcome is not ArmMovementOutcome.RUNNING:
            return update
        case.clock.now += 0.25
        update = case.execution.poll()
    pytest.fail("Sensor tip orientation execution did not finish")


def begin_alignment(case):
    update = case.execution.start(case.command)
    for _ in range(40):
        if case.executor.calls:
            return update
        assert update.outcome is ArmMovementOutcome.RUNNING
        case.clock.now += 0.25
        update = case.execution.poll()
    pytest.fail("Sensor tip alignment motion was not requested")


def test_measurement_finishes_before_alignment_starts():
    case = harness()
    case.measurement.wait = True
    update = case.execution.start(case.command)
    for _ in range(3):
        assert update.outcome is ArmMovementOutcome.RUNNING
        case.clock.now += 0.25
        update = case.execution.poll()
    assert case.executor.calls == []
    assert len(case.measurement.starts) == 1
    assert case.measurement.starts[0].expected_distance_m == 0.25
    case.measurement.wait = False
    assert finish(case, update).outcome is ArmMovementOutcome.SUCCESS


@pytest.mark.parametrize("angle_deg", [3.0, 9.0])
def test_alignment_faces_measured_normal_at_returned_tip_with_collision_and_force_guards(angle_deg):
    case = harness(angle_deg=angle_deg)
    returned = deepcopy(case.measurement.result.returned_probe_pose)
    assert finish(case).outcome is ArmMovementOutcome.SUCCESS
    assert len(case.executor.calls) == 1
    target, sensor_id, options = case.executor.calls[0]
    assert sensor_id == "probe"
    assert target.header.frame_id == "odom"
    target_pose = pose_to_pose_data(target.pose)
    assert target_pose.position == returned.position
    forward = rotate_vector(target_pose.orientation, Vector3Data(1.0, 0.0, 0.0))
    normal = case.measurement.result.surface_plane_odom.normal
    assert (forward.x, forward.y, forward.z) == pytest.approx(
        (-normal.x, -normal.y, -normal.z), abs=1e-8,
    )
    assert options["ignore_environment_collisions"] is False
    assert options["cartesian_path"] is True
    assert options["force_threshold_n"] == case.execution.config.force_contact_threshold_n
    assert not case.execution.active


def test_alignment_uses_actual_returned_tip_after_measurement_excursion():
    case = harness()
    returned = deepcopy(case.executor.probe_motion_planner.tip)
    returned.position.y += 0.003
    case.measurement.result = replace(case.measurement.result, returned_probe_pose=returned)
    case.executor.probe_motion_planner.tip = deepcopy(returned)
    case.executor.probe_motion_planner.hand.position.y += 0.003
    assert finish(case).outcome is ArmMovementOutcome.SUCCESS
    assert case.executor.calls[0][0].pose.position.y == pytest.approx(0.003)


@pytest.mark.parametrize("outcome", [
    ArmMovementOutcome.EXECUTION_ERROR, ArmMovementOutcome.RECOVERY_FAILED,
    ArmMovementOutcome.STOP_UNCONFIRMED,
])
def test_failed_measurement_never_starts_alignment(outcome):
    case = harness()
    case.measurement.outcome = MeasureSensorTipDistanceOutcome.FAILURE
    case.measurement.failure_outcome = outcome
    case.measurement.feedback_message = "Measurement or return failed"
    update = finish(case)
    assert update.outcome is outcome
    assert "failed" in update.detail.lower()
    assert case.executor.calls == []
    assert not case.execution.active


def test_missing_collision_planner_rejects_alignment():
    case = harness()
    case.executor.moveit_arm_planner = None
    update = finish(case)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert "MoveIt" in update.detail
    assert case.executor.calls == []


def test_already_aligned_probe_finishes_without_an_alignment_movement():
    case = harness(angle_deg=0.0)
    assert finish(case).outcome is ArmMovementOutcome.SUCCESS
    assert case.executor.calls == []


def test_busy_executor_does_not_cancel_another_owner():
    case = harness()
    case.executor.active = True
    update = case.execution.start(case.command)
    assert update.outcome is ArmMovementOutcome.BUSY
    assert case.executor.active
    assert case.executor.cancel_count == 0
    assert case.measurement.starts == []


@pytest.mark.parametrize("change", ["attachment", "body_position", "body_orientation"])
def test_changed_reference_during_alignment_stops_without_recovery_motion(change):
    case = harness()
    update = begin_alignment(case)
    if change == "attachment":
        case.source.attachment = ("probe", 2)
    elif change == "body_position":
        case.executor.probe_motion_planner.body.position.x = 0.011
    else:
        case.executor.probe_motion_planner.body.orientation = quaternion_from_euler(
            "z", math.radians(2.1),
        )
    update = finish(case, update)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert len(case.executor.calls) == 1
    assert case.executor.cancel_count == 1
    assert not case.execution.active


def test_cancel_during_measurement_propagates_without_alignment():
    case = harness()
    case.measurement.wait = True
    case.execution.start(case.command)
    case.execution.cancel()
    assert case.measurement.cancel_count == 1
    assert case.executor.calls == []
    assert not case.execution.active


def test_cancel_during_alignment_stops_without_return_or_another_motion():
    case = harness()
    begin_alignment(case)
    case.execution.cancel()
    assert case.executor.cancel_count == 1
    assert len(case.executor.calls) == 1
    assert not case.execution.active


@pytest.mark.parametrize("outcome", [
    ArmMovementOutcome.CONTACT, ArmMovementOutcome.PLANNING_FAILED,
    ArmMovementOutcome.STOP_UNCONFIRMED,
])
def test_failed_or_contact_alignment_never_reports_success_or_retries(outcome):
    case = harness()
    case.executor.outcomes[1] = outcome
    update = finish(case)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert len(case.executor.calls) == 1
    assert not case.execution.active


@pytest.mark.parametrize("error", ["tip_position", "axis_alignment"])
def test_inaccurate_achieved_alignment_fails_without_blind_retry(error):
    case = harness()
    if error == "tip_position":
        case.executor.position_error.y = 0.006
    else:
        case.executor.keep_original_orientation = True
    update = finish(case)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert len(case.executor.calls) == 1
    assert not case.execution.active


def test_alignment_result_waits_for_fresh_post_motion_snapshot():
    case = harness()
    snapshot = case.source.sensor_tip_pose_snapshot

    def missing_after_motion(*args, **kwargs):
        if case.executor.calls and not case.executor.active:
            raise ValueError("fresh post-alignment timestamped transform unavailable")
        return snapshot(*args, **kwargs)

    case.source.sensor_tip_pose_snapshot = missing_after_motion
    update = finish(case)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert "fresh post-alignment" in update.detail
    assert len(case.executor.calls) == 1


def test_alignment_verification_only_accepts_snapshot_after_settle_time():
    case = harness()
    begin_alignment(case)
    case.clock.now += 0.25
    completed_at = case.clock.now
    update = case.execution.poll()
    assert update.outcome is ArmMovementOutcome.RUNNING
    assert finish(case, update).outcome is ArmMovementOutcome.SUCCESS
    assert case.source.snapshot_calls[-1] >= completed_at + case.execution.config.settle_sec


def test_shared_distance_execution_supplies_plane_before_alignment():
    case = harness()
    angle = math.radians(9.0)
    case.source.normal = Vector3Data(-math.cos(angle), math.sin(angle), 0.0)
    case.execution._measurement = MeasureSensorTipDistanceExecution(
        case.executor, case.source, case.execution.config,
        monotonic_clock=case.clock,
    )
    update = finish(case)
    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert case.source.sample_calls
    assert len(case.executor.calls) == 1
    forward = rotate_vector(
        case.executor.probe_motion_planner.tip.orientation,
        Vector3Data(1.0, 0.0, 0.0),
    )
    assert (forward.x, forward.y, forward.z) == pytest.approx(
        (math.cos(angle), -math.sin(angle), 0.0), abs=1e-8,
    )


@pytest.mark.parametrize("angle_deg", [0.0, 9.0])
@pytest.mark.parametrize("change", ["position", "orientation"])
def test_changed_tip_since_measurement_cannot_use_stale_surface_result(angle_deg, change):
    case = harness(angle_deg=angle_deg)
    if change == "position":
        case.executor.probe_motion_planner.tip.position.y += 0.006
    else:
        case.executor.probe_motion_planner.tip.orientation = quaternion_from_euler(
            "z", math.radians(2.1),
        )
    update = finish(case)
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert case.executor.calls == []
    assert not case.execution.active


def test_attachment_roll_is_preserved_while_hand_stays_upright():
    case = harness()
    calibrated = deepcopy(case.executor.probe_motion_planner.tip)
    calibrated.orientation = quaternion_from_euler("x", math.radians(30.0))
    case.executor.probe_motion_planner.tip = deepcopy(calibrated)
    case.measurement.result = replace(case.measurement.result, returned_probe_pose=calibrated)
    assert finish(case).outcome is ArmMovementOutcome.SUCCESS
    hand_up = rotate_vector(
        case.executor.probe_motion_planner.hand.orientation,
        Vector3Data(0.0, 0.0, 1.0),
    )
    assert (hand_up.x, hand_up.y, hand_up.z) == pytest.approx((0.0, 0.0, 1.0), abs=1e-8)
    assert case.executor.probe_motion_planner.tip.position == calibrated.position
