"""Offline contracts for measurement excursions and verified returns."""

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import math

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    SensorTipObservationView,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.sensor_models import sensor_probe_frame
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    SurfaceDistanceSample,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import SensorTipPoseSample
from fault_detector_spot.inspection.setup.reference_view_depth_projection import ImageRegion
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import (
    MeasureSensorTipDistanceCommand,
)
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig,
    MeasureSensorTipDistanceExecution,
    MeasureSensorTipDistanceOutcome,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose, pose_to_pose_data


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(Vector3Data(x, y, z), orientation or QuaternionData.identity())


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


class Planner:
    def __init__(self, clock):
        self.clock = clock
        self.hand = pose()
        self.tip = pose(x=0.06, z=-0.10)
        self.body = pose()

    def current_pose(self, frame, controlled):
        assert frame == "odom"
        value = self.body if controlled == "body" else self.tip
        assert controlled in {"body", sensor_probe_frame("probe")}
        stamped = PoseStamped()
        stamped.header.frame_id = frame
        stamped.header.stamp.sec = int(self.clock.now)
        stamped.header.stamp.nanosec = int((self.clock.now % 1.0) * 1e9)
        stamped.pose = pose_data_to_pose(value)
        return stamped

    def current_hand_pose(self, frame):
        value = self.current_pose(frame, "body")
        value.pose = pose_data_to_pose(self.hand)
        return value


class Executor:
    def __init__(self, clock):
        self.active = False
        self.moveit_arm_planner = object()
        self.probe_motion_planner = Planner(clock)
        self.speed_policy = SimpleNamespace(
            default_speed=SimpleNamespace(angular_speed_rad_s=0.5),
        )
        self.calls = []
        self.outcomes = {}
        self.return_error = Vector3Data.zero()
        self.cancel_count = 0
        self.stop_count = 0
        self.pending = None

    def guarded_probe(self, target, sensor_id, **options):
        assert not self.active
        self.calls.append((deepcopy(target), sensor_id, options))
        self.pending = deepcopy(target)
        self.active = True
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "moving")

    def poll(self):
        assert self.active
        self.active = False
        if self.pending is None:
            return ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop confirmed")
        outcome = self.outcomes.get(len(self.calls), ArmMovementOutcome.SUCCESS)
        if outcome is ArmMovementOutcome.SUCCESS:
            planner = self.probe_motion_planner
            target = pose_to_pose_data(self.pending.pose)
            if len(self.calls) == 2:
                target.position.x += self.return_error.x
                target.position.y += self.return_error.y
                target.position.z += self.return_error.z
            for axis in ("x", "y", "z"):
                delta = getattr(target.position, axis) - getattr(planner.hand.position, axis)
                setattr(planner.tip.position, axis, getattr(planner.tip.position, axis) + delta)
            planner.hand = target
        self.pending = None
        return ArmMovementUpdate(outcome, outcome.value)

    def cancel(self):
        self.cancel_count += 1
        self.active = False
        self.pending = None

    def confirm_stop(self):
        self.stop_count += 1
        self.active = True
        self.pending = None
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "confirming stop")


class Source:
    def __init__(self, executor, clock, visible):
        self.executor = executor
        self.clock = clock
        self.initially_visible = visible
        self.attachment = ("probe", 1)
        self.translation = Vector3Data(0.0, 0.10, 0.0)
        self.sample_error = None
        self.snapshot_error = None
        self.sample_calls = []
        self.view_calls = []
        self.snapshot_calls = []
        self.pose_sample_calls = []
        self.normal = Vector3Data(-1.0, 0.0, 0.0)

    def active_attachment(self):
        return self.attachment

    def sensor_tip_pose_snapshot(self, sensor_id, *, receipt_not_before=0.0):
        assert sensor_id == "probe"
        self.snapshot_calls.append(receipt_not_before)
        if self.snapshot_error:
            raise ValueError(self.snapshot_error)
        planner = self.executor.probe_motion_planner
        return deepcopy((planner.hand, planner.tip, planner.body))

    def pose_acquisition_cutoff(self, delay_sec=0.0):
        return self.clock.now + delay_sec

    def sensor_tip_pose_sample(
        self, sensor_id, *, receipt_not_before=0.0, acquisition_not_before=0.0,
        after_stamp_seconds=0.0,
    ):
        self.pose_sample_calls.append((receipt_not_before, acquisition_not_before))
        if self.clock.now < acquisition_not_before:
            raise ValueError("No post-settle acquisition is available")
        hand, probe, body = self.sensor_tip_pose_snapshot(
            sensor_id, receipt_not_before=receipt_not_before,
        )
        return SensorTipPoseSample(self.clock.now, hand, probe, body)

    def observation_view(self, reference, expected_distance_m=None, *, receipt_not_before=0.0):
        self.view_calls.append((deepcopy(reference), expected_distance_m, receipt_not_before))
        return SensorTipObservationView(
            visible=self.initially_visible or len(self.executor.calls) > 0,
            camera_pose_odom=pose(),
            hand_translation_odom=deepcopy(self.translation),
            detail="target in view" if self.initially_visible else "target outside image",
        )

    def sensor_tip_distance_samples(self, reference, expected_distance_m, **options):
        self.sample_calls.append((deepcopy(reference), expected_distance_m, options))
        if self.sample_error:
            raise ValueError(self.sample_error)
        plane = SurfacePlane(
            point=Vector3Data(0.25, 0.0, 0.0), normal=self.normal,
            frame_id="probe", inlier_count=100, sample_count=100,
            inlier_ratio=1.0, rmse_m=0.001,
        )
        distance = plane.signed_distance(Vector3Data.zero())
        return tuple(
            SurfaceDistanceSample(
                distance_m=distance, stamp_seconds=self.clock.now - 1.0 + index * 0.25,
                frame_id="hand_color_image_sensor", sample_count=100,
                valid_pixel_ratio=1.0, spread_m=0.001,
                source_region=ImageRegion(x=100, y=100, width=31, height=31),
                surface_plane_probe=plane,
            )
            for index in range(5)
        )


def harness(*, visible=True, **config):
    clock = Clock()
    executor = Executor(clock)
    source = Source(executor, clock, visible)
    execution = MeasureSensorTipDistanceExecution(
        executor, source, MeasureSensorTipDistanceConfig(**config),
        monotonic_clock=clock,
    )
    command = MeasureSensorTipDistanceCommand(
        CommandID.MEASURE_SENSOR_TIP_DISTANCE, object(), expected_distance_m=0.25,
    )
    return SimpleNamespace(
        clock=clock, executor=executor, source=source, execution=execution,
        command=command,
    )


def finish(case, outcome=None):
    outcome = outcome or case.execution.start(case.command)
    for _ in range(400):
        if outcome is not MeasureSensorTipDistanceOutcome.RUNNING:
            return outcome
        case.clock.now += 0.25
        outcome = case.execution.poll()
    pytest.fail("Measurement execution did not finish")


def begin_excursion(case):
    outcome = case.execution.start(case.command)
    for _ in range(40):
        if case.executor.calls:
            return outcome
        assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
        case.clock.now += 0.25
        outcome = case.execution.poll()
    pytest.fail("Observation movement was not requested")


def test_visible_target_measures_without_moving():
    case = harness()

    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert case.executor.calls == []
    assert case.execution.result.distance_m == pytest.approx(0.25)
    assert not case.execution.active
    assert case.source.view_calls[0][1] is None
    assert case.source.sample_calls[0][1] is None


def test_excursion_is_guarded_collision_checked_and_returns_to_saved_hand_pose():
    case = harness(visible=False)
    original = deepcopy(case.executor.probe_motion_planner.hand)

    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 2
    outward, returning = case.executor.calls
    assert outward[0].header.frame_id == returning[0].header.frame_id == "odom"
    assert outward[0].pose.position.y == pytest.approx(0.10)
    assert returning[0].pose == pose_data_to_pose(original)
    for _target, _sensor, options in case.executor.calls:
        assert options["ignore_environment_collisions"] is False
        assert options["cartesian_path"] is True
    saved_target = case.source.sample_calls[0][0]
    assert saved_target.position.y == 0.0
    assert saved_target.position.z == -0.10


def test_distance_uses_achieved_return_pose_instead_of_assuming_exact_return():
    case = harness(visible=False)
    case.executor.return_error.x = 0.003

    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert case.execution.result.distance_m == pytest.approx(0.247)


def test_failed_measurement_returns_before_reporting_failure():
    case = harness(visible=False)
    case.source.sample_error = "target patch has no depth"

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.executor.probe_motion_planner.hand == pose()
    assert "target patch has no depth" in case.execution.feedback_message
    assert case.execution.result is None


def test_excessive_return_error_invalidates_result_and_prevents_further_motion():
    case = harness(visible=False)
    case.executor.return_error.y = 0.021

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED


def test_failed_return_prohibits_outer_workflow_recovery():
    case = harness(visible=False)
    case.executor.outcomes[2] = ArmMovementOutcome.MOTION_FAILED

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED


@pytest.mark.parametrize("outcome", [
    ArmMovementOutcome.STOP_UNCONFIRMED,
    ArmMovementOutcome.RETREAT_FAILED,
    ArmMovementOutcome.UNSTABLE_ARM,
])
def test_unsafe_observation_failure_never_commands_return(outcome):
    case = harness(visible=False)
    case.executor.outcomes[1] = outcome

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 1
    assert case.execution.failure_outcome is outcome
    assert case.execution.result is None


def test_user_cancel_stops_without_automatic_return():
    case = harness(visible=False)
    begin_excursion(case)

    case.execution.cancel()

    assert case.executor.cancel_count == 1
    assert len(case.executor.calls) == 1
    assert not case.execution.active
    assert case.execution.result is None


def test_missing_collision_planner_rejects_excursion():
    case = harness(visible=False)
    case.executor.moveit_arm_planner = None

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert "MoveIt" in case.execution.feedback_message


def test_excessive_observation_translation_is_rejected_before_motion():
    case = harness(visible=False)
    case.source.translation.y = 0.201

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []


@pytest.mark.parametrize("change", ["attachment", "body_position", "body_orientation"])
def test_changed_reference_during_excursion_invalidates_measurement_without_return(change):
    case = harness(visible=False)
    outcome = begin_excursion(case)
    if change == "attachment":
        case.source.attachment = ("probe", 2)
    elif change == "body_position":
        case.executor.probe_motion_planner.body.position.x = 0.011
    else:
        case.executor.probe_motion_planner.body.orientation = quaternion_from_euler(
            "z", math.radians(2.1),
        )

    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 1
    assert case.execution.result is None
    assert case.execution.failure_outcome in {
        ArmMovementOutcome.RECOVERY_FAILED, ArmMovementOutcome.UNSTABLE_ARM,
    }


def test_measurement_reports_misalignment_for_the_approach_owner_to_validate():
    case = harness(visible=False)
    angle = math.radians(9.0)
    case.source.normal = Vector3Data(-math.cos(angle), math.sin(angle), 0.0)

    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 2
    assert case.execution.result.axis_error_rad == pytest.approx(angle)
    assert case.execution.result.axial_distance_m == pytest.approx(0.25)
    assert case.execution.result.distance_m == pytest.approx(0.25 * math.cos(angle))


def test_missing_fresh_initial_snapshot_never_moves():
    case = harness(visible=False)
    case.source.snapshot_error = "fresh depth timestamp is unavailable"

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert case.execution.result is None
    assert "fresh depth" in case.execution.feedback_message


def test_failed_observation_confirms_stop_before_return():
    case = harness(visible=False)
    case.executor.outcomes[1] = ArmMovementOutcome.MOTION_FAILED

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.stop_count == 1
    assert len(case.executor.calls) == 2
    assert case.source.sample_calls == []
    assert "Observation movement failed" in case.execution.feedback_message


def test_return_requires_fresh_post_return_snapshot():
    case = harness(visible=False)
    snapshot = case.source.sensor_tip_pose_snapshot

    def fail_after_return(*args, **kwargs):
        if len(case.executor.calls) == 2:
            raise ValueError("post-return timestamped transform unavailable")
        return snapshot(*args, **kwargs)

    case.source.sensor_tip_pose_snapshot = fail_after_return

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED
    assert "post-return timestamped transform" in case.execution.feedback_message


def test_fresh_measurement_and_return_snapshots_start_after_movement_settles():
    case = harness(visible=False)

    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    initial_receipt = case.source.snapshot_calls[0]
    observation_receipt = case.source.sample_calls[0][2]["receipt_not_before"]
    return_receipt = case.source.snapshot_calls[-1]
    assert initial_receipt < observation_receipt < return_receipt
    result = case.execution.result
    assert result.surface_plane_odom.frame_id == "odom"
    assert result.surface_plane_odom.point.x == pytest.approx(0.31)
    assert result.surface_plane_odom.point.z == pytest.approx(-0.10)
    assert result.plane_at_probe(result.returned_probe_pose).point.x == pytest.approx(0.25)


@pytest.mark.parametrize("unavailable", ["body_transform", "attachment"])
def test_unknown_return_reference_after_observation_fails_without_another_motion(unavailable):
    case = harness(visible=False)
    begin_excursion(case)
    case.clock.now += 0.25
    assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
    assert not case.executor.active

    def missing(*_args, **_kwargs):
        raise ValueError("return reference unavailable")

    if unavailable == "body_transform":
        case.executor.probe_motion_planner.current_pose = missing
    else:
        case.source.active_attachment = missing

    assert case.execution.poll() is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 1
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED


def alternating_normals(samples):
    """Hold perpendicular range constant while plane orientation changes."""
    result = []
    for index, sample in enumerate(samples):
        angle = math.radians(9.0 if index % 2 else 0.0)
        plane = replace(
            sample.surface_plane_probe,
            point=Vector3Data(sample.distance_m / math.cos(angle), 0.0, 0.0),
            normal=Vector3Data(-math.cos(angle), math.sin(angle), 0.0),
        )
        result.append(replace(sample, surface_plane_probe=plane))
    return tuple(result)


@pytest.mark.parametrize("visible", [True, False])
def test_stable_ranges_with_unstable_normals_cannot_produce_a_measurement(visible):
    case = harness(visible=visible)
    read_samples = case.source.sensor_tip_distance_samples
    case.source.sensor_tip_distance_samples = lambda *args, **kwargs: alternating_normals(
        read_samples(*args, **kwargs),
    )

    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.execution.result is None
    assert "surface normals are unstable" in case.execution.feedback_message
    assert len(case.executor.calls) == (0 if visible else 2)


def test_old_unstable_normals_do_not_poison_a_later_stable_window():
    case = harness()
    read_samples = case.source.sensor_tip_distance_samples
    case.source.sensor_tip_distance_samples = lambda *args, **kwargs: alternating_normals(
        read_samples(*args, **kwargs),
    )
    outcome = case.execution.start(case.command)
    for _ in range(20):
        if "surface normals are unstable" in case.execution.feedback_message:
            break
        case.clock.now += 0.25
        outcome = case.execution.poll()
        assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
    assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
    assert "surface normals are unstable" in case.execution.feedback_message
    case.source.sensor_tip_distance_samples = read_samples

    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert case.execution.result.distance_m == pytest.approx(0.25)
    assert case.executor.calls == []


def test_busy_executor_rejects_measurement_without_cancelling_other_owner():
    case = harness()
    case.executor.active = True
    assert case.execution.start(case.command) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.execution.failure_outcome is ArmMovementOutcome.BUSY
    assert case.executor.active
    assert case.executor.cancel_count == 0
    assert case.executor.calls == []


def test_sparse_first_post_observation_frame_is_retried_before_return():
    case = harness(visible=False)
    read_view = case.source.observation_view
    sparse_frames = []

    def transient_sparse(*args, **kwargs):
        view = read_view(*args, **kwargs)
        if len(case.executor.calls) == 1 and not sparse_frames:
            sparse_frames.append(case.clock.now)
            return replace(view, visible=False, detail="12% < 60%")
        return view

    case.source.observation_view = transient_sparse
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(sparse_frames) == 1
    assert len(case.executor.calls) == 2
    assert case.source.sample_calls


def test_persistent_sparse_depth_waits_sampling_budget_then_returns_without_extra_search():
    case = harness(visible=False)
    read_view = case.source.observation_view
    sparse_times = []

    def persistent_sparse(*args, **kwargs):
        view = read_view(*args, **kwargs)
        if len(case.executor.calls) == 1:
            sparse_times.append(case.clock.now)
            return replace(view, visible=False, detail="12% < 60%")
        return view

    case.source.observation_view = persistent_sparse
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert sparse_times[-1] - sparse_times[0] >= case.execution.config.sample_timeout_sec
    assert len(case.executor.calls) == 2
    assert not case.source.sample_calls
    assert "12% < 60%" in case.execution.feedback_message
    assert "returned to original pose" in case.execution.feedback_message


def test_in_view_but_sparse_target_retries_in_place_without_centering_move():
    case = harness(visible=False)
    case.source.translation = Vector3Data.zero()
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert len(case.source.view_calls) > 1
    assert "no reliable depth" in case.execution.feedback_message
