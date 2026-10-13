"""Observe the original probe target, return, and report its measured geometry."""

from copy import deepcopy
from dataclasses import dataclass, fields
from enum import Enum
import math
import time

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.geometry.pose import inverse_pose
from fault_detector_spot.inspection.geometry.sensor_tip_observation import transform_surface_plane
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.sensor_models import BARE_HAND_MOTION_ID
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    aggregate_surface_distance_samples, _recent_sample_window,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    MAX_HAND_DEPTH_AGE_SEC, NoNewSensorTipPoseSample,
)
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.pose_precision import PoseCorrectionBudget, PoseStabilityWindow
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import rotation_distance_rad
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose, pose_to_pose_data


class MeasureSensorTipDistanceOutcome(Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


class _ReturnPoseMismatch(ValueError):
    """A fresh, valid return snapshot missed the original hand or tip pose."""

    def __init__(self, errors, diagnostics, target_label="Original pose"):
        self.position_error_m = max(position for _, position, _ in errors)
        self.orientation_error_rad = max(angle for _, _, angle in errors)
        details = "; ".join(
            f"{name} {position*1000:.2f} mm, {math.degrees(angle):.2f} deg error"
            for name, position, angle in errors
        )
        super().__init__(f"{target_label} not restored: {details}; {diagnostics}")


@dataclass(frozen=True)
class MeasureSensorTipDistanceConfig:
    # Shared with close_surface so tests and approaches use identical quality gates.
    settle_sec: float = 0.5
    sample_timeout_sec: float = 6.0
    minimum_surface_samples: int = 5
    minimum_surface_span_sec: float = 1.0
    surface_stability_tolerance_m: float = 0.005
    maximum_axis_error_rad: float = math.radians(5.0)
    maximum_observation_translation_m: float = 0.20
    observation_speed_mps: float = 0.020
    return_position_tolerance_m: float = 0.010
    return_orientation_tolerance_rad: float = math.radians(2.0)
    tip_alignment_position_tolerance_m: float = 0.005
    return_correction_speed_scale: float = 0.25
    maximum_return_corrections: int = 3
    minimum_return_correction_improvement_fraction: float = 0.10
    maximum_return_correction_translation_m: float = 0.020
    maximum_return_correction_rotation_rad: float = math.radians(5.0)
    maximum_return_correction_total_translation_m: float = 0.040
    maximum_return_correction_total_rotation_rad: float = math.radians(10.0)
    pose_stability_minimum_samples: int = 3
    pose_stability_minimum_span_sec: float = 0.3
    pose_stability_position_tolerance_m: float = 0.001
    pose_stability_orientation_tolerance_rad: float = math.radians(0.2)
    planning_frame_position_tolerance_m: float = 0.001
    planning_frame_orientation_tolerance_rad: float = math.radians(0.2)
    reference_position_tolerance_m: float = 0.010
    reference_orientation_tolerance_rad: float = math.radians(2.0)
    maximum_measurement_duration_sec: float = 60.0
    force_contact_threshold_n: float = 5.0

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(cls):
            name = f"close_surface.{field.name}"
            if not node.has_parameter(name):
                node.declare_parameter(name, getattr(defaults, field.name))
            value = node.get_parameter(name).value
            values[field.name] = value if type(getattr(defaults, field.name)) is int else float(value)
        result = cls(**values)
        result.validate()
        return result

    def validate(self):
        for field in fields(MeasureSensorTipDistanceConfig):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be positive and finite")
        if (type(self.minimum_surface_samples) is not int
                or self.minimum_surface_samples < 3):
            raise ValueError("Surface sampling requires at least three samples")
        if self.return_correction_speed_scale >= 1.0:
            raise ValueError("Return correction speed scale must be less than one")
        for name in ("maximum_return_corrections", "pose_stability_minimum_samples"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if self.pose_stability_minimum_samples < 3:
            raise ValueError("Pose stability requires at least three distinct samples")
        if self.minimum_return_correction_improvement_fraction >= 1.0:
            raise ValueError("Return correction improvement fraction must be less than one")
        if self.pose_stability_minimum_span_sec >= self.sample_timeout_sec:
            raise ValueError("Pose stability span must be shorter than the sampling timeout")


@dataclass(frozen=True)
class SensorTipDistanceResult:
    """A measured odom plane and distance from the verified returned tip."""
    surface_plane_odom: SurfacePlane
    returned_probe_pose: PoseData
    reference_body_pose: PoseData
    distance_m: float
    axial_distance_m: float
    axis_error_rad: float
    sample_count: int
    sample_span_sec: float
    peak_to_peak_m: float

    def plane_at_probe(self, probe_pose):
        return transform_surface_plane(self.surface_plane_odom, inverse_pose(probe_pose), "probe")


class MeasureSensorTipDistanceExecution:
    """Own one optional observation excursion and its verified return.

    Cancellation only stops; it never initiates an automatic return. A failed
    return or changed frame/attachment prohibits recovery by enclosing workflows.
    """
    UNSAFE_TO_RETURN = frozenset({
        ArmMovementOutcome.STOP_UNCONFIRMED, ArmMovementOutcome.RETREAT_FAILED,
        ArmMovementOutcome.RECOVERY_FAILED, ArmMovementOutcome.UNSTABLE_ARM,
        ArmMovementOutcome.BUSY, ArmMovementOutcome.TRAJECTORY_CANCELLED,
    })

    def __init__(self, executor, surface_source, config, monotonic_clock=time.monotonic, logger=None):
        config.validate()
        self.executor = executor
        self.surface_source = surface_source
        self.config = config
        self._clock = monotonic_clock
        self.logger = logger
        self._started = False
        self.result = None
        self.failure_outcome = None
        self.feedback_message = ""

    @property
    def active(self):
        return self._started

    def start(self, command):
        if self.active:
            raise RuntimeError("Sensor tip distance measurement is already active")
        # Tip orientation also invokes this execution with its own command;
        # it intentionally measures and returns relative to the current pose.
        self._return_target = getattr(command, "return_target", None)
        # Current-pose actions discover distance from depth. A saved tag goal
        # can additionally check correspondence, without seeding the geometry.
        self.expected_distance_m = None
        if self._return_target is not None and command.expected_distance_m is not None:
            expected = float(command.expected_distance_m)
            if not math.isfinite(expected) or not 0.003 <= expected <= 0.50:
                raise ValueError("Expected probe distance must be between 0.003 and 0.50 m")
            self.expected_distance_m = expected
        self._return_label = "saved tag pose" if self._return_target is not None else "original pose"
        self._return_position_tolerance_m = self.config.return_position_tolerance_m
        self._return_orientation_tolerance_rad = self.config.return_orientation_tolerance_rad
        self.result = None
        self.failure_outcome = None
        self._samples = {}
        self._aggregate = None
        self._failure_detail = ""
        self._moved = False
        self._return_started = False
        self._return_budget = PoseCorrectionBudget(
            maximum_corrections=self.config.maximum_return_corrections,
            maximum_translation_m=self.config.maximum_return_correction_translation_m,
            maximum_rotation_rad=self.config.maximum_return_correction_rotation_rad,
            maximum_total_translation_m=self.config.maximum_return_correction_total_translation_m,
            maximum_total_rotation_rad=self.config.maximum_return_correction_total_rotation_rad,
            minimum_improvement_fraction=self.config.minimum_return_correction_improvement_fraction)
        self._acquiring_attachment = None
        self._last_motion_notice = ""
        self._sensor_id = None
        self._phase = "acquire"
        self._start_time = self._clock()
        self._started = True
        self.feedback_message = "Settling before measuring sensor tip distance"
        if self.executor.active:
            # This execution has not acquired the arm; never cancel its owner.
            self._started = False
            self.failure_outcome = ArmMovementOutcome.BUSY
            self.feedback_message = "Arm executor is busy; sensor tip distance was not started"
            return MeasureSensorTipDistanceOutcome.FAILURE
        try:
            self._begin_pose_settle()
        except Exception as error:
            return self._fail(str(error))
        return self.poll()

    def cancel(self):
        if self.executor.active:
            self.executor.cancel()
        self._started = False
        self.result = None

    def poll(self):
        if not self.active:
            raise RuntimeError("No sensor tip distance measurement is active")
        try:
            if self._clock() - self._start_time > self.config.maximum_measurement_duration_sec:
                return self._fail("Sensor tip distance operation timed out", ArmMovementOutcome.RECOVERY_FAILED)
            if self._sensor_id is not None:
                self._check_attachment()
                self._check_reference(pose_to_pose_data(
                    self.executor.probe_motion_planner.current_pose("odom", "body")))
            if self._phase == "acquire":
                return self._acquire()
            if self._phase == "view":
                return self._view()
            if self._phase in ("observing", "returning"):
                return self._motion_update(self.executor.poll())
            if self._phase == "sampling":
                return self._sample()
            if self._phase == "return_settle":
                return self._verify_return()
            if self._phase == "confirm_stop":
                return self._stop_update(self.executor.poll())
            raise RuntimeError(f"Unknown distance measurement phase: {self._phase}")
        except Exception as error:
            # Unknown failures during motion must stop, never guess a return path.
            if self.executor.active or self._return_started:
                return self._fail(str(error), ArmMovementOutcome.RECOVERY_FAILED)
            if self._moved:
                return self._begin_return(str(error))
            return self._fail(str(error))

    def _acquire(self):
        if self._clock() < self._receipt_after:
            return MeasureSensorTipDistanceOutcome.RUNNING
        try:
            sensor_id, revision = self.surface_source.active_attachment()
            if self._acquiring_attachment != (sensor_id, revision):
                if self._acquiring_attachment is not None:
                    self._pose_stability.invalidate("Sensor attachment changed during pose acquisition")
                self._acquiring_attachment = (sensor_id, revision)
            hand, probe, body = self._settled_pose(sensor_id)
        except Exception as error:
            return self._wait_or_fail(error)
        self._original_hand = deepcopy(hand)
        self._original_probe = deepcopy(probe)
        self._original_body = deepcopy(body)
        self._sensor_id, self._attachment_revision = sensor_id, revision
        self._check_attachment()
        self._return_hand = deepcopy(hand)
        self._return_probe = deepcopy(probe)
        self._nominal_return_required = False
        if self._return_target is not None:
            self._resolve_return_target()
        self._phase = "view"
        self._phase_start = self._clock()
        return self._view()

    def _resolve_return_target(self):
        """Freeze a nearby saved tag goal once; never chase a tag during return."""
        planner = self.executor.probe_motion_planner
        resolved = planner.resolve_tag(self._return_target, self.executor.tag_state_source)
        if resolved.sensor_id != self._sensor_id:
            raise ValueError("Saved return target does not match the active sensor attachment")
        target = planner.normalize_target(resolved.target, "odom")
        probe = pose_to_pose_data(target)
        hand = pose_to_pose_data(planner.probe_target_to_hand_target(target, self._sensor_id))
        probe.validate()
        hand.validate()
        errors = tuple(
            (self._position_error(actual, nominal),
             rotation_distance_rad(actual.orientation, nominal.orientation))
            for actual, nominal in ((self._original_hand, hand), (self._original_probe, probe)))
        # This subcommand is an observation near the aligned goal, not a path
        # planner for reaching a distant saved pose. Reject before any motion.
        if any(position > self.config.maximum_return_correction_translation_m
               or angle > self.config.maximum_return_correction_rotation_rad
               for position, angle in errors):
            raise ValueError("Saved tag return target is outside the bounded observation-return range; "
                             "move to the aligned pose before measuring")
        if self._return_target.precision_mode:
            position_tolerance = float(self._return_target.tag_position_tolerance_m)
            orientation_tolerance = float(self._return_target.precision_orientation_tolerance_rad)
            if any(not math.isfinite(value) or value <= 0.0
                   for value in (position_tolerance, orientation_tolerance)):
                raise ValueError("Saved return target precision tolerances must be positive and finite")
            self._return_position_tolerance_m = min(self._return_position_tolerance_m, position_tolerance)
            self._return_orientation_tolerance_rad = min(
                self._return_orientation_tolerance_rad, orientation_tolerance)
        self._return_hand, self._return_probe = hand, probe
        self._nominal_return_required = any(
            position > self._return_position_tolerance_m
            or angle > self._return_orientation_tolerance_rad for position, angle in errors)
        self._log_return("Saved tag return target frozen in odom for sensor tip measurement")

    def _begin_pose_settle(self):
        self._receipt_after = self._clock() + self.config.settle_sec
        self._phase_start = self._receipt_after
        # The source owns the ROS acquisition clock; execution deadlines use
        # monotonic time. Never compare an image stamp with a monotonic deadline.
        self._pose_acquisition_after = self.surface_source.pose_acquisition_cutoff(
            self.config.settle_sec)
        self._pose_stability = PoseStabilityWindow(
            minimum_samples=self.config.pose_stability_minimum_samples,
            minimum_span_sec=self.config.pose_stability_minimum_span_sec,
            position_tolerance_m=self.config.pose_stability_position_tolerance_m,
            orientation_tolerance_rad=self.config.pose_stability_orientation_tolerance_rad,
            maximum_sample_gap_sec=MAX_HAND_DEPTH_AGE_SEC,
            pose_names=("hand", "sensor tip", "body"))

    def _settled_pose(self, sensor_id):
        try:
            sample = self.surface_source.sensor_tip_pose_sample(
                sensor_id, receipt_not_before=self._receipt_after,
                acquisition_not_before=self._pose_acquisition_after,
                after_stamp_seconds=self._pose_stability.last_stamp_seconds)
            if not math.isfinite(sample.stamp_seconds) or sample.stamp_seconds <= 0.0:
                raise ValueError("Pose acquisition timestamp is invalid")
            if sample.stamp_seconds < self._pose_acquisition_after:
                raise ValueError("Pose was acquired before the settling deadline")
            for value in (sample.hand, sample.probe, sample.body):
                value.validate()
        except NoNewSensorTipPoseSample as error:
            # No new evidence: neither count the frame nor discard a valid
            # window merely because polling is faster than the camera.
            raise NoNewSensorTipPoseSample(f"{error}; {self._pose_stability_status()}") from error
        except Exception as error:
            self._pose_stability.invalidate(f"Pose input rejected: {error}")
            raise
        stable = self._pose_stability.add(sample.stamp_seconds, (sample.hand, sample.probe, sample.body))
        if not stable:
            raise ValueError(self._pose_stability_status())
        return sample.hand, sample.probe, sample.body

    def _pose_stability_status(self):
        window = self._pose_stability
        reason = f"; last reset: {window.last_reset_reason}" if window.last_reset_reason else ""
        return (
            "Waiting for stable hand, sensor tip and body poses: "
            f"{window.sample_count} distinct samples spanning {window.span_sec:.3f} s; "
            f"need {window.minimum_samples} over {window.minimum_span_sec:.3f} s, "
            f"variation ≤ {window.position_tolerance_m * 1000:.2f} mm / "
            f"{math.degrees(window.orientation_tolerance_rad):.2f} deg{reason}")

    def _view(self):
        if self._clock() < self._receipt_after:
            return MeasureSensorTipDistanceOutcome.RUNNING
        try:
            view = self.surface_source.observation_view(
                self._return_probe, self.expected_distance_m,
                receipt_not_before=self._receipt_after)
        except Exception as error:
            return self._wait_or_fail(error)
        if view.visible:
            self._phase = "sampling"
            self._phase_start = self._clock()
            self.feedback_message = "Collecting sensor tip distance at the original target"
            return self._sample()
        if self._moved:
            # One sparse frame after settling is not a complete sampling attempt.
            # Retry at this pose within the existing budget; never move farther
            # blindly or relax the local target support requirement.
            return self._wait_or_fail(ValueError(f"Target remains unobservable: {view.detail}"))
        delta = view.hand_translation_odom
        distance = self._norm(delta)
        if distance < 0.001:
            return self._wait_or_fail(ValueError(f"Target patch has no reliable depth: {view.detail}"))
        if distance > self.config.maximum_observation_translation_m:
            return self._fail(
                f"Target is not visible; required observation translation {distance:.3f} m "
                f"is outside 0.001–{self.config.maximum_observation_translation_m:.3f} m: {view.detail}")
        if self.executor.moveit_arm_planner is None:
            return self._fail("Sensor tip observation requires collision-checked MoveIt planning")
        hand = deepcopy(self._original_hand)
        hand.position = Vector3Data(hand.position.x + delta.x, hand.position.y + delta.y, hand.position.z + delta.z)
        self._observation_hand = hand
        self._phase = "observing"
        self.feedback_message = f"Moving camera to observe target ({distance * 1000:.1f} mm translation)"
        self._moved = True
        return self._motion_update(self._move_hand(hand))

    def _sample(self):
        try:
            fresh = self.surface_source.sensor_tip_distance_samples(
                self._return_probe, self.expected_distance_m,
                receipt_not_before=self._receipt_after,
                maximum_age_sec=max(self.config.sample_timeout_sec, self.config.minimum_surface_span_sec),
                minimum_samples=self.config.minimum_surface_samples,
                maximum_latest_age_sec=MAX_HAND_DEPTH_AGE_SEC)
            for sample in fresh:
                self._samples[sample.stamp_seconds] = sample
            aggregate = aggregate_surface_distance_samples(
                tuple(self._samples.values()), target_distance_m=0.001,
                maximum_step_m=0.010, minimum_samples=self.config.minimum_surface_samples,
                minimum_span_sec=self.config.minimum_surface_span_sec,
                stability_tolerance_m=self.config.surface_stability_tolerance_m)
            if aggregate.surface_plane_probe is None:
                raise ValueError("Sensor tip distance has no fitted surface plane")
            selected = _recent_sample_window(
                sorted(self._samples.values(), key=lambda sample: sample.stamp_seconds),
                self.config.minimum_surface_samples, self.config.minimum_surface_span_sec,
            )
            normal = aggregate.surface_plane_probe.normal_array()
            for sample in selected:
                if sample.surface_plane_probe is None:
                    raise ValueError("Sensor tip distance frame has no fitted plane")
                cosine = float(normal @ sample.surface_plane_probe.normal_array())
                if math.acos(max(-1.0, min(1.0, cosine))) > self.config.maximum_axis_error_rad:
                    raise ValueError("Sensor tip surface normals are unstable across selected frames")
            self._aggregate = aggregate
            self._plane_odom = transform_surface_plane(
                aggregate.surface_plane_probe, self._return_probe, "odom")
            return (self._begin_return() if self._moved or self._nominal_return_required
                    else self._finish_result())
        except Exception as error:
            return self._wait_or_fail(error)

    def _move_hand(self, hand, *, speed_scale=1.0):
        if self.executor.moveit_arm_planner is None:
            raise RuntimeError("Sensor tip observation requires collision-checked MoveIt planning")
        stamped = PoseStamped()
        stamped.header.frame_id = "odom"
        stamped.pose = pose_data_to_pose(hand)
        speed = ArmMotionSpeed(
            self.config.observation_speed_mps * speed_scale,
            self.executor.speed_policy.default_speed.angular_speed_rad_s * speed_scale,
        )
        return self.executor.guarded_probe(
            stamped, BARE_HAND_MOTION_ID, speed=speed,
            force_threshold_n=self.config.force_contact_threshold_n,
            cartesian_path=True, ignore_environment_collisions=False,
            planning_frame_position_tolerance_m=self.config.planning_frame_position_tolerance_m,
            planning_frame_orientation_tolerance_rad=self.config.planning_frame_orientation_tolerance_rad)

    def _motion_update(self, update):
        if update.outcome is ArmMovementOutcome.RUNNING:
            if (any(marker in update.detail for marker in ("Replanning prepared", "ArmStopCommand"))
                    and update.detail != self._last_motion_notice):
                self._last_motion_notice = update.detail
                self.feedback_message = f"Sensor tip distance {self._phase}: {update.detail}"
                self._log_return(self.feedback_message)
            return MeasureSensorTipDistanceOutcome.RUNNING
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            if update.outcome in self.UNSAFE_TO_RETURN:
                return self._fail(
                    update.detail, update.outcome,
                    cancel_motion=update.outcome is not ArmMovementOutcome.BUSY,
                )
            if self._phase == "returning":
                return self._fail(f"Observation return failed: {update.detail}", ArmMovementOutcome.RECOVERY_FAILED)
            self._failure_detail = f"Observation movement failed: {update.detail}"
            self._phase = "confirm_stop"
            return self._stop_update(self.executor.confirm_stop())
        self._begin_pose_settle()
        self._phase = "return_settle" if self._phase == "returning" else "view"
        return MeasureSensorTipDistanceOutcome.RUNNING

    def _stop_update(self, update):
        if update.outcome is ArmMovementOutcome.RUNNING:
            return MeasureSensorTipDistanceOutcome.RUNNING
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            return self._fail(f"Observation stop unconfirmed: {update.detail}", ArmMovementOutcome.STOP_UNCONFIRMED)
        return self._begin_return(self._failure_detail)

    def _begin_return(self, failure="", *, correction=False):
        if failure:
            self._failure_detail = failure
        self._return_started = True
        try:
            self._check_attachment()
            self._check_reference(pose_to_pose_data(
                self.executor.probe_motion_planner.current_pose("odom", "body")))
            if self.executor.active:
                return self._fail(
                    "Arm executor is busy; observation return cannot start",
                    ArmMovementOutcome.BUSY, cancel_motion=False,
                )
            self._phase = "returning"
            if correction:
                self.feedback_message = (
                    f"Fine-adjusting observation return to the {self._return_label} "
                    f"(slow correction {self._return_budget.corrections}/{self.config.maximum_return_corrections})")
            else:
                self.feedback_message = f"Returning to {self._return_label} after sensor tip distance observation"
            return self._motion_update(self._move_hand(
                self._return_hand,
                speed_scale=self.config.return_correction_speed_scale if correction else 1.0,
            ))
        except Exception as error:
            return self._fail(f"Observation return failed: {error}", ArmMovementOutcome.RECOVERY_FAILED)

    def _verify_return(self):
        if self._clock() < self._receipt_after:
            return MeasureSensorTipDistanceOutcome.RUNNING
        try:
            return self._finish_result()
        except _ReturnPoseMismatch as error:
            self._log_return(str(error))
            try:
                self._return_budget.reserve(
                    error.position_error_m, error.orientation_error_rad,
                    self._return_position_tolerance_m, self._return_orientation_tolerance_rad)
            except ValueError as limit:
                detail = str(limit).replace("Pose", "Return").replace("pose", "return")
                return self._fail(f"{error}; {detail}", ArmMovementOutcome.RECOVERY_FAILED)
            return self._begin_return(correction=True)
        except Exception as error:
            return self._wait_or_fail(error)

    def _finish_result(self):
        hand, probe, body = self._settled_pose(self._sensor_id)
        self._check_attachment()
        self._check_reference(body)
        errors = []
        for name, current, original in (
            ("hand", hand, self._return_hand),
            ("sensor tip", probe, self._return_probe),
        ):
            current.validate()
            original.validate()
            position_error = self._position_error(current, original)
            angle = rotation_distance_rad(current.orientation, original.orientation)
            errors.append((name, position_error, angle))
        if any(position > self._return_position_tolerance_m
               or angle > self._return_orientation_tolerance_rad
               for _, position, angle in errors):
            raise _ReturnPoseMismatch(errors, self._return_diagnostics(hand, probe, body),
                                      self._return_label.capitalize())
        if self._return_started:
            self._log_return(self._return_label.capitalize() + " verified; " + self._return_diagnostics(hand, probe, body))
        if self._failure_detail:
            return self._fail(f"{self._failure_detail}; returned to {self._return_label}")
        plane = transform_surface_plane(self._plane_odom, inverse_pose(probe), "probe")
        alignment = -plane.normal.x
        distance = plane.signed_distance(Vector3Data.zero())
        if alignment <= 1e-6 or distance <= 0:
            return self._fail("Measured surface does not intersect forward probe +X")
        angle = math.acos(max(-1.0, min(1.0, alignment)))
        self.result = SensorTipDistanceResult(
            deepcopy(self._plane_odom), deepcopy(probe), deepcopy(body), distance,
            distance/alignment, angle, self._aggregate.sample_count,
            self._aggregate.sample_span_sec, self._aggregate.peak_to_peak_m)
        self.feedback_message = (
            f"Sensor tip distance: {distance*1000:.2f} mm perpendicular; "
            f"{self.result.axial_distance_m*1000:.2f} mm along probe +X; "
            f"axis error {math.degrees(angle):.2f} deg; "
            f"{self.result.sample_count} frames, spread {self.result.peak_to_peak_m*1000:.2f} mm; "
            f"{self._return_label} verified"
            + (f" after {self._return_budget.corrections} slow return correction(s)" if self._return_budget.corrections else ""))
        self._started = False
        return MeasureSensorTipDistanceOutcome.SUCCESS

    def _wait_or_fail(self, error):
        self.feedback_message = f"Measuring sensor tip distance: {error}"
        if self._clock() - self._phase_start < self.config.sample_timeout_sec:
            return MeasureSensorTipDistanceOutcome.RUNNING
        if self._return_started:
            detail = str(error)
            if self._return_budget.corrections:
                detail += f"; {self._return_budget.corrections} slow return correction(s) used"
            return self._fail(detail, ArmMovementOutcome.RECOVERY_FAILED)
        if self._moved:
            return self._begin_return(str(error))
        return self._fail(str(error))

    def _return_diagnostics(self, hand, probe, body):
        details = []
        for name, current, original in (
            ("hand", hand, self._return_hand),
            ("tip", probe, self._return_probe),
            ("body", body, self._original_body),
        ):
            delta = tuple((getattr(current.position, axis) - getattr(original.position, axis)) * 1000
                          for axis in ("x", "y", "z"))
            angle = math.degrees(rotation_distance_rad(current.orientation, original.orientation))
            details.append(f"{name} odom delta=({delta[0]:+.2f}, {delta[1]:+.2f}, {delta[2]:+.2f}) mm, {angle:.2f} deg")
        return "; ".join(details)

    def _log_return(self, detail):
        if self.logger is not None:
            self.logger.info(detail)

    @classmethod
    def _position_error(cls, current, original):
        return cls._norm(Vector3Data(
            current.position.x - original.position.x,
            current.position.y - original.position.y,
            current.position.z - original.position.z))

    def _check_attachment(self):
        if self.surface_source.active_attachment() != (self._sensor_id, self._attachment_revision):
            # A changed attachment invalidates the original return geometry too.
            self._return_started = True
            raise RuntimeError("Sensor attachment changed; measurement and automatic return invalidated")

    def _check_reference(self, current):
        delta = Vector3Data(current.position.x-self._original_body.position.x,
                            current.position.y-self._original_body.position.y,
                            current.position.z-self._original_body.position.z)
        if (self._norm(delta) > self.config.reference_position_tolerance_m
                or rotation_distance_rad(current.orientation, self._original_body.orientation)
                > self.config.reference_orientation_tolerance_rad):
            self._return_started = True
            raise RuntimeError("Body/odom reference changed; measurement and automatic return invalidated")

    def _fail(self, detail, outcome=ArmMovementOutcome.EXECUTION_ERROR, *, cancel_motion=True):
        if cancel_motion and self.executor.active:
            self.executor.cancel()
        self.result = None
        self.failure_outcome = outcome
        self.feedback_message = str(detail)
        self._started = False
        return MeasureSensorTipDistanceOutcome.FAILURE

    @staticmethod
    def _norm(vector):
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
