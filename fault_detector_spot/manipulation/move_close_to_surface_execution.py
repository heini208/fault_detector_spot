"""Close-surface workflow execution below the behavior tree."""

from copy import deepcopy
from dataclasses import dataclass, fields
from enum import Enum
import math
import time

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.shared.geometry.rotation import (
    rotate_vector,
    rotation_distance_rad,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    aggregate_surface_distance_samples,
)
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.shared.geometry.transforms import (
    pose_data_to_pose,
    pose_to_pose_data,
)


CONTACT_MODE_PLANNING_DISTANCE_M = 0.001
MAX_CARTESIAN_STANDOFF_MOVES = 2
MAX_CARTESIAN_CONTACT_MOVES = 1


class MoveCloseToSurfaceOutcome(Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


@dataclass(frozen=True)
class MoveCloseToSurfaceConfig:
    settle_sec: float = 0.5
    sample_timeout_sec: float = 3.0
    maximum_step_m: float = 0.010
    tolerance_m: float = 0.005
    maximum_travel_m: float = 0.400
    maximum_approach_steps: int = 60
    minimum_surface_samples: int = 5
    minimum_surface_span_sec: float = 1.0
    surface_stability_tolerance_m: float = 0.005
    force_contact_threshold_n: float = 5.0
    force_near_target_threshold_n: float = 3.0
    force_near_target_distance_m: float = 0.020
    approach_far_speed_mps: float = 0.005
    approach_near_speed_mps: float = 0.001
    contact_search_retreat_distance_m: float = 0.010
    contact_search_speed_mps: float = 0.001
    contact_search_force_threshold_n: float = 2.0
    approach_slowdown_distance_m: float = 0.050
    contact_search_overtravel_m: float = 0.005
    recovery_speed_mps: float = 0.020
    maximum_lateral_drift_m: float = 0.010
    maximum_axis_error_rad: float = math.radians(5.0)
    minimum_step_progress_ratio: float = 0.25

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        integer_fields = {
            "maximum_approach_steps",
            "minimum_surface_samples",
        }
        for field in fields(cls):
            name = f"close_surface.{field.name}"
            default = getattr(defaults, field.name)
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            value = node.get_parameter(name).value
            values[field.name] = (
                int(value) if field.name in integer_fields else float(value)
            )
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        c = self
        positive = (
            c.settle_sec,
            c.sample_timeout_sec,
            c.maximum_step_m,
            c.tolerance_m,
            c.maximum_travel_m,
            c.minimum_surface_span_sec,
            c.surface_stability_tolerance_m,
            c.force_contact_threshold_n,
            c.force_near_target_threshold_n,
            c.force_near_target_distance_m,
            c.approach_far_speed_mps,
            c.approach_near_speed_mps,
            c.contact_search_speed_mps,
            c.contact_search_retreat_distance_m,
            c.contact_search_force_threshold_n,
            c.approach_slowdown_distance_m,
            c.contact_search_overtravel_m,
            c.recovery_speed_mps,
            c.maximum_lateral_drift_m,
            c.maximum_axis_error_rad,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("Close-surface configuration values must be positive")
        if c.force_near_target_threshold_n > c.force_contact_threshold_n:
            raise ValueError(
                "Near-target force threshold must not exceed the normal threshold"
            )
        if c.approach_near_speed_mps > c.approach_far_speed_mps:
            raise ValueError(
                "Near-surface approach speed must not exceed far approach speed"
            )
        if c.maximum_approach_steps < 1:
            raise ValueError("Maximum surface approach steps must be positive")
        if c.maximum_step_m > 0.010 + 1e-12:
            raise ValueError("Surface approach step must not exceed 0.010 m")
        if c.contact_search_overtravel_m > c.maximum_step_m + 1e-12:
            raise ValueError(
                "Contact search overtravel must not exceed the maximum step"
            )
        if c.maximum_travel_m > (
            c.maximum_step_m * c.maximum_approach_steps + 1e-12
        ):
            raise ValueError(
                "Maximum surface approach travel exceeds the configured "
                "step-count limit"
            )
        if c.minimum_surface_samples < 3:
            raise ValueError("Surface sampling requires at least three samples")
        if not 0.0 < c.minimum_step_progress_ratio <= 1.0:
            raise ValueError(
                "Minimum surface-step progress ratio must be in (0, 1]"
            )



class MoveCloseToSurfaceExecution:
    """Own the complete nonblocking close-surface workflow."""

    def __init__(
        self,
        executor,
        surface_source,
        config,
        monotonic_clock=time.monotonic,
        logger=None,
    ):
        if executor is None:
            raise RuntimeError("Close-surface execution requires arm executor")
        if surface_source is None:
            raise RuntimeError("Close-surface execution requires surface source")
        if not isinstance(config, MoveCloseToSurfaceConfig):
            raise TypeError("Close-surface execution requires its config")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        config.validate()
        self.executor = executor
        self.surface_source = surface_source
        self.config = config
        self._clock = monotonic_clock
        self.logger = logger
        self.feedback_message = ""
        self._clear_runtime()

    @property
    def active(self) -> bool:
        return self._started

    def start(self, command) -> MoveCloseToSurfaceOutcome:
        if self.active:
            raise RuntimeError("Close-surface execution is already active")
        if not isinstance(command, MoveCloseToSurfaceCommand):
            raise RuntimeError(
                "Expected MoveCloseToSurfaceCommand, got "
                f"{type(command).__name__}"
            )
        self._clear_runtime()
        self._command = command
        self._phase = "acquire"
        self._phase_started = self._clock()
        self._started = True
        self.feedback_message = "Resolving active probe attachment"
        return self.poll()

    def poll(self) -> MoveCloseToSurfaceOutcome:
        if not self.active:
            raise RuntimeError("No close-surface execution is active")
        try:
            if self._phase == "acquire":
                return self._update_acquire()
            if self._phase == "contact_replan":
                return self._start_contact_search()
            if self._phase == "sampling":
                return self._update_sampling()
            if self._phase == "approach":
                return self._handle_approach_update(self.executor.poll())
            if self._phase == "settling":
                return self._update_settling()
            if self._phase == "recovery_prepare":
                return self._update_recovery_prepare()
            if self._phase == "recovering":
                return self._handle_recovery_update(self.executor.poll())
            return self._fail_workflow(
                f"Unknown close-surface phase: {self._phase}"
            )
        except Exception as exception:
            if self._approach_steps > 0 and self._recovery_hand_pose is not None:
                return self._begin_recovery(str(exception))
            return self._fail_workflow(str(exception))

    def cancel(self) -> None:
        if self.executor.active:
            self.executor.cancel()
        self._started = False

    @property
    def contact_mode(self) -> bool:
        return (
            self._command is not None
            and self._command.target_surface_distance_m == 0.0
        )

    def _update_acquire(self) -> MoveCloseToSurfaceOutcome:
        try:
            sensor_id, revision = self.surface_source.active_attachment()
            recovery_pose = self._current_hand_pose()
        except Exception as exception:
            if self._clock() - self._phase_started < self.config.sample_timeout_sec:
                self.feedback_message = (
                    f"Waiting for probe runtime state: {exception}"
                )
                return MoveCloseToSurfaceOutcome.RUNNING
            return self._fail_workflow(str(exception))

        self._sensor_id = sensor_id
        self._attachment_revision = revision
        self._recovery_hand_pose = deepcopy(recovery_pose)
        if self.contact_mode:
            return self._start_contact_search()
        self._start_sampling()
        return MoveCloseToSurfaceOutcome.RUNNING

    def _start_contact_search(self) -> MoveCloseToSurfaceOutcome:
        """Search along probe +X with a force guard, without surface sensing."""
        self._require_attachment_unchanged()
        current = self._current_probe_pose()
        current.validate()
        self._previous_probe_pose = deepcopy(current)
        self._aligned_probe_orientation = deepcopy(current.orientation)
        self._contact_inward = rotate_vector(
            current.orientation, Vector3Data(x=1.0, y=0.0, z=0.0)
        )
        if self._contact_search_distance_m is None:
            self._contact_search_distance_m = self.config.maximum_travel_m
        self._requested_step_m = self._contact_search_distance_m
        target = PoseData(
            position=Vector3Data(
                x=current.position.x + self._contact_inward.x * self._requested_step_m,
                y=current.position.y + self._contact_inward.y * self._requested_step_m,
                z=current.position.z + self._contact_inward.z * self._requested_step_m,
            ),
            orientation=deepcopy(self._aligned_probe_orientation),
        )
        self._approach_steps = 1
        self._phase = "approach"
        speed = self._speed(self.config.contact_search_speed_mps)
        threshold = self.config.contact_search_force_threshold_n
        update = self.executor.guarded_probe(
            self._pose_stamped(target),
            self._sensor_id,
            speed=speed,
            force_threshold_n=threshold,
            retreat_distance_m=self.config.contact_search_retreat_distance_m,
            cartesian_path=True,
        )
        self.feedback_message = (
            f"Searching for contact along probe +X by {self._requested_step_m:.4f} m "
            f"at {speed.linear_speed_mps:.4f} m/s with {threshold:.2f} N guard"
        )
        return self._handle_approach_update(update)

    def _start_sampling(self) -> None:
        self._surface_samples = {}
        self._sample_receipt_not_before = self._clock()
        self._phase_started = self._sample_receipt_not_before
        self._plan = None
        self._previous_probe_pose = None
        self._approach_steps = 0
        self._phase = "sampling"
        self.feedback_message = "Collecting initial live surface distance"

    def _update_sampling(self) -> MoveCloseToSurfaceOutcome:
        self._require_attachment_unchanged()
        now = self._clock()
        last_error = None

        try:
            fresh = self.surface_source.surface_distance_samples(
                self._sensor_id,
                receipt_not_before=self._sample_receipt_not_before,
                maximum_age_sec=max(
                    self.config.sample_timeout_sec,
                    self.config.minimum_surface_span_sec,
                ),
                minimum_samples=self.config.minimum_surface_samples,
            )
            for sample in fresh:
                self._surface_samples[sample.stamp_seconds] = sample

            planning_target = self._planning_target_distance()
            aggregate = aggregate_surface_distance_samples(
                tuple(self._surface_samples.values()),
                planning_target,
                self.config.maximum_step_m,
                tolerance_m=self._surface_tolerance(),
                minimum_samples=self.config.minimum_surface_samples,
                minimum_span_sec=self.config.minimum_surface_span_sec,
                stability_tolerance_m=(
                    self.config.surface_stability_tolerance_m
                ),
            )
            current_probe = self._current_probe_pose()

            if aggregate.surface_plane_probe is None:
                raise RuntimeError(
                    "Stable surface sampling did not produce surface geometry"
                )
            measured_axis_error = math.acos(max(-1.0, min(
                1.0, -aggregate.surface_plane_probe.normal.x,
            )))
            if measured_axis_error > self.config.maximum_axis_error_rad:
                raise ValueError(
                    "Probe axis is not aligned with the measured surface: "
                    f"{math.degrees(measured_axis_error):.2f} deg > "
                    f"{math.degrees(self.config.maximum_axis_error_rad):.2f} deg"
                )
            if not self.contact_mode and aggregate.verified:
                return self._success(
                    "Surface stand-off already reached from live measurement: "
                    f"{aggregate.distance_m:.4f} m"
                )
            self._plan = freeze_probe_surface_approach(
                current_probe_pose_execution=current_probe,
                surface_plane_probe=aggregate.surface_plane_probe,
                target_distance_m=planning_target,
                maximum_travel_m=self.config.maximum_travel_m,
            )
            if (
                self._plan.initial_axis_error_rad
                > self.config.maximum_axis_error_rad
            ):
                raise ValueError(
                    "Probe axis is not aligned with the estimated surface: "
                    f"{math.degrees(self._plan.initial_axis_error_rad):.2f} deg "
                    f"> {math.degrees(self.config.maximum_axis_error_rad):.2f} deg"
                )

            self._previous_probe_pose = deepcopy(current_probe)
            self._aligned_probe_orientation = deepcopy(
                current_probe.orientation
            )
            mode = "contact" if self.contact_mode else "stand-off"
            self.feedback_message = (
                f"Frozen {mode} Cartesian approach from live surface estimate at "
                f"{aggregate.distance_m:.4f} m"
            )
            return self._prepare_next_approach_step()
        except Exception as exception:
            last_error = exception

        detail = str(last_error)
        if self._surface_samples:
            detail += (
                f"; {len(self._surface_samples)} distinct valid frame(s) "
                "accumulated"
            )

        if now - self._phase_started >= self.config.sample_timeout_sec:
            return self._fail_workflow(
                "Unable to establish stable surface distance: "
                f"{detail}"
            )
        self.feedback_message = (
            "Collecting initial surface distance: "
            f"{detail}"
        )
        return MoveCloseToSurfaceOutcome.RUNNING

    def _prepare_next_approach_step(self) -> MoveCloseToSurfaceOutcome:
        self._require_attachment_unchanged()
        current_probe = self._current_probe_pose()
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.config.maximum_step_m,
            tolerance_m=self._surface_tolerance(),
        )
        self._validate_axis_guard(evaluation)

        if evaluation.reached and not self.contact_mode:
            return self._success(
                "Reached requested surface stand-off from frozen estimate: "
                f"{evaluation.estimated_distance_m:.4f} m"
            )

        movement_limit = self._cartesian_movement_limit()
        if self._approach_steps >= movement_limit:
            if self.contact_mode:
                detail = (
                    "Cartesian contact search completed without detecting "
                    "surface contact"
                )
            else:
                detail = (
                    "Cartesian surface approach did not reach the requested "
                    "stand-off after one endpoint correction"
                )
            return self._begin_recovery(detail)

        requested_step_m = self._requested_step(evaluation)
        if requested_step_m <= 0.0:
            return self._begin_recovery(
                "Cartesian surface approach reached its travel limit without "
                "reaching the requested target"
            )

        self._previous_probe_pose = deepcopy(current_probe)
        self._requested_step_m = requested_step_m
        threshold_n = self._force_threshold_for(
            evaluation.remaining_inward_travel_m
        )
        speed = self._approach_speed_for(
            evaluation.estimated_distance_m
        )

        inward = self._plan.inward_direction()
        target = PoseData(
            position=Vector3Data(
                x=current_probe.position.x + inward.x * requested_step_m,
                y=current_probe.position.y + inward.y * requested_step_m,
                z=current_probe.position.z + inward.z * requested_step_m,
            ),
            orientation=deepcopy(self._aligned_probe_orientation),
        )
        self._approach_steps += 1
        self._phase = "approach"
        update = self.executor.guarded_probe(
            self._pose_stamped(target),
            self._sensor_id,
            speed=speed,
            force_threshold_n=threshold_n,
            cartesian_path=True,
        )
        stage = "approach" if self._approach_steps == 1 else "correction"
        self.feedback_message = (
            f"Starting straight Cartesian surface {stage} by "
            f"{requested_step_m:.4f} m at {speed.linear_speed_mps:.4f} m/s "
            f"with {threshold_n:.2f} N guard "
            f"(move {self._approach_steps}/{movement_limit})"
        )
        return self._handle_approach_update(update)

    def _handle_approach_update(self, update) -> MoveCloseToSurfaceOutcome:
        if self.contact_mode and update.outcome in (
            ArmMovementOutcome.STOP_UNCONFIRMED,
            ArmMovementOutcome.RETREAT_FAILED,
        ):
            return self._fail_workflow(
                "Contact search could not complete its local stop/retreat: "
                f"{update.outcome.value}: {update.detail}; "
                "no return to the pre-approach pose was commanded"
            )
        # An incomplete Cartesian plan has not been executed. Shorten its
        # endpoint and replan; the eventual successful path executes once.
        if (
            self.contact_mode
            and update.outcome is ArmMovementOutcome.PLANNING_FAILED
            and update.detail.startswith("MoveIt Cartesian path is incomplete:")
            and self._contact_plan_retries < 8
            and self._requested_step_m * 0.8 >= self.config.maximum_step_m
        ):
            self._contact_plan_retries += 1
            self._contact_search_distance_m = self._requested_step_m * 0.8
            self._phase = "contact_replan"
            self.feedback_message = (
                "Shortening contact-search endpoint before motion to "
                f"{self._contact_search_distance_m:.4f} m for Cartesian replanning"
            )
            return MoveCloseToSurfaceOutcome.RUNNING
        if update.outcome is ArmMovementOutcome.RUNNING:
            return MoveCloseToSurfaceOutcome.RUNNING
        if update.outcome is ArmMovementOutcome.CONTACT:
            if self.contact_mode:
                return self._success(
                    "Surface contact detected; shared arm guard completed "
                    "the local snap retreat"
                )
            return self._begin_recovery(
                "Unexpected contact before the requested stand-off was "
                f"reached: {update.detail}"
            )
        if update.outcome is ArmMovementOutcome.SUCCESS:
            self._settle_deadline = self._clock() + self.config.settle_sec
            self._phase = "settling"
            self.feedback_message = (
                "Cartesian surface movement completed; waiting to settle"
            )
            return MoveCloseToSurfaceOutcome.RUNNING
        return self._begin_recovery(
            "Surface approach movement failed: "
            f"{update.outcome.value}: {update.detail}"
        )

    def _update_settling(self) -> MoveCloseToSurfaceOutcome:
        self._require_attachment_unchanged()
        if self._clock() < self._settle_deadline:
            return MoveCloseToSurfaceOutcome.RUNNING

        current_probe = self._current_probe_pose()
        if self.contact_mode:
            orientation_error = rotation_distance_rad(
                current_probe.orientation, self._aligned_probe_orientation,
            )
            if orientation_error > self.config.maximum_axis_error_rad:
                raise RuntimeError("Probe rotated away from the contact search axis")
            self._validate_step_motion(current_probe)
            return self._begin_recovery(
                "Cartesian contact search reached its planned endpoint without "
                "detecting surface contact"
            )
        evaluation = evaluate_probe_surface_approach(
            self._plan,
            current_probe_pose_execution=current_probe,
            maximum_step_m=self.config.maximum_step_m,
            tolerance_m=self._surface_tolerance(),
        )
        self._validate_axis_guard(evaluation)
        achieved, lateral = self._validate_step_motion(current_probe)
        self.feedback_message = (
            f"Cartesian move settled: inward {achieved:.4f} m, lateral "
            f"{lateral:.4f} m; estimated surface distance "
            f"{evaluation.estimated_distance_m:.4f} m"
        )

        return self._prepare_next_approach_step()

    def _begin_recovery(self, detail: str) -> MoveCloseToSurfaceOutcome:
        self._recovery_detail = str(detail).strip()
        if self.executor is not None and self.executor.active:
            self.executor.cancel()
        if self._recovery_hand_pose is None or self._approach_steps == 0:
            return self._fail_workflow(self._recovery_detail)
        self._phase = "recovery_prepare"
        self.feedback_message = (
            f"{self._recovery_detail}; returning to the original "
            "pre-approach hand pose"
        )
        self._log_warning(self.feedback_message)
        return self._update_recovery_prepare()

    def _update_recovery_prepare(self) -> MoveCloseToSurfaceOutcome:
        current = self._current_hand_pose()
        remaining = self._translation_between(
            current,
            self._recovery_hand_pose,
        )
        distance = self._norm(remaining)

        if distance <= 0.005:
            orientation_error = rotation_distance_rad(
                current.orientation,
                self._recovery_hand_pose.orientation,
            )
            if orientation_error > self.config.maximum_axis_error_rad:
                return self._fail_workflow(
                    "Surface recovery reached the start position with an "
                    "excessive orientation error of "
                    f"{math.degrees(orientation_error):.2f} deg"
                )
            return self._fail_workflow(self._recovery_detail)

        if self._recovery_started:
            return self._fail_workflow(
                "Surface recovery did not reach the original pre-approach "
                f"pose ({distance:.4f} m remaining). "
                f"Original failure: {self._recovery_detail}"
            )

        self._recovery_started = True
        self._phase = "recovering"
        update = self.executor.probe(
            self._pose_stamped(deepcopy(self._recovery_hand_pose)),
            BARE_HAND_MOTION_ID,
            speed=self._speed(self.config.recovery_speed_mps),
        )
        self.feedback_message = (
            "Recovering to original pre-approach pose in one movement"
        )
        return self._handle_recovery_update(update)

    def _handle_recovery_update(self, update) -> MoveCloseToSurfaceOutcome:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return MoveCloseToSurfaceOutcome.RUNNING
        if update.outcome is ArmMovementOutcome.SUCCESS:
            self._phase = "recovery_prepare"
            return MoveCloseToSurfaceOutcome.RUNNING
        return self._fail_workflow(
            "Surface recovery movement failed: "
            f"{update.outcome.value}: {update.detail}. Original failure: "
            f"{self._recovery_detail}"
        )

    def _current_probe_pose(self) -> PoseData:
        current = self.executor.probe_motion_planner.current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(self._sensor_id),
        )
        return pose_to_pose_data(current)

    def _current_hand_pose(self) -> PoseData:
        return pose_to_pose_data(
            self.executor.probe_motion_planner.current_hand_pose(
                GRAV_ALIGNED_BODY_FRAME_NAME
            )
        )

    def _requested_step(self, evaluation) -> float:
        traveled = max(0.0, float(evaluation.traveled_inward_m))
        remaining_guard = self.config.maximum_travel_m - traveled
        if remaining_guard <= 1e-9:
            return 0.0

        remaining = max(0.0, float(evaluation.remaining_inward_travel_m))
        if not self.contact_mode:
            return min(remaining, remaining_guard)

        if self._approach_steps > 0:
            return 0.0
        desired = remaining + self.config.contact_search_overtravel_m
        return min(desired, remaining_guard)

    def _cartesian_movement_limit(self) -> int:
        workflow_limit = (
            MAX_CARTESIAN_CONTACT_MOVES
            if self.contact_mode
            else MAX_CARTESIAN_STANDOFF_MOVES
        )
        return min(self.config.maximum_approach_steps, workflow_limit)

    def _approach_speed_for(self, estimated_surface_distance_m: float):
        distance = max(0.0, float(estimated_surface_distance_m))
        fraction = min(
            1.0,
            distance / self.config.approach_slowdown_distance_m,
        )
        linear_speed = (
            self.config.approach_near_speed_mps
            + (
                self.config.approach_far_speed_mps
                - self.config.approach_near_speed_mps
            )
            * fraction
        )
        return self._speed(linear_speed)

    def _speed(self, linear_speed_mps: float) -> ArmMotionSpeed:
        default = self.executor.speed_policy.default_speed
        return ArmMotionSpeed(
            linear_speed_mps=float(linear_speed_mps),
            angular_speed_rad_s=default.angular_speed_rad_s,
        )

    def _force_threshold_for(self, remaining_inward_travel_m: float) -> float:
        remaining = max(0.0, float(remaining_inward_travel_m))
        fraction = min(
            1.0,
            remaining / self.config.force_near_target_distance_m,
        )
        return (
            self.config.force_near_target_threshold_n
            + (
                self.config.force_contact_threshold_n
                - self.config.force_near_target_threshold_n
            )
            * fraction
        )

    def _planning_target_distance(self) -> float:
        target = float(self._command.target_surface_distance_m)
        if target > 0.0:
            return target
        return CONTACT_MODE_PLANNING_DISTANCE_M

    def _surface_tolerance(self) -> float:
        requested = float(
            getattr(self._command, "surface_tolerance_m", 0.0)
        )
        if requested > 0.0:
            return requested
        return float(self.config.tolerance_m)

    def _require_attachment_unchanged(self) -> None:
        sensor_id, revision = self.surface_source.active_attachment()
        if (
            sensor_id != self._sensor_id
            or revision != self._attachment_revision
        ):
            raise RuntimeError(
                "Sensor attachment changed during close-surface movement"
            )

    def _validate_axis_guard(self, evaluation) -> None:
        if evaluation.axis_error_rad > self.config.maximum_axis_error_rad:
            raise RuntimeError(
                "Probe axis rotated away from the frozen surface normal by "
                f"{math.degrees(evaluation.axis_error_rad):.2f} deg"
            )

    def _validate_step_motion(self, current_probe: PoseData):
        inward = (
            self._contact_inward if self.contact_mode
            else self._plan.inward_direction()
        )
        previous = self._previous_probe_pose.position
        current = current_probe.position
        delta = Vector3Data(
            x=current.x - previous.x,
            y=current.y - previous.y,
            z=current.z - previous.z,
        )
        achieved = (
            delta.x * inward.x
            + delta.y * inward.y
            + delta.z * inward.z
        )
        lateral_vector = Vector3Data(
            x=delta.x - inward.x * achieved,
            y=delta.y - inward.y * achieved,
            z=delta.z - inward.z * achieved,
        )
        lateral = self._norm(lateral_vector)
        if lateral > self.config.maximum_lateral_drift_m:
            raise RuntimeError(
                "settled endpoint lateral error exceeded the safety limit: "
                f"{lateral:.4f} m > "
                f"{self.config.maximum_lateral_drift_m:.4f} m"
            )
        minimum = (
            self._requested_step_m
            * self.config.minimum_step_progress_ratio
        )
        if achieved + 1e-9 < minimum:
            raise RuntimeError(
                "surface approach did not achieve enough inward progress: "
                f"requested {self._requested_step_m:.4f} m, "
                f"achieved {achieved:.4f} m"
            )
        return achieved, lateral

    def _success(self, detail: str) -> MoveCloseToSurfaceOutcome:
        self.feedback_message = str(detail).strip()
        self._started = False
        return MoveCloseToSurfaceOutcome.SUCCESS

    def _fail_workflow(self, detail: str) -> MoveCloseToSurfaceOutcome:
        if self.executor.active:
            self.executor.cancel()
        self.feedback_message = str(detail).strip() or "Movement failed"
        self._started = False
        return MoveCloseToSurfaceOutcome.FAILURE

    def _clear_runtime(self) -> None:
        self._started = False
        self._command = None
        self._phase = "idle"
        self._phase_started = 0.0
        self._sensor_id = ""
        self._attachment_revision = -1
        self._recovery_hand_pose = None
        self._aligned_probe_orientation = None
        self._contact_inward = None
        self._contact_search_distance_m = None
        self._contact_plan_retries = 0
        self._surface_samples = {}
        self._sample_receipt_not_before = 0.0
        self._plan = None
        self._previous_probe_pose = None
        self._requested_step_m = 0.0
        self._approach_steps = 0
        self._settle_deadline = 0.0
        self._recovery_detail = ""
        self._recovery_started = False

    @staticmethod
    def _pose_stamped(data: PoseData) -> PoseStamped:
        data.validate()
        stamped = PoseStamped()
        stamped.header.frame_id = GRAV_ALIGNED_BODY_FRAME_NAME
        stamped.pose = pose_data_to_pose(data)
        return stamped

    @staticmethod
    def _translation_between(current: PoseData, target: PoseData) -> Vector3Data:
        return Vector3Data(
            x=target.position.x - current.position.x,
            y=target.position.y - current.position.y,
            z=target.position.z - current.position.z,
        )

    @staticmethod
    def _norm(vector: Vector3Data) -> float:
        return math.sqrt(
            vector.x * vector.x
            + vector.y * vector.y
            + vector.z * vector.z
        )

    def _log_warning(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warning(f"[MoveCloseToSurfaceExecution] {message}")


__all__ = [
    "CONTACT_MODE_PLANNING_DISTANCE_M",
    "MAX_CARTESIAN_CONTACT_MOVES",
    "MAX_CARTESIAN_STANDOFF_MOVES",
    "MoveCloseToSurfaceConfig",
    "MoveCloseToSurfaceExecution",
    "MoveCloseToSurfaceOutcome",
]
