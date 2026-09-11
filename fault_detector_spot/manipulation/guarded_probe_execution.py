"""Force guard state machine for one probe movement."""

from copy import deepcopy
from enum import Enum
import math
import time

from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineOutcome,
)
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.directional_force import (
    directional_force_delta,
)
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingOutcome,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlan,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryUnavailable,
)


class _Phase(Enum):
    FORCE_BASELINE = "force_baseline"
    PLAN_WAIT = "plan_wait"
    MOVING = "moving"
    CONTACT_STOPPING = "contact_stopping"
    ABORT_STOPPING = "abort_stopping"
    RETREATING = "retreating"
    POST_RETREAT_SETTLING = "post_retreat_settling"


class GuardedProbeExecution:
    """Own force baseline, monitoring, stop confirmation, and retreat."""

    def __init__(
        self,
        arm_state_source,
        settling_detector,
        force_baseline_sampler,
        force_contact_policy,
        start_goal,
        poll_goal,
        cancel_goal,
        current_hand_pose,
        build_motion_goal,
        default_angular_speed_rad_s: float,
        force_stale_timeout_sec: float,
        retreat_distance_m: float,
        retreat_speed_mps: float,
        monotonic_clock=time.monotonic,
    ):
        required = (
            (arm_state_source, "arm state source"),
            (settling_detector, "settling detector"),
            (force_baseline_sampler, "force baseline sampler"),
            (force_contact_policy, "force contact policy"),
        )
        for value, label in required:
            if value is None:
                raise RuntimeError(
                    f"GuardedProbeExecution requires {label}"
                )
        callbacks = (
            (start_goal, "start goal"),
            (poll_goal, "poll goal"),
            (cancel_goal, "cancel goal"),
            (current_hand_pose, "current hand pose"),
            (build_motion_goal, "build motion goal"),
        )
        for callback, label in callbacks:
            if not callable(callback):
                raise TypeError(
                    f"GuardedProbeExecution {label} callback must be callable"
                )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.arm_state_source = arm_state_source
        self.settling_detector = settling_detector
        self.force_baseline_sampler = force_baseline_sampler
        self.force_contact_policy = force_contact_policy
        self._start_goal = start_goal
        self._poll_goal = poll_goal
        self._cancel_goal = cancel_goal
        self._current_hand_pose = current_hand_pose
        self._build_motion_goal = build_motion_goal
        self.default_angular_speed_rad_s = self._positive(
            default_angular_speed_rad_s,
            "Default angular speed",
        )
        self.force_stale_timeout_sec = self._positive(
            force_stale_timeout_sec,
            "Force stale timeout",
        )
        self.retreat_distance_m = self._positive(
            retreat_distance_m,
            "Contact retreat distance",
        )
        self.retreat_speed_mps = self._positive(
            retreat_speed_mps,
            "Contact retreat speed",
        )
        self._monotonic_clock = monotonic_clock
        self.reset()

    @property
    def active(self) -> bool:
        return self._phase is not None

    def start(
        self,
        plan_builder,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        if not callable(plan_builder):
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement requires a plan builder",
            )
        self.reset()
        self._plan_builder = plan_builder
        self._force_threshold_override_n = force_threshold_n
        return self._prepare_plan()

    def poll(self) -> ArmMovementUpdate:
        phase = self._phase
        if phase is _Phase.FORCE_BASELINE:
            return self._handle_force_baseline(
                self.force_baseline_sampler.poll()
            )
        if phase is _Phase.PLAN_WAIT:
            return self._prepare_plan()
        if phase is _Phase.MOVING:
            return self._poll_primary_motion()
        if phase in (
            _Phase.CONTACT_STOPPING,
            _Phase.ABORT_STOPPING,
        ):
            return self._handle_stop_settling(
                self.settling_detector.poll()
            )
        if phase is _Phase.RETREATING:
            return self._poll_retreat()
        if phase is _Phase.POST_RETREAT_SETTLING:
            return self._handle_post_retreat_settling(
                self.settling_detector.poll()
            )
        return self._terminal(
            ArmMovementOutcome.EXECUTION_ERROR,
            "No guarded probe movement is active",
        )

    def cancel(self) -> None:
        if self.active:
            self._cancel_goal()
        self.reset()

    def reset(self) -> None:
        self._phase = None
        self._plan_builder = None
        self._plan = None
        self._force_baseline = None
        self._force_threshold_override_n = None
        self._force_threshold_n = None
        self._force_last_received_at = None
        self._force_contact_count = 0
        self._contact_detail = ""
        self._abort_outcome = None
        self._abort_detail = ""
        self._retreat_distance_m = 0.0
        self._peak_opposing_force_delta_n = 0.0
        self._peak_total_force_delta_n = 0.0
        self._last_hand_orientation = None
        self.settling_detector.reset()
        self.force_baseline_sampler.reset()

    def _begin_force_baseline(self) -> ArmMovementUpdate:
        self._phase = _Phase.FORCE_BASELINE
        try:
            update = self.force_baseline_sampler.start()
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Could not start force baseline acquisition: {exception}",
            )
        return self._handle_force_baseline(update)

    def _handle_force_baseline(self, update) -> ArmMovementUpdate:
        if update.outcome is ForceBaselineOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                update.detail,
            )
        if update.outcome is ForceBaselineOutcome.READY:
            self._force_baseline = update.baseline
            return self._start_primary_motion()
        mapping = {
            ForceBaselineOutcome.FORCE_UNAVAILABLE: (
                ArmMovementOutcome.FORCE_UNAVAILABLE
            ),
            ForceBaselineOutcome.FORCE_STALE: (
                ArmMovementOutcome.FORCE_STALE
            ),
            ForceBaselineOutcome.FORCE_UNSTABLE: (
                ArmMovementOutcome.FORCE_UNSTABLE
            ),
        }
        return self._terminal(
            mapping.get(
                update.outcome,
                ArmMovementOutcome.FORCE_UNAVAILABLE,
            ),
            update.detail,
        )

    def _prepare_plan(self) -> ArmMovementUpdate:
        builder = self._plan_builder
        if builder is None:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no pending plan builder",
            )
        try:
            plan = builder()
        except MovementGeometryUnavailable as exception:
            self._phase = _Phase.PLAN_WAIT
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                str(exception),
            )
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                str(exception),
            )

        self._plan = plan
        if not isinstance(plan, ProbeMotionPlan):
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe plan builder returned an invalid plan",
            )
        if not plan.motion_required:
            return self._terminal(
                ArmMovementOutcome.SUCCESS,
                "Skipped zero arm movement",
            )
        if not plan.force_guard_enabled:
            return self._start_primary_motion_without_force_guard()
        try:
            self._force_threshold_n = self._resolve_force_threshold(
                plan.linear_speed_mps,
                self._force_threshold_override_n,
            )
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                str(exception),
            )
        return self._begin_force_baseline()

    def _start_primary_motion_without_force_guard(
        self,
    ) -> ArmMovementUpdate:
        plan = self._plan
        if plan is None or plan.goal is None:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Unguarded rotation movement lost its plan",
            )

        self._phase = _Phase.MOVING
        update = self._start_goal(plan.goal)
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        return self._terminal(update.outcome, update.detail)

    def _start_primary_motion(self) -> ArmMovementUpdate:
        plan = self._plan
        baseline = self._force_baseline
        if plan is None or baseline is None:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement lost its plan or force baseline",
            )

        self._force_last_received_at = baseline.last_received_at
        self._force_contact_count = 0
        self._last_hand_orientation = deepcopy(
            plan.current_hand.pose.orientation
        )
        self._phase = _Phase.MOVING
        update = self._start_goal(plan.goal)
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        return self._terminal(update.outcome, update.detail)

    def _poll_primary_motion(self) -> ArmMovementUpdate:
        plan = self._plan
        if plan is None:
            return self._terminal(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement lost its plan",
            )

        if plan.force_guard_enabled:
            guard_update = self._check_force_guard()
            if guard_update is not None:
                return guard_update

        update = self._poll_goal()
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        if update.outcome is ArmMovementOutcome.SUCCESS:
            if not plan.force_guard_enabled:
                return self._terminal(
                    update.outcome,
                    f"{update.detail}; translational force guard skipped "
                    "for rotation-only movement",
                )
            return self._terminal(
                update.outcome,
                f"{update.detail}; peak opposing force delta "
                f"{self._peak_opposing_force_delta_n:.2f} N, peak total "
                f"force delta {self._peak_total_force_delta_n:.2f} N, "
                f"directional threshold "
                f"{self._force_threshold_n:.2f} N at "
                f"{plan.linear_speed_mps:.4f} m/s",
            )
        return self._terminal(update.outcome, update.detail)

    def _check_force_guard(self):
        now = self._monotonic_clock()
        sample = self.arm_state_source.hand_force_sample()

        if sample is None:
            if not self._force_timed_out(now):
                return None
            if getattr(
                self.arm_state_source,
                "last_received_at",
                None,
            ) is None:
                return self._begin_abort(
                    ArmMovementOutcome.FORCE_UNAVAILABLE,
                    "End-effector force is unavailable during "
                    "guarded probe movement",
                )
            return self._begin_abort(
                ArmMovementOutcome.FORCE_STALE,
                "End-effector force became stale during "
                "guarded probe movement",
            )

        if (
            self._force_last_received_at is not None
            and sample.received_at
            <= self._force_last_received_at + 1e-12
        ):
            if self._force_timed_out(now):
                return self._begin_abort(
                    ArmMovementOutcome.FORCE_STALE,
                    "End-effector force stopped updating during "
                    "guarded probe movement",
                )
            return None

        self._force_last_received_at = sample.received_at
        baseline = self._force_baseline
        plan = self._plan
        if baseline is None or plan is None:
            return self._begin_abort(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement lost its force baseline or plan",
            )

        try:
            hand_orientation = self._current_hand_orientation(plan)
            force_delta = directional_force_delta(
                baseline_force_hand=(
                    baseline.x_n,
                    baseline.y_n,
                    baseline.z_n,
                ),
                current_force_hand=(
                    sample.x_n,
                    sample.y_n,
                    sample.z_n,
                ),
                baseline_hand_orientation=(
                    plan.current_hand.pose.orientation
                ),
                current_hand_orientation=hand_orientation,
                movement_direction=(
                    plan.direction_x,
                    plan.direction_y,
                    plan.direction_z,
                ),
            )
        except Exception as exception:
            return self._begin_abort(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Could not evaluate directional contact force: "
                f"{exception}",
            )

        self._peak_opposing_force_delta_n = max(
            self._peak_opposing_force_delta_n,
            force_delta.opposing_n,
        )
        self._peak_total_force_delta_n = max(
            self._peak_total_force_delta_n,
            force_delta.total_n,
        )

        threshold_n = self._force_threshold_n
        if threshold_n is None:
            return self._begin_abort(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Translational force guard has no threshold",
            )
        if force_delta.opposing_n >= threshold_n:
            self._force_contact_count += 1
        else:
            self._force_contact_count = 0

        if (
            self._force_contact_count
            < self.force_contact_policy.consecutive_samples
        ):
            return None

        return self._begin_contact(
            "Contact detected from opposing end-effector force delta "
            f"{force_delta.opposing_n:.2f} N exceeding directional "
            f"threshold {threshold_n:.2f} N at "
            f"{plan.linear_speed_mps:.4f} m/s; peak opposing "
            f"{self._peak_opposing_force_delta_n:.2f} N, peak total "
            f"{self._peak_total_force_delta_n:.2f} N"
        )

    def _current_hand_orientation(self, plan):
        frame_id = str(plan.direction_frame).strip()
        if not frame_id:
            raise ValueError(
                "Guarded probe plan has no movement direction frame"
            )

        try:
            current_hand = self._current_hand_pose(frame_id)
        except Exception:
            orientation = self._last_hand_orientation
            if orientation is None:
                raise
            return orientation

        if current_hand.header.frame_id.strip() != frame_id:
            raise ValueError(
                "Measured hand pose is not expressed in the movement "
                f"direction frame '{frame_id}'"
            )
        self._last_hand_orientation = deepcopy(
            current_hand.pose.orientation
        )
        return self._last_hand_orientation

    def _resolve_force_threshold(
        self,
        linear_speed_mps: float,
        override_n,
    ) -> float:
        if override_n is None:
            return self.force_contact_policy.threshold_for(
                linear_speed_mps
            )

        threshold = float(override_n)
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError(
                "Guarded probe force threshold override must be "
                "positive and finite"
            )
        return threshold

    def _force_timed_out(self, now: float) -> bool:
        if self._force_last_received_at is None:
            return True
        return (
            now - self._force_last_received_at
            >= self.force_stale_timeout_sec
        )

    def _begin_contact(self, detail: str) -> ArmMovementUpdate:
        self._contact_detail = str(detail)
        self._cancel_goal()
        self._phase = _Phase.CONTACT_STOPPING
        try:
            update = self.settling_detector.start()
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                "Contact detected, but physical stop confirmation "
                f"could not start: {exception}",
            )
        return self._handle_stop_settling(update)

    def _begin_abort(
        self,
        outcome: ArmMovementOutcome,
        detail: str,
    ) -> ArmMovementUpdate:
        self._abort_outcome = outcome
        self._abort_detail = str(detail)
        self._cancel_goal()
        self._phase = _Phase.ABORT_STOPPING
        try:
            update = self.settling_detector.start()
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                f"{detail}; physical stop confirmation could not start: "
                f"{exception}",
            )
        return self._handle_stop_settling(update)

    def _handle_stop_settling(self, update) -> ArmMovementUpdate:
        if update.outcome is HandSettlingOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for physical arm stop after cancellation: "
                f"{update.detail}",
            )
        if update.outcome is not HandSettlingOutcome.SETTLED:
            reason = (
                self._contact_detail
                or self._abort_detail
                or "Guarded probe movement was cancelled"
            )
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                f"{reason}; physical stop could not be confirmed: "
                f"{update.detail}",
            )

        if self._phase is _Phase.ABORT_STOPPING:
            return self._terminal(
                self._abort_outcome
                or ArmMovementOutcome.EXECUTION_ERROR,
                self._abort_detail,
            )
        return self._begin_retreat()

    def _begin_retreat(self) -> ArmMovementUpdate:
        plan = self._plan
        if plan is None:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                "Contact retreat has no guarded probe plan",
            )

        try:
            current_hand = self._current_hand_pose(
                plan.direction_frame
            )
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                "Could not measure hand pose for contact retreat: "
                f"{exception}",
            )

        start = plan.current_hand.pose.position
        current = current_hand.pose.position
        forward = (
            (current.x - start.x) * plan.direction_x
            + (current.y - start.y) * plan.direction_y
            + (current.z - start.z) * plan.direction_z
        )
        self._retreat_distance_m = min(
            self.retreat_distance_m,
            max(0.0, forward),
        )
        if self._retreat_distance_m <= 1e-4:
            return self._terminal(
                ArmMovementOutcome.CONTACT,
                f"{self._contact_detail}; arm stopped before measurable "
                "forward travel, so no local retreat was required",
            )

        target = deepcopy(current_hand)
        target.pose.position.x -= (
            plan.direction_x * self._retreat_distance_m
        )
        target.pose.position.y -= (
            plan.direction_y * self._retreat_distance_m
        )
        target.pose.position.z -= (
            plan.direction_z * self._retreat_distance_m
        )
        speed = ArmMotionSpeed(
            linear_speed_mps=self.retreat_speed_mps,
            angular_speed_rad_s=self.default_angular_speed_rad_s,
        )
        try:
            goal = self._build_motion_goal(
                current_hand,
                target,
                speed,
            )
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                f"Could not build contact retreat: {exception}",
            )

        self._phase = _Phase.RETREATING
        update = self._start_goal(goal)
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        return self._terminal(
            ArmMovementOutcome.RETREAT_FAILED,
            f"Contact retreat could not start: {update.detail}",
        )

    def _poll_retreat(self) -> ArmMovementUpdate:
        update = self._poll_goal()
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                f"Contact retreat failed: {update.detail}",
            )

        self._phase = _Phase.POST_RETREAT_SETTLING
        try:
            settling = self.settling_detector.start()
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                "Contact retreat completed, but settling confirmation "
                f"could not start: {exception}",
            )
        return self._handle_post_retreat_settling(settling)

    def _handle_post_retreat_settling(
        self,
        update,
    ) -> ArmMovementUpdate:
        if update.outcome is HandSettlingOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Contact retreat completed; waiting for arm to settle: "
                f"{update.detail}",
            )
        if update.outcome is not HandSettlingOutcome.SETTLED:
            return self._terminal(
                ArmMovementOutcome.RETREAT_FAILED,
                "Contact retreat completed, but the arm did not settle: "
                f"{update.detail}",
            )
        return self._terminal(
            ArmMovementOutcome.CONTACT,
            f"{self._contact_detail}; retreated "
            f"{self._retreat_distance_m:.4f} m opposite the "
            "measured travel direction",
        )

    def _terminal(
        self,
        outcome: ArmMovementOutcome,
        detail: str,
    ) -> ArmMovementUpdate:
        update = ArmMovementUpdate(
            outcome,
            str(detail).strip(),
        )
        self._phase = None
        return update

    @staticmethod
    def _positive(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized


__all__ = ["GuardedProbeExecution"]
