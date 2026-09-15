"""Force guard state machine for one probe movement."""

from copy import deepcopy
from enum import Enum
import math
import time

from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
    ShadowContactClassification,
)
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
    ARM_STOPPING = "arm_stopping"
    ARM_STOP_SETTLING = "arm_stop_settling"
    RETREATING = "retreating"


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
        start_stop,
        poll_stop,
        current_hand_pose,
        build_motion_goal,
        default_angular_speed_rad_s: float,
        force_stale_timeout_sec: float,
        retreat_distance_m: float,
        retreat_speed_mps: float,
        contact_evidence_analyzer=None,
        contact_telemetry=None,
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
            (start_stop, "start arm stop"),
            (poll_stop, "poll arm stop"),
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
        self._start_stop = start_stop
        self._poll_stop = poll_stop
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
        self.contact_evidence_analyzer = (
            contact_evidence_analyzer
            if contact_evidence_analyzer is not None
            else ArmContactEvidenceAnalyzer()
        )
        self.contact_telemetry = contact_telemetry
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
        if phase is _Phase.ARM_STOPPING:
            return self._poll_arm_stop()
        if phase is _Phase.ARM_STOP_SETTLING:
            return self._handle_arm_stop_settling(
                self.settling_detector.poll()
            )
        if phase is _Phase.RETREATING:
            return self._poll_retreat()
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
        self._self_motion_suppression_count = 0
        self._contact_detail = ""
        self._abort_outcome = None
        self._abort_detail = ""
        self._retreat_distance_m = 0.0
        self._peak_opposing_force_delta_n = 0.0
        self._peak_total_force_delta_n = 0.0
        self._last_hand_orientation = None
        self._primary_motion_started_at = None
        self._telemetry_movement_sequence = None
        self._stop_terminal_outcome = None
        self._stop_terminal_detail = ""
        self._stop_then_retreat = False
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
                plan.angular_speed_rad_s,
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
        self._self_motion_suppression_count = 0
        try:
            self.contact_evidence_analyzer.begin_movement()
        except Exception:
            pass
        self._last_hand_orientation = deepcopy(
            plan.current_hand.pose.orientation
        )
        self._primary_motion_started_at = self._monotonic_clock()
        self._telemetry_movement_sequence = (
            self._begin_contact_telemetry()
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
        if update.outcome in (
            ArmMovementOutcome.GOAL_REJECTED,
            ArmMovementOutcome.GOAL_RESPONSE_TIMEOUT,
        ):
            return self._terminal(update.outcome, update.detail)

        detail = update.detail
        if update.outcome is ArmMovementOutcome.SUCCESS:
            if not plan.force_guard_enabled:
                return self._terminal(update.outcome, detail)
            if plan.translational_motion:
                detail = (
                    f"{detail}; peak opposing force delta "
                    f"{self._peak_opposing_force_delta_n:.2f} N, "
                    f"peak total force delta "
                    f"{self._peak_total_force_delta_n:.2f} N, "
                    f"motion threshold "
                    f"{self._force_threshold_n:.2f} N at "
                    f"{plan.linear_speed_mps:.4f} m/s linear, "
                    f"{plan.angular_speed_rad_s:.4f} rad/s angular"
                )
                return self._begin_arm_stop(
                    terminal_outcome=update.outcome,
                    terminal_detail=detail,
                )
            detail = (
                f"{detail}; peak total force delta "
                f"{self._peak_total_force_delta_n:.2f} N, "
                f"orientation threshold "
                f"{self._force_threshold_n:.2f} N at "
                f"{plan.angular_speed_rad_s:.4f} rad/s angular"
            )
            return self._terminal(update.outcome, detail)
        return self._begin_arm_stop(
            terminal_outcome=update.outcome,
            terminal_detail=detail,
        )

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
            hand_orientation, current_hand = (
                self._current_hand_measurement(plan)
            )
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
                "Force guard has no threshold",
            )

        evidence = None
        self_motion_suppressed = False
        if plan.translational_motion:
            evidence = self._contact_evidence(
                plan=plan,
                force_delta=force_delta,
                force_threshold_n=threshold_n,
            )
            self_motion_suppressed = (
                evidence is not None
                and evidence.classification
                is ShadowContactClassification.LIKELY_SELF_MOTION
            )

        if force_delta.opposing_n < threshold_n:
            self._force_contact_count = 0
            authoritative_decision = "below_threshold"
        elif self_motion_suppressed:
            self._force_contact_count = 0
            self._self_motion_suppression_count += 1
            authoritative_decision = "self_motion_suppressed"
        else:
            self._force_contact_count += 1
            if (
                self._force_contact_count
                >= self.force_contact_policy.consecutive_samples
            ):
                authoritative_decision = "contact"
            else:
                authoritative_decision = "force_candidate"

        self._observe_contact_telemetry(
            observed_at=now,
            plan=plan,
            force_sample=sample,
            force_baseline=baseline,
            force_delta=force_delta,
            current_hand=current_hand,
            contact_evidence=evidence,
            authoritative_contact_count=self._force_contact_count,
            authoritative_decision=authoritative_decision,
            self_motion_suppressed=self_motion_suppressed,
        )

        if (
            self._force_contact_count
            < self.force_contact_policy.consecutive_samples
        ):
            return None

        if plan.translational_motion:
            detail = (
                "Contact detected from opposing end-effector force delta "
                f"{force_delta.opposing_n:.2f} N exceeding motion "
                f"threshold {threshold_n:.2f} N at "
                f"{plan.linear_speed_mps:.4f} m/s linear, "
                f"{plan.angular_speed_rad_s:.4f} rad/s angular; "
                f"peak opposing {self._peak_opposing_force_delta_n:.2f} N, "
                f"peak total {self._peak_total_force_delta_n:.2f} N"
            )
        else:
            detail = (
                "Contact detected from total end-effector force delta "
                f"{force_delta.total_n:.2f} N exceeding orientation "
                f"threshold {threshold_n:.2f} N at "
                f"{plan.angular_speed_rad_s:.4f} rad/s angular; "
                f"peak total {self._peak_total_force_delta_n:.2f} N"
            )
        return self._begin_contact(detail)

    def _contact_evidence(
        self,
        *,
        plan,
        force_delta,
        force_threshold_n,
    ):
        if not plan.translational_motion:
            return None
        try:
            velocity_method = getattr(
                self.arm_state_source,
                "hand_velocity_sample",
                None,
            )
            velocity = (
                velocity_method() if callable(velocity_method) else None
            )
            hand_linear_velocity = None
            if velocity is not None:
                hand_linear_velocity = (
                    velocity.linear_x_mps,
                    velocity.linear_y_mps,
                    velocity.linear_z_mps,
                )
            return self.contact_evidence_analyzer.analyze(
                force_threshold_n=force_threshold_n,
                required_consecutive_samples=(
                    self.force_contact_policy.consecutive_samples
                ),
                movement_direction=(
                    plan.direction_x,
                    plan.direction_y,
                    plan.direction_z,
                ),
                opposing_force_delta_n=force_delta.opposing_n,
                hand_linear_velocity_mps=hand_linear_velocity,
            )
        except Exception:
            return None

    def _begin_contact_telemetry(self):
        telemetry = self.contact_telemetry
        if telemetry is None:
            return None
        try:
            return telemetry.begin_movement()
        except Exception:
            return None

    def _observe_contact_telemetry(
        self,
        *,
        observed_at,
        plan,
        force_sample,
        force_baseline,
        force_delta,
        current_hand,
        contact_evidence,
        authoritative_contact_count,
        authoritative_decision,
        self_motion_suppressed,
    ) -> None:
        telemetry = self.contact_telemetry
        sequence = self._telemetry_movement_sequence
        started_at = self._primary_motion_started_at
        if telemetry is None or sequence is None or started_at is None:
            return
        try:
            telemetry.observe(
                movement_sequence=sequence,
                observed_at=observed_at,
                elapsed_sec=observed_at - started_at,
                phase=_Phase.MOVING.value,
                plan=plan,
                force_sample=force_sample,
                force_baseline=force_baseline,
                force_delta=force_delta,
                current_hand=current_hand,
                contact_evidence=contact_evidence,
                authoritative_contact_count=(
                    authoritative_contact_count
                ),
                authoritative_decision=authoritative_decision,
                self_motion_suppressed=self_motion_suppressed,
            )
        except Exception:
            return

    def _current_hand_measurement(self, plan):
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
            return orientation, None

        if current_hand.header.frame_id.strip() != frame_id:
            raise ValueError(
                "Measured hand pose is not expressed in the movement "
                f"direction frame '{frame_id}'"
            )
        self._last_hand_orientation = deepcopy(
            current_hand.pose.orientation
        )
        return self._last_hand_orientation, current_hand

    def _resolve_force_threshold(
        self,
        linear_speed_mps: float,
        angular_speed_rad_s: float,
        override_n,
    ) -> float:
        if override_n is None:
            threshold_for = self.force_contact_policy.threshold_for
            try:
                return threshold_for(
                    linear_speed_mps,
                    angular_speed_rad_s,
                )
            except TypeError as exception:
                if angular_speed_rad_s > 1e-12:
                    raise TypeError(
                        "Force contact policy must accept linear and "
                        "angular speed for rotational guarded motion"
                    ) from exception
                return threshold_for(linear_speed_mps)

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
        plan = self._plan
        return self._begin_arm_stop(
            terminal_outcome=ArmMovementOutcome.CONTACT,
            terminal_detail=self._contact_detail,
            then_retreat=bool(
                plan is not None and plan.translational_motion
            ),
        )

    def _begin_abort(
        self,
        outcome: ArmMovementOutcome,
        detail: str,
    ) -> ArmMovementUpdate:
        self._abort_outcome = outcome
        self._abort_detail = str(detail)
        self._cancel_goal()
        return self._begin_arm_stop(
            terminal_outcome=outcome,
            terminal_detail=self._abort_detail,
        )

    def _begin_arm_stop(
        self,
        *,
        terminal_outcome: ArmMovementOutcome,
        terminal_detail: str,
        then_retreat: bool = False,
    ) -> ArmMovementUpdate:
        self._stop_terminal_outcome = terminal_outcome
        self._stop_terminal_detail = str(terminal_detail).strip()
        self._stop_then_retreat = bool(then_retreat)

        self._phase = _Phase.ARM_STOPPING
        update = self._start_stop()
        if update.outcome is ArmMovementOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Arm movement ended; ArmStopCommand sent",
            )
        return self._terminal(
            ArmMovementOutcome.STOP_UNCONFIRMED,
            self._stop_detail(
                f"ArmStopCommand could not start: {update.detail}"
            ),
        )

    def _poll_arm_stop(self) -> ArmMovementUpdate:
        update = self._poll_stop()
        if update.outcome is ArmMovementOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for ArmStopCommand service response",
            )
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                self._stop_detail(
                    f"ArmStopCommand failed: {update.detail}"
                ),
            )

        self._phase = _Phase.ARM_STOP_SETTLING
        try:
            settling = self.settling_detector.start()
        except Exception as exception:
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                self._stop_detail(
                    "ArmStopCommand was accepted, but physical settling "
                    f"confirmation could not start: {exception}"
                ),
            )
        return self._handle_arm_stop_settling(settling)

    def _handle_arm_stop_settling(
        self,
        update,
    ) -> ArmMovementUpdate:
        if update.outcome is HandSettlingOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "ArmStopCommand accepted; waiting for physical "
                f"stability: {update.detail}",
            )
        if update.outcome is HandSettlingOutcome.TIMEOUT:
            return self._terminal(
                ArmMovementOutcome.UNSTABLE_ARM,
                self._stop_detail(
                    "ArmStopCommand was accepted, but the hand remained "
                    f"unstable: {update.detail}"
                ),
            )
        if update.outcome is not HandSettlingOutcome.SETTLED:
            return self._terminal(
                ArmMovementOutcome.STOP_UNCONFIRMED,
                self._stop_detail(
                    "ArmStopCommand was accepted, but physical stability "
                    f"could not be confirmed: {update.detail}"
                ),
            )

        if self._stop_then_retreat:
            return self._begin_retreat()

        outcome = (
            self._stop_terminal_outcome
            or ArmMovementOutcome.EXECUTION_ERROR
        )
        detail = self._stop_terminal_detail
        suffix = "ArmStopCommand accepted and physical stability confirmed"
        return self._terminal(
            outcome,
            f"{detail}; {suffix}" if detail else suffix,
        )

    def _stop_detail(self, detail: str) -> str:
        original = self._stop_terminal_detail.strip()
        normalized = str(detail).strip()
        if not original:
            return normalized
        if not normalized:
            return original
        return f"{original}; {normalized}"

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

        if update.outcome is ArmMovementOutcome.SUCCESS:
            outcome = ArmMovementOutcome.CONTACT
            detail = (
                f"{self._contact_detail}; retreated "
                f"{self._retreat_distance_m:.4f} m opposite the "
                "measured travel direction"
            )
        else:
            outcome = ArmMovementOutcome.RETREAT_FAILED
            detail = f"Contact retreat failed: {update.detail}"

        return self._begin_arm_stop(
            terminal_outcome=outcome,
            terminal_detail=detail,
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


__all__ = ["GuaredProbeExecution"]
