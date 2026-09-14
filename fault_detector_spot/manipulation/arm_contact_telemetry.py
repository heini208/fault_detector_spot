"""Shadow telemetry for guarded arm contact analysis."""

from dataclasses import asdict
import json
import math
from pathlib import Path
from threading import RLock
import time

from fault_detector_spot.manipulation.arm_contact_observation import (
    ArmContactObservation,
)
from fault_detector_spot.manipulation.arm_joint_state_source import (
    ARM_JOINT_NAMES,
)
from fault_detector_spot.shared.persistence.runtime_paths import (
    fault_detector_runtime_root,
)


RAW_LOGGING_PARAMETER = "arm.contact.telemetry.raw_logging_enabled"
CONTACT_TELEMETRY_DIRECTORY = "contact_telemetry"


class ArmContactTelemetry:
    """Collect synchronized contact evidence without controlling movement."""

    def __init__(
        self,
        arm_state_source,
        arm_joint_state_source,
        raw_logging_enabled: bool = False,
        raw_log_root=None,
        logger=None,
    ):
        if arm_state_source is None:
            raise RuntimeError("ArmContactTelemetry requires arm state")
        if arm_joint_state_source is None:
            raise RuntimeError("ArmContactTelemetry requires joint state")

        self.arm_state_source = arm_state_source
        self.arm_joint_state_source = arm_joint_state_source
        self.raw_logging_enabled = bool(raw_logging_enabled)
        self.raw_log_root = Path(
            raw_log_root
            if raw_log_root is not None
            else fault_detector_runtime_root() / CONTACT_TELEMETRY_DIRECTORY
        ).expanduser()
        self.logger = logger
        self._lock = RLock()
        self._movement_sequence = 0
        self._observation_count = 0
        self._latest_observation = None
        self._last_error = ""
        self._raw_log_path = None
        self._raw_log_file = None
        self._previous_joint_received_at = None
        self._previous_joint_efforts_nm = None
        self._previous_observed_at = None
        self._previous_position_error_m = None

    @classmethod
    def from_node(
        cls,
        node,
        arm_state_source,
        arm_joint_state_source,
    ):
        if not node.has_parameter(RAW_LOGGING_PARAMETER):
            node.declare_parameter(RAW_LOGGING_PARAMETER, False)
        enabled = bool(node.get_parameter(RAW_LOGGING_PARAMETER).value)
        return cls(
            arm_state_source=arm_state_source,
            arm_joint_state_source=arm_joint_state_source,
            raw_logging_enabled=enabled,
            logger=node.get_logger(),
        )

    @property
    def observation_count(self) -> int:
        with self._lock:
            return self._observation_count

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    @property
    def raw_log_path(self):
        with self._lock:
            return self._raw_log_path

    def latest_observation(self):
        with self._lock:
            return self._latest_observation

    def begin_movement(self) -> int:
        with self._lock:
            self._movement_sequence += 1
            sequence = self._movement_sequence
            self._previous_joint_received_at = None
            self._previous_joint_efforts_nm = None
            self._previous_observed_at = None
            self._previous_position_error_m = None
            return sequence

    def observe(
        self,
        *,
        movement_sequence: int,
        observed_at: float,
        elapsed_sec: float,
        phase: str,
        plan,
        force_sample,
        force_baseline,
        force_delta,
        current_hand=None,
        contact_evidence=None,
        authoritative_contact_count: int = 0,
        authoritative_decision: str = "unavailable",
        self_motion_suppressed: bool = False,
    ) -> ArmContactObservation:
        velocity = self.arm_state_source.hand_velocity_sample()
        joint_state = self.arm_joint_state_source.sample()

        hand_velocity_received_at = None
        hand_linear_velocity_mps = None
        hand_angular_velocity_rad_s = None
        if velocity is not None:
            hand_velocity_received_at = float(velocity.received_at)
            hand_linear_velocity_mps = (
                float(velocity.linear_x_mps),
                float(velocity.linear_y_mps),
                float(velocity.linear_z_mps),
            )
            hand_angular_velocity_rad_s = (
                float(velocity.angular_x_rad_s),
                float(velocity.angular_y_rad_s),
                float(velocity.angular_z_rad_s),
            )

        joint_state_received_at = None
        joint_positions_rad = None
        joint_velocities_rad_s = None
        joint_efforts_nm = None
        if joint_state is not None:
            joint_state_received_at = float(joint_state.received_at)
            ordered = tuple(
                joint_state.joints[name]
                for name in ARM_JOINT_NAMES
            )
            joint_positions_rad = tuple(
                float(joint.position_rad)
                for joint in ordered
            )
            joint_velocities_rad_s = tuple(
                float(joint.velocity_rad_s)
                for joint in ordered
            )
            joint_efforts_nm = tuple(
                float(joint.effort_nm)
                for joint in ordered
            )

        motion = self._motion_metrics(plan, current_hand)
        direction = (
            float(plan.direction_x),
            float(plan.direction_y),
            float(plan.direction_z),
        )
        max_joint_velocity = self._max_abs(joint_velocities_rad_s)
        effort_rate = self._joint_effort_rate(
            joint_state_received_at,
            joint_efforts_nm,
        )
        progress_rate = self._position_progress_rate(
            observed_at,
            motion["position_error_m"],
        )

        observation = ArmContactObservation(
            movement_sequence=int(movement_sequence),
            observed_at=float(observed_at),
            elapsed_sec=max(0.0, float(elapsed_sec)),
            phase=str(phase),
            planned_linear_speed_mps=float(plan.linear_speed_mps),
            direction_frame=str(plan.direction_frame),
            movement_direction=direction,
            initial_translation_distance_m=(
                motion["initial_translation_distance_m"]
            ),
            current_hand_position_m=motion["current_hand_position_m"],
            current_hand_orientation_xyzw=(
                motion["current_hand_orientation_xyzw"]
            ),
            target_hand_position_m=motion["target_hand_position_m"],
            target_hand_orientation_xyzw=(
                motion["target_hand_orientation_xyzw"]
            ),
            position_error_m=motion["position_error_m"],
            rotation_error_rad=motion["rotation_error_rad"],
            forward_progress_m=motion["forward_progress_m"],
            off_axis_displacement_m=(
                motion["off_axis_displacement_m"]
            ),
            translation_progress_fraction=(
                motion["translation_progress_fraction"]
            ),
            position_progress_rate_mps=progress_rate,
            force_received_at=float(force_sample.received_at),
            force_hand_n=(
                float(force_sample.x_n),
                float(force_sample.y_n),
                float(force_sample.z_n),
            ),
            baseline_force_hand_n=(
                float(force_baseline.x_n),
                float(force_baseline.y_n),
                float(force_baseline.z_n),
            ),
            force_delta_n=(
                float(force_delta.delta_x_n),
                float(force_delta.delta_y_n),
                float(force_delta.delta_z_n),
            ),
            opposing_force_delta_n=float(force_delta.opposing_n),
            total_force_delta_n=float(force_delta.total_n),
            shadow_force_threshold_n=(
                None
                if contact_evidence is None
                else contact_evidence.force_threshold_n
            ),
            shadow_force_threshold_exceeded=(
                None
                if contact_evidence is None
                else contact_evidence.force_threshold_exceeded
            ),
            shadow_force_candidate_count=(
                None
                if contact_evidence is None
                else contact_evidence.force_candidate_count
            ),
            shadow_required_consecutive_samples=(
                None
                if contact_evidence is None
                else contact_evidence.required_consecutive_samples
            ),
            shadow_classification=(
                "unavailable"
                if contact_evidence is None
                else contact_evidence.classification.value
            ),
            shadow_off_axis_speed_threshold_mps=(
                None
                if contact_evidence is None
                else contact_evidence.off_axis_speed_threshold_mps
            ),
            authoritative_contact_count=int(
                authoritative_contact_count
            ),
            authoritative_self_motion_suppressed=bool(
                self_motion_suppressed
            ),
            authoritative_decision=str(authoritative_decision),
            hand_velocity_received_at=hand_velocity_received_at,
            hand_linear_velocity_mps=hand_linear_velocity_mps,
            hand_angular_velocity_rad_s=hand_angular_velocity_rad_s,
            parallel_hand_speed_mps=(
                None
                if contact_evidence is None
                else contact_evidence.parallel_hand_speed_mps
            ),
            off_axis_hand_speed_mps=(
                None
                if contact_evidence is None
                else contact_evidence.off_axis_hand_speed_mps
            ),
            off_axis_speed_ratio=(
                None
                if contact_evidence is None
                else contact_evidence.off_axis_speed_ratio
            ),
            joint_state_received_at=joint_state_received_at,
            joint_names=tuple(ARM_JOINT_NAMES),
            joint_positions_rad=joint_positions_rad,
            joint_velocities_rad_s=joint_velocities_rad_s,
            joint_efforts_nm=joint_efforts_nm,
            max_joint_velocity_rad_s=max_joint_velocity,
            joint_effort_rate_max_nm_s=effort_rate,
        )

        with self._lock:
            self._latest_observation = observation
            self._observation_count += 1
            self._write_raw_locked(observation)
        return observation

    @staticmethod
    def _max_abs(values):
        if values is None:
            return None
        normalized = tuple(float(value) for value in values)
        if not normalized or not all(
            math.isfinite(value) for value in normalized
        ):
            return None
        return max(abs(value) for value in normalized)

    def _joint_effort_rate(self, received_at, efforts):
        rate = None
        if received_at is not None and efforts is not None:
            timestamp = float(received_at)
            current = tuple(float(value) for value in efforts)
            previous_time = self._previous_joint_received_at
            previous = self._previous_joint_efforts_nm
            if (
                math.isfinite(timestamp)
                and current
                and all(math.isfinite(value) for value in current)
            ):
                if (
                    previous_time is not None
                    and previous is not None
                    and len(previous) == len(current)
                    and timestamp > previous_time + 1e-12
                ):
                    dt = timestamp - previous_time
                    rate = max(
                        abs(current_value - previous_value) / dt
                        for current_value, previous_value in zip(
                            current,
                            previous,
                        )
                    )
                if (
                    previous_time is None
                    or timestamp > previous_time + 1e-12
                ):
                    self._previous_joint_received_at = timestamp
                    self._previous_joint_efforts_nm = current
        return rate

    def _position_progress_rate(self, observed_at, position_error_m):
        rate = None
        if position_error_m is not None:
            timestamp = float(observed_at)
            error = float(position_error_m)
            previous_time = self._previous_observed_at
            previous_error = self._previous_position_error_m
            if math.isfinite(timestamp) and math.isfinite(error):
                if (
                    previous_time is not None
                    and previous_error is not None
                    and timestamp > previous_time + 1e-12
                ):
                    rate = (
                        previous_error - error
                    ) / (timestamp - previous_time)
                self._previous_observed_at = timestamp
                self._previous_position_error_m = error
        return rate

    @classmethod
    def _motion_metrics(cls, plan, current_hand):
        start_position = cls._position(plan.current_hand)
        target_position = cls._position(plan.target_hand)
        target_orientation = cls._orientation(plan.target_hand)
        initial_distance = cls._distance(
            start_position,
            target_position,
        )
        result = {
            "initial_translation_distance_m": initial_distance,
            "current_hand_position_m": None,
            "current_hand_orientation_xyzw": None,
            "target_hand_position_m": target_position,
            "target_hand_orientation_xyzw": target_orientation,
            "position_error_m": None,
            "rotation_error_rad": None,
            "forward_progress_m": None,
            "off_axis_displacement_m": None,
            "translation_progress_fraction": None,
        }
        if current_hand is None:
            return result

        current_position = cls._position(current_hand)
        current_orientation = cls._orientation(current_hand)
        position_error = cls._distance(
            current_position,
            target_position,
        )
        displacement = tuple(
            current - start
            for current, start in zip(
                current_position,
                start_position,
            )
        )
        direction = (
            float(plan.direction_x),
            float(plan.direction_y),
            float(plan.direction_z),
        )
        forward_progress = sum(
            value * axis
            for value, axis in zip(displacement, direction)
        )
        off_axis = tuple(
            value - forward_progress * axis
            for value, axis in zip(displacement, direction)
        )
        off_axis_displacement = math.sqrt(
            sum(value * value for value in off_axis)
        )
        progress_fraction = None
        if initial_distance > 1e-9:
            progress_fraction = 1.0 - position_error / initial_distance

        result.update({
            "current_hand_position_m": current_position,
            "current_hand_orientation_xyzw": current_orientation,
            "position_error_m": position_error,
            "rotation_error_rad": cls._rotation_distance_rad(
                current_orientation,
                target_orientation,
            ),
            "forward_progress_m": forward_progress,
            "off_axis_displacement_m": off_axis_displacement,
            "translation_progress_fraction": progress_fraction,
        })
        return result

    @staticmethod
    def _position(pose_stamped):
        position = pose_stamped.pose.position
        return (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    @staticmethod
    def _orientation(pose_stamped):
        orientation = pose_stamped.pose.orientation
        return (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    @staticmethod
    def _distance(first, second) -> float:
        return math.sqrt(
            sum(
                (a - b) * (a - b)
                for a, b in zip(first, second)
            )
        )

    @staticmethod
    def _rotation_distance_rad(first, second) -> float:
        first_norm = math.sqrt(sum(value * value for value in first))
        second_norm = math.sqrt(sum(value * value for value in second))
        if first_norm <= 1e-12 or second_norm <= 1e-12:
            raise ValueError("Arm telemetry quaternion norm is zero")
        dot = abs(
            sum(
                (a / first_norm) * (b / second_norm)
                for a, b in zip(first, second)
            )
        )
        dot = min(1.0, max(0.0, dot))
        return 2.0 * math.acos(dot)

    def _write_raw_locked(self, observation) -> None:
        if not self.raw_logging_enabled:
            return
        try:
            if self._raw_log_file is None:
                self.raw_log_root.mkdir(parents=True, exist_ok=True)
                self._raw_log_path = self.raw_log_root / (
                    f"arm_contact_{time.time_ns()}.jsonl"
                )
                self._raw_log_file = self._raw_log_path.open(
                    "a",
                    encoding="utf-8",
                )
            payload = asdict(observation)
            self._raw_log_file.write(
                json.dumps(payload, separators=(",", ":")) + "\n"
            )
            self._raw_log_file.flush()
        except Exception as exception:
            self._last_error = str(exception)
            self._disable_raw_logging_locked()
            if self.logger is not None:
                try:
                    self.logger.warning(
                        "Arm contact telemetry raw logging disabled: "
                        f"{exception}"
                    )
                except Exception:
                    pass

    def _disable_raw_logging_locked(self) -> None:
        raw_log_file = self._raw_log_file
        self._raw_log_file = None
        self.raw_logging_enabled = False
        if raw_log_file is not None:
            try:
                raw_log_file.close()
            except Exception:
                pass

    def close(self) -> None:
        with self._lock:
            raw_log_file = self._raw_log_file
            self._raw_log_file = None
        if raw_log_file is not None:
            raw_log_file.close()


__all__ = [
    "ArmContactTelemetry",
    "CONTACT_TELEMETRY_DIRECTORY",
    "RAW_LOGGING_PARAMETER",
]
