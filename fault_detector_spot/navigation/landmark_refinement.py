"""Conservative, bounded refinement of a manually established landmark pose."""

from dataclasses import dataclass, fields
import math

import numpy as np
from scipy.spatial.transform import Rotation

from fault_detector_spot.navigation.landmark_observations import LandmarkPoseWindow
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_rotation, rotation_distance_rad, rotation_from_quaternion,
)
from fault_detector_spot.shared.persistence.basic_settings import parameter_override


@dataclass(frozen=True)
class LandmarkRefinementConfig:
    enabled: bool = True
    required_samples: int = 5
    maximum_sample_span_sec: float = 30.0
    maximum_position_spread_m: float = 0.03
    maximum_rotation_spread_rad: float = math.radians(3.0)
    maximum_observation_distance_m: float = 2.5
    maximum_localization_position_std_m: float = 0.10
    maximum_localization_yaw_std_rad: float = math.radians(3.0)
    gain: float = 0.1
    maximum_step_position_m: float = 0.01
    maximum_step_rotation_rad: float = math.radians(0.5)
    maximum_discrepancy_position_m: float = 0.15
    maximum_discrepancy_rotation_rad: float = math.radians(10.0)
    maximum_cumulative_position_m: float = 0.05
    maximum_cumulative_rotation_rad: float = math.radians(3.0)
    minimum_interval_sec: float = 300.0
    attempt_interval_sec: float = 300.0
    sample_interval_sec: float = 0.5
    encounter_absence_sec: float = 5.0

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("Landmark refinement enabled must be a boolean")
        if type(self.required_samples) is not int or self.required_samples < 2:
            raise ValueError("Landmark refinement requires at least two samples")
        for field in fields(self):
            value = getattr(self, field.name)
            if field.type is float and (
                type(value) not in (int, float) or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"Landmark refinement {field.name} must be positive and finite")
        if self.gain > 1.0:
            raise ValueError("Landmark refinement gain must not exceed one")
        for step, cumulative, discrepancy in (
            (self.maximum_step_position_m, self.maximum_cumulative_position_m,
             self.maximum_discrepancy_position_m),
            (self.maximum_step_rotation_rad, self.maximum_cumulative_rotation_rad,
             self.maximum_discrepancy_rotation_rad),
        ):
            if not step <= cumulative <= discrepancy:
                raise ValueError("Refinement limits must satisfy step <= cumulative <= discrepancy")

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(cls):
            key = f"navigation.landmark_refinement.{field.name}"
            if not node.has_parameter(key):
                node.declare_parameter(key, getattr(defaults, field.name))
            values[field.name] = parameter_override(key, node.get_parameter(key).value)
        return cls(**values)

    def new_window(self):
        return LandmarkPoseWindow(
            self.required_samples, self.maximum_sample_span_sec,
            self.maximum_position_spread_m, self.maximum_rotation_spread_rad,
        )


@dataclass
class _TagRefinement:
    window: LandmarkPoseWindow
    visible: bool = False
    absent_since: float | None = None
    encounter_used: bool = False
    last_commit_at: float | None = None
    last_tag_stamp_ns: int = 0
    last_registration_stamp_ns: int = 0
    last_attempt_at: float | None = None
    anchor_signature: tuple | None = None
    proposal: PoseData | None = None


class LandmarkRefiner:
    """Propose small full-3D adjustments; callers own evidence and persistence.

    Each sample must already have passed independent localization, observation
    quality, timestamp-alignment and range checks. This policy is a bounded
    heuristic, not a covariance estimator. A committed adjustment or qualified
    agreement consumes the encounter.
    """

    def __init__(self, config=None):
        self.config = config if config is not None else LandmarkRefinementConfig()
        self._tags = {}
        self._last_time = None
        self._attempt_tag_id = None
        self._attempt_started_at = None
        self._last_attempt_at = None
        self._last_sample_at = None
        self._last_stationarity_sample_at = None
        self._reserved_sample = None
        self.detail = "Waiting for independent landmark observations"

    @staticmethod
    def _validate_time(steady_now):
        if type(steady_now) not in (int, float) or not math.isfinite(steady_now) or steady_now < 0:
            raise ValueError("Refinement time must be non-negative and finite")

    @staticmethod
    def _validate_tag_id(tag_id):
        if type(tag_id) is not int or tag_id < 0:
            raise ValueError("Refinement tag ID must be a non-negative integer")

    def _state(self, tag_id):
        if tag_id not in self._tags:
            self._tags[tag_id] = _TagRefinement(self.config.new_window())
        return self._tags[tag_id]

    @staticmethod
    def _clear_burst(state):
        state.window.clear()
        state.proposal = None

    def reset_observations(self):
        """Drop evidence without restarting the attempt or its cooldown."""
        for state in self._tags.values():
            self._clear_burst(state)
        self._reserved_sample = None

    def reset_context(self):
        """Discard policy state after changing map or localization process."""
        self._tags.clear()
        self._last_time = None
        self._attempt_tag_id = None
        self._attempt_started_at = None
        self._last_attempt_at = None
        self._last_sample_at = None
        self._last_stationarity_sample_at = None
        self._reserved_sample = None
        self.detail = "Waiting for independent landmark observations"

    def observe_visibility(self, visible_ids, steady_now):
        """Recognize a new encounter only after a sustained actual absence."""
        self._validate_time(steady_now)
        visible_ids = set(visible_ids)
        for tag_id in visible_ids:
            self._validate_tag_id(tag_id)
            self._state(tag_id)
        if self._last_time is not None and steady_now < self._last_time:
            self.reset_observations()
            self.detail = "Refinement clock regressed"
            return
        self._last_time = steady_now
        for tag_id, state in self._tags.items():
            if tag_id in visible_ids:
                if (state.absent_since is not None
                        and steady_now - state.absent_since >= self.config.encounter_absence_sec):
                    state.encounter_used = False
                state.absent_since = None
                state.visible = True
            else:
                if state.absent_since is None:
                    state.absent_since = steady_now
                state.visible = False
                self._clear_burst(state)
                if self._reserved_sample is not None and self._reserved_sample[0] == tag_id:
                    self._reserved_sample = None

    def _end_attempt(self):
        if self._attempt_tag_id is not None:
            self._clear_burst(self._tags[self._attempt_tag_id])
        self._attempt_tag_id = None
        self._attempt_started_at = None
        self._last_sample_at = None
        self._last_stationarity_sample_at = None
        self._reserved_sample = None

    def _expire_attempt(self, steady_now):
        if (self._attempt_started_at is not None
                and steady_now - self._attempt_started_at >= self.config.maximum_sample_span_sec):
            self._end_attempt()

    @property
    def attempt_tag_id(self):
        """Tag selected for the bounded attempt, or None between attempts."""
        return self._attempt_tag_id

    def stationarity_sample_due(self, steady_now):
        """Reserve a current-odometry check independently of registration rate.

        The caller still supplies current confidence, command and range gates.
        This slot never starts or extends an attempt, consumes geometry stamps,
        or resets the cooldown.
        """
        self._validate_time(steady_now)
        if self._last_time is not None and steady_now < self._last_time:
            self.reset_observations()
            self.detail = "Refinement clock regressed"
            return False
        self._last_time = steady_now
        self._expire_attempt(steady_now)
        if not self.config.enabled or self._attempt_tag_id is None:
            return False
        if not self._tags[self._attempt_tag_id].visible:
            return False
        if (self._last_stationarity_sample_at is not None
                and steady_now - self._last_stationarity_sample_at < self.config.sample_interval_sec):
            return False
        self._last_stationarity_sample_at = steady_now
        return True

    def begin_samples(self, observations, steady_now):
        """Reserve at most one sample before the caller does quality/TF work.

        ``observations`` maps eligible visible tag IDs to (tag, registration)
        acquisition stamps. A reservation consumes its stamps and sampling slot
        even when subsequent quality or TF checks fail. Failed attempts remain
        bounded by the same deadline and interval as successful attempts.
        """
        self._validate_time(steady_now)
        self._reserved_sample = None
        if self._last_time is not None and steady_now < self._last_time:
            self.reset_observations()
            self.detail = "Refinement clock regressed"
            return set()
        self._last_time = steady_now
        self._expire_attempt(steady_now)
        if not self.config.enabled:
            self.detail = "Refinement disabled"
            return set()
        eligible = []
        for tag_id, stamps in observations.items():
            self._validate_tag_id(tag_id)
            if len(stamps) != 2 or any(type(stamp) is not int or stamp <= 0 for stamp in stamps):
                raise ValueError("Refinement acquisition stamps must be positive integers")
            state = self._state(tag_id)
            if not state.visible:
                self.detail = "Refinement tag not visible"
                continue
            if state.encounter_used:
                self.detail = "Landmark already refined during this encounter"
                continue
            if (state.last_commit_at is not None
                    and steady_now - state.last_commit_at < self.config.minimum_interval_sec):
                self.detail = "Waiting for landmark refinement interval"
                continue
            if stamps[0] < state.last_tag_stamp_ns or stamps[1] < state.last_registration_stamp_ns:
                self._clear_burst(state)
                self.detail = "Refinement acquisition timestamps regressed"
                continue
            if stamps[0] == state.last_tag_stamp_ns or stamps[1] == state.last_registration_stamp_ns:
                self.detail = "Waiting for a new tag capture and accepted registration"
                continue
            eligible.append(tag_id)
        if not eligible:
            return set()
        if self._attempt_tag_id is None:
            if (self._last_attempt_at is not None
                    and steady_now - self._last_attempt_at < self.config.attempt_interval_sec):
                self.detail = "Waiting for landmark refinement attempt interval"
                return set()
            # Rotate through visible candidates instead of starving later IDs
            # when one landmark repeatedly fails its geometry/quality checks.
            tag_id = min(eligible, key=lambda value: (
                self._tags[value].last_attempt_at
                if self._tags[value].last_attempt_at is not None else -math.inf, value,
            ))
            self._attempt_tag_id = tag_id
            self._attempt_started_at = self._last_attempt_at = steady_now
            self._tags[tag_id].last_attempt_at = steady_now
            self._clear_burst(self._tags[tag_id])
        elif self._attempt_tag_id not in eligible:
            return set()
        if (self._last_stationarity_sample_at is not None
                and steady_now - self._last_stationarity_sample_at < self.config.sample_interval_sec):
            return set()
        if (self._last_sample_at is not None
                and steady_now - self._last_sample_at < self.config.sample_interval_sec):
            return set()
        tag_id = self._attempt_tag_id
        tag_stamp_ns, registration_stamp_ns = observations[tag_id]
        state = self._tags[tag_id]
        state.last_tag_stamp_ns = tag_stamp_ns
        state.last_registration_stamp_ns = registration_stamp_ns
        self._last_sample_at = steady_now
        self._reserved_sample = (tag_id, tag_stamp_ns, registration_stamp_ns, steady_now)
        return {tag_id}

    def observe(
        self, tag_id, tag_stamp_ns, registration_stamp_ns,
        observed_pose, current_pose, original_pose, steady_now,
    ):
        """Return a bounded proposal after distinct, consistent evidence."""
        self._validate_tag_id(tag_id)
        self._validate_time(steady_now)
        state = self._state(tag_id)
        reservation = (tag_id, tag_stamp_ns, registration_stamp_ns, steady_now)
        if self._reserved_sample != reservation:
            if tag_id not in self.begin_samples({tag_id: (tag_stamp_ns, registration_stamp_ns)}, steady_now):
                return None
        self._reserved_sample = None
        try:
            for pose in (observed_pose, current_pose, original_pose):
                if not isinstance(pose, PoseData):
                    raise TypeError("Refinement requires full PoseData values")
                pose.validate()
        except (TypeError, ValueError):
            self._clear_burst(state)
            raise
        signature = (current_pose.to_dict(), original_pose.to_dict())
        if state.anchor_signature != signature:
            self._clear_burst(state)
            state.anchor_signature = signature
        state.proposal = None
        discrepancy = self._distance(current_pose, observed_pose)
        if (discrepancy[0] > self.config.maximum_discrepancy_position_m
                or discrepancy[1] > self.config.maximum_discrepancy_rotation_rad):
            self._clear_burst(state)
            self.detail = "Observed landmark differs too much; explicit resaving is required"
            return None
        if state.window.add(tag_stamp_ns, observed_pose) is None:
            self.detail = state.window.detail
            return None
        mean = state.window.qualified_mean_pose()
        discrepancy = self._distance(current_pose, mean)
        if (discrepancy[0] > self.config.maximum_discrepancy_position_m
                or discrepancy[1] > self.config.maximum_discrepancy_rotation_rad):
            self._clear_burst(state)
            self.detail = "Observed landmark differs too much; explicit resaving is required"
            return None
        step = (self._position(mean) - self._position(current_pose)) * self.config.gain
        magnitude = float(np.linalg.norm(step))
        if magnitude > self.config.maximum_step_position_m:
            step *= self.config.maximum_step_position_m / magnitude
        current_rotation = rotation_from_quaternion(current_pose.orientation)
        difference = current_rotation.inv() * rotation_from_quaternion(mean.orientation)
        rotation_step = difference.as_rotvec() * self.config.gain
        angle = float(np.linalg.norm(rotation_step))
        if angle > self.config.maximum_step_rotation_rad:
            rotation_step *= self.config.maximum_step_rotation_rad / angle
        proposal = PoseData(
            position=Vector3Data(*(float(value) for value in self._position(current_pose) + step)),
            orientation=quaternion_from_rotation(current_rotation * Rotation.from_rotvec(rotation_step)),
        )
        cumulative = self._distance(original_pose, proposal)
        if (cumulative[0] > self.config.maximum_cumulative_position_m + 1e-12
                or cumulative[1] > self.config.maximum_cumulative_rotation_rad + 1e-12):
            self._clear_burst(state)
            self.detail = "Landmark refinement reached its original-anchor limit"
            return None
        actual_step = self._distance(current_pose, proposal)
        if actual_step[0] < 1e-12 and actual_step[1] < 1e-12:
            self.detail = "Landmark already agrees with independent observations"
            state.encounter_used = True
            self._end_attempt()
            return None
        state.proposal = proposal
        self.detail = "Bounded landmark refinement ready"
        return PoseData.from_dict(proposal.to_dict())

    def commit(self, tag_id, steady_now):
        """Consume the encounter only after the proposed pose was persisted."""
        self._validate_tag_id(tag_id)
        self._validate_time(steady_now)
        self._expire_attempt(steady_now)
        state = self._tags.get(tag_id)
        if state is None or state.proposal is None:
            raise ValueError("No landmark refinement proposal is ready to commit")
        if self._last_time is not None and steady_now < self._last_time:
            raise ValueError("Refinement clock regressed before commit")
        state.encounter_used = True
        state.last_commit_at = steady_now
        self._end_attempt()

    @staticmethod
    def _position(pose):
        return np.array((pose.position.x, pose.position.y, pose.position.z), dtype=float)

    @classmethod
    def _distance(cls, first, second):
        return (
            float(np.linalg.norm(cls._position(first) - cls._position(second))),
            rotation_distance_rad(first.orientation, second.orientation),
        )


__all__ = ["LandmarkRefinementConfig", "LandmarkRefiner"]
