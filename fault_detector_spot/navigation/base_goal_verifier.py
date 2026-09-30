"""Measured endpoint verification for planar base movements."""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class BaseGoalVerificationConfig:
    position_tolerance_m: float = 0.03
    yaw_tolerance_rad: float = math.radians(3.0)
    settle_sec: float = 0.5
    timeout_sec: float = 5.0
    maximum_pose_age_sec: float = 0.5
    settle_position_tolerance_m: float = 0.005
    settle_yaw_tolerance_rad: float = math.radians(0.5)

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"Base verification {field.name} "
                    "must be positive and finite"
                )
        if self.timeout_sec <= self.settle_sec:
            raise ValueError(
                "Base verification timeout must exceed settling time"
            )

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(cls):
            key = f"base.goal_verification.{field.name}"
            if not node.has_parameter(key):
                node.declare_parameter(
                    key,
                    getattr(defaults, field.name),
                )
            values[field.name] = float(
                node.get_parameter(key).value
            )
        return cls(**values)


class BaseGoalVerifier:
    """Track physical settling independently from requested goal accuracy."""

    def __init__(self, target, config, started_at, motion_timeout_sec=None):
        if len(target) != 3 or not all(
            math.isfinite(value) for value in target
        ):
            raise ValueError(
                "Base goal must contain finite x, y and yaw"
            )
        self.target = target
        self.config = config
        self.started_at = started_at
        self.motion_timeout_sec = motion_timeout_sec
        self._progress_at = started_at
        self._progress_error = None
        self.anchor = None
        self.stable_since = None
        self.last_stamp = None
        self.detail = "Waiting for measured base pose"
        self.current_error = None
        self.within_tolerance = None
        self.settled = False
        self.settled_stamp = None

    @staticmethod
    def sample_is_fresh(
        pose,
        stamp,
        ros_now,
        maximum_age_sec,
    ) -> bool:
        if pose is None or stamp is None:
            return False
        values = (*pose, stamp, ros_now, maximum_age_sec)
        if not all(math.isfinite(value) for value in values):
            return False
        age = ros_now - stamp
        return 0.0 <= age <= maximum_age_sec

    @staticmethod
    def errors(first, second):
        return (
            math.hypot(
                first[0] - second[0],
                first[1] - second[1],
            ),
            abs(
                math.atan2(
                    math.sin(first[2] - second[2]),
                    math.cos(first[2] - second[2]),
                )
            ),
        )

    def update(self, pose, stamp, ros_now, now):
        c = self.config
        self.current_error = None
        if (
            self.last_stamp is not None
            and stamp is not None
            and stamp < self.last_stamp
        ):
            self._reset_settling()
            self.last_stamp = None

        fresh = self.sample_is_fresh(
            pose,
            stamp,
            ros_now,
            c.maximum_pose_age_sec,
        )
        if not fresh:
            self._reset_settling()
            self.within_tolerance = None
            self.detail = "Base pose unavailable or stale"
        else:
            position, yaw = self.errors(pose, self.target)
            self.current_error = (position, yaw)
            # Driver AT_GOAL can precede physical arrival. Give a moving
            # base time to finish, but do not extend the deadline for noise,
            # motion away from the goal, or an unavailable pose.
            if self.motion_timeout_sec is not None and not self._timed_out(now):
                previous = self._progress_error
                if previous is None:
                    self._progress_error = (position, yaw)
                elif (
                    position <= previous[0]
                    and yaw <= previous[1] + c.settle_yaw_tolerance_rad
                    and previous[0] - position > c.settle_position_tolerance_m
                ) or (
                    yaw <= previous[1]
                    and position <= previous[0] + c.settle_position_tolerance_m
                    and previous[1] - yaw > c.settle_yaw_tolerance_rad
                ):
                    self._progress_error = (position, yaw)
                    self._progress_at = now
            self.within_tolerance = (
                position <= c.position_tolerance_m
                and yaw <= c.yaw_tolerance_rad
            )
            self.detail = (
                f"Base goal error: {position:.4f} m, "
                f"{math.degrees(yaw):.2f} deg"
            )

            if self.last_stamp is None or stamp > self.last_stamp:
                drift = (
                    self.errors(pose, self.anchor)
                    if self.anchor
                    else (math.inf, math.inf)
                )
                if (
                    drift[0] > c.settle_position_tolerance_m
                    or drift[1] > c.settle_yaw_tolerance_rad
                ):
                    self.anchor = pose
                    self.stable_since = now
                    self.settled = False
                    self.settled_stamp = None
                elif (
                    self.stable_since is not None
                    and now - self.stable_since >= c.settle_sec
                ):
                    if not self.settled:
                        self.settled = True
                        self.settled_stamp = stamp
                self.last_stamp = stamp

            if self.settled:
                self.detail += "; base settled"
                if (
                    self.within_tolerance
                    and not self._timed_out(now)
                ):
                    return True

        if self._timed_out(now):
            return False
        return None

    def _timed_out(self, now):
        return (
            now - self._progress_at >= self.config.timeout_sec
            or (
                self.motion_timeout_sec is not None
                and now - self.started_at >= self.motion_timeout_sec
            )
        )

    def _reset_settling(self) -> None:
        self.stable_since = None
        self.anchor = None
        self.settled = False
        self.settled_stamp = None


__all__ = [
    "BaseGoalVerificationConfig",
    "BaseGoalVerifier",
]
