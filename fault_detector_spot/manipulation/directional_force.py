"""Directional force projection for guarded Cartesian arm motion."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DirectionalForceDelta:
    """Baseline-subtracted force metrics in one common execution frame."""

    opposing_n: float
    total_n: float
    delta_x_n: float
    delta_y_n: float
    delta_z_n: float


def directional_force_delta(
    baseline_force_hand,
    current_force_hand,
    baseline_hand_orientation,
    current_hand_orientation,
    movement_direction,
) -> DirectionalForceDelta:
    """Project force change onto the direction opposing planned travel."""
    direction = _normalized_vector(
        movement_direction,
        "Movement direction",
    )
    baseline_execution = _rotate_local_to_parent(
        baseline_hand_orientation,
        baseline_force_hand,
    )
    current_execution = _rotate_local_to_parent(
        current_hand_orientation,
        current_force_hand,
    )

    delta = tuple(
        current - baseline
        for current, baseline in zip(
            current_execution,
            baseline_execution,
        )
    )
    total = math.sqrt(sum(value * value for value in delta))
    opposing = max(
        0.0,
        -sum(
            force_component * direction_component
            for force_component, direction_component
            in zip(delta, direction)
        ),
    )
    return DirectionalForceDelta(
        opposing_n=float(opposing),
        total_n=float(total),
        delta_x_n=float(delta[0]),
        delta_y_n=float(delta[1]),
        delta_z_n=float(delta[2]),
    )


def _rotate_local_to_parent(quaternion, vector):
    x, y, z, w = _normalized_quaternion(quaternion)
    vx, vy, vz = _finite_vector(vector, "Force vector")

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return (
        r00 * vx + r01 * vy + r02 * vz,
        r10 * vx + r11 * vy + r12 * vz,
        r20 * vx + r21 * vy + r22 * vz,
    )


def _normalized_quaternion(quaternion):
    values = tuple(
        float(value)
        for value in (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Hand orientation contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("Hand orientation quaternion is invalid")
    return tuple(value / norm for value in values)


def _normalized_vector(vector, label):
    values = _finite_vector(vector, label)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError(f"{label} must be non-zero")
    return tuple(value / norm for value in values)


def _finite_vector(vector, label):
    values = tuple(float(value) for value in vector)
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains a non-finite value")
    return values


__all__ = [
    "DirectionalForceDelta",
    "directional_force_delta",
]
