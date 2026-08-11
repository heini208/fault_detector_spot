"""Authoritative surface-distance contracts for setup and execution."""

import math


MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M = 0.05


def require_positive_finite_distance(value: float, label: str) -> float:
    """Return one validated positive finite distance."""
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return float(value)


def validate_surface_distance_pair(
    target_surface_distance_m: float,
    aligned_preapproach_distance_m: float,
) -> float:
    """Validate absolute probe and aligned distances and return separation."""
    target = require_positive_finite_distance(
        target_surface_distance_m,
        "Target surface distance",
    )
    aligned = require_positive_finite_distance(
        aligned_preapproach_distance_m,
        "Aligned pre-approach distance",
    )
    separation = aligned - target
    if (
        separation < MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M
        and not math.isclose(
            separation,
            MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise ValueError(
            "Aligned pre-approach distance must be at least 0.05 m "
            "greater than the target surface distance"
        )
    return separation
