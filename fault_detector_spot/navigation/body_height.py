"""Bounds for a stationary body-height offset from nominal standing height."""

import math

MIN_BODY_HEIGHT_M = -0.20
MAX_BODY_HEIGHT_M = 0.20


def validate_body_height(value: float) -> float:
    height = float(value)
    if (
        not math.isfinite(height)
        or not MIN_BODY_HEIGHT_M <= height <= MAX_BODY_HEIGHT_M
    ):
        raise ValueError("Body height must be finite and between -0.20 and +0.20 m")
    return height
