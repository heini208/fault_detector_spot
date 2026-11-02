"""Validate persisted scalar values without silently changing their meaning."""

import math


def require_integer(value, label):
    """Return an integer unchanged, or raise ValueError (including for bool)."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    return value


def require_number(value, label):
    """Return a finite number unchanged, or raise ValueError."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{label} contains a non-finite value")
    return value


def require_text(value, label):
    """Return nonempty text unchanged, or raise ValueError."""
    if not isinstance(value, str):
        raise ValueError(f"{label} must be text")
    if not value.strip():
        raise ValueError(f"{label} must not be empty")
    return value
