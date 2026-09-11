"""Tests for direction-aware force projection."""

import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.directional_force import (
    directional_force_delta,
)


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return SimpleNamespace(x=x, y=y, z=z, w=w)


def test_opposing_force_is_projected_onto_arbitrary_diagonal_direction():
    component = 4.0 / math.sqrt(2.0)

    result = directional_force_delta(
        baseline_force_hand=(0.0, 0.0, 0.0),
        current_force_hand=(-component, -component, 0.0),
        baseline_hand_orientation=quaternion(),
        current_hand_orientation=quaternion(),
        movement_direction=(1.0, 1.0, 0.0),
    )

    assert result.opposing_n == pytest.approx(4.0)
    assert result.total_n == pytest.approx(4.0)


def test_large_sideways_force_has_zero_opposing_component():
    component = 8.0 / math.sqrt(2.0)

    result = directional_force_delta(
        baseline_force_hand=(0.0, 0.0, 0.0),
        current_force_hand=(component, -component, 0.0),
        baseline_hand_orientation=quaternion(),
        current_hand_orientation=quaternion(),
        movement_direction=(1.0, 1.0, 0.0),
    )

    assert result.opposing_n == pytest.approx(0.0, abs=1e-9)
    assert result.total_n == pytest.approx(8.0)


def test_force_in_same_direction_as_travel_does_not_count_as_opposing():
    result = directional_force_delta(
        baseline_force_hand=(0.0, 0.0, 0.0),
        current_force_hand=(7.0, 0.0, 0.0),
        baseline_hand_orientation=quaternion(),
        current_hand_orientation=quaternion(),
        movement_direction=(1.0, 0.0, 0.0),
    )

    assert result.opposing_n == pytest.approx(0.0)
    assert result.total_n == pytest.approx(7.0)


def test_hand_rotation_does_not_create_false_force_delta():
    half = math.sqrt(0.5)

    result = directional_force_delta(
        baseline_force_hand=(1.0, 0.0, 0.0),
        current_force_hand=(0.0, -1.0, 0.0),
        baseline_hand_orientation=quaternion(),
        current_hand_orientation=quaternion(
            z=half,
            w=half,
        ),
        movement_direction=(1.0, 0.0, 0.0),
    )

    assert result.opposing_n == pytest.approx(0.0, abs=1e-9)
    assert result.total_n == pytest.approx(0.0, abs=1e-9)
