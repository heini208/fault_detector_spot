"""Tests for explicit tag and live-surface alignment orientations."""

import math

import pytest

from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.alignment_orientation import (
    surface_aligned_probe_orientation,
    tag_aligned_probe_orientation,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    rotate_vector,
)


def roll_quaternion(degrees):
    half = math.radians(degrees) * 0.5
    return QuaternionData(
        x=math.sin(half),
        y=0.0,
        z=0.0,
        w=math.cos(half),
    )


def test_tag_alignment_matches_relative_to_tag_probe_orientation():
    probe = tag_aligned_probe_orientation(roll_quaternion(90.0))

    assert probe.x == pytest.approx(0.0)
    assert probe.y == pytest.approx(math.sin(math.pi / 4.0))
    assert probe.z == pytest.approx(0.0)
    assert probe.w == pytest.approx(math.cos(math.pi / 4.0))


def test_surface_alignment_preserves_tilted_surface_facing_axis():
    outward = Vector3Data(x=0.4, y=-0.2, z=0.8944271909999159)
    gravity_up = Vector3Data(x=0.0, y=0.0, z=1.0)

    probe = surface_aligned_probe_orientation(
        outward,
        QuaternionData.identity(),
        gravity_up,
    )
    axis = rotate_vector(
        probe,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    norm = math.sqrt(outward.x ** 2 + outward.y ** 2 + outward.z ** 2)

    assert axis.x == pytest.approx(-outward.x / norm, abs=1e-9)
    assert axis.y == pytest.approx(-outward.y / norm, abs=1e-9)
    assert axis.z == pytest.approx(-outward.z / norm, abs=1e-9)


def test_surface_alignment_uses_gravity_to_choose_upright_roll():
    outward = Vector3Data(x=1.0, y=0.2, z=0.1)
    gravity_up = Vector3Data(x=0.0, y=0.0, z=1.0)

    probe = surface_aligned_probe_orientation(
        outward,
        QuaternionData.identity(),
        gravity_up,
    )
    hand_up = rotate_vector(
        probe,
        Vector3Data(x=0.0, y=0.0, z=1.0),
    )

    assert hand_up.z > 0.9
