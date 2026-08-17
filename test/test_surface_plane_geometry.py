"""Tests for shared Open3D surface-plane geometry."""

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.surface_plane import (
    fit_surface_plane,
)
from fault_detector_spot.inspection.model.models import Vector3Data


def test_surface_plane_fits_flat_points_and_orients_toward_target():
    ys = np.linspace(-0.05, 0.05, 8)
    zs = np.linspace(-0.05, 0.05, 8)
    points = np.array(
        [[0.20, y, z] for y in ys for z in zs],
        dtype=float,
    )

    plane = fit_surface_plane(
        points,
        "probe",
        distance_threshold_m=0.002,
        minimum_inliers=20,
        minimum_inlier_ratio=0.8,
        ransac_iterations=100,
        minimum_tangent_spread_m=0.001,
    ).oriented_toward(Vector3Data(x=0.0, y=0.0, z=0.0))

    assert plane.inlier_count == len(points)
    assert plane.inlier_ratio == pytest.approx(1.0)
    assert plane.point.x == pytest.approx(0.20, abs=1e-6)
    assert plane.normal.x == pytest.approx(-1.0, abs=1e-6)
    assert plane.signed_distance(Vector3Data(x=0.0, y=0.0, z=0.0)) == (
        pytest.approx(0.20, abs=1e-6)
    )
