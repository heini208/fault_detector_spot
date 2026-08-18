"""Tests for frozen fitted-plane probe surface approach tracking."""

import math

import pytest

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


def _pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def _yaw(degrees):
    half = math.radians(degrees) * 0.5
    return QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half),
    )


def _plane(distance=0.30, normal=None):
    return SurfacePlane(
        point=Vector3Data(x=distance, y=0.0, z=0.0),
        normal=normal or Vector3Data(x=-1.0, y=0.0, z=0.0),
        frame_id="probe",
        inlier_count=30,
        sample_count=32,
        inlier_ratio=30 / 32,
        rmse_m=0.001,
    )


def test_freeze_uses_fitted_surface_plane():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(x=1.0, y=2.0),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    assert plan.surface_point_execution == pytest.approx((1.30, 2.0, 0.0))
    assert plan.surface_normal_execution == pytest.approx((-1.0, 0.0, 0.0))
    assert plan.inward_direction_execution == pytest.approx((1.0, 0.0, 0.0))
    assert plan.measured_initial_distance_m == pytest.approx(0.30)
    assert plan.planned_travel_m == pytest.approx(0.25)


def test_evaluation_uses_frozen_plane_without_new_depth():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(x=0.10),
        maximum_step_m=0.01,
        tolerance_m=0.005,
    )

    assert evaluation.estimated_distance_m == pytest.approx(0.20)
    assert evaluation.remaining_inward_travel_m == pytest.approx(0.15)
    assert evaluation.requested_step_m == pytest.approx(0.01)
    assert evaluation.axis_error_rad == pytest.approx(0.0)
    assert not evaluation.reached


def test_evaluation_halves_remaining_travel_near_target():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(x=0.236),
        maximum_step_m=0.01,
        tolerance_m=0.005,
    )

    assert evaluation.estimated_distance_m == pytest.approx(0.064)
    assert evaluation.remaining_inward_travel_m == pytest.approx(0.014)
    assert evaluation.requested_step_m == pytest.approx(0.007)
    assert not evaluation.reached


def test_evaluation_uses_minimum_normal_step_outside_tolerance():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(x=0.244),
        maximum_step_m=0.01,
        minimum_step_m=0.004,
        tolerance_m=0.001,
    )

    assert evaluation.remaining_inward_travel_m == pytest.approx(0.006)
    assert evaluation.requested_step_m == pytest.approx(0.004)
    assert not evaluation.reached


def test_evaluation_allows_final_step_below_minimum():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(x=0.2485),
        maximum_step_m=0.01,
        minimum_step_m=0.004,
        tolerance_m=0.001,
    )

    assert evaluation.remaining_inward_travel_m == pytest.approx(0.0015)
    assert evaluation.requested_step_m == pytest.approx(0.0015)
    assert not evaluation.reached


def test_tilted_plane_changes_required_axis_travel():
    normal = Vector3Data(
        x=-math.cos(math.radians(30.0)),
        y=math.sin(math.radians(30.0)),
        z=0.0,
    )
    plane = SurfacePlane(
        point=Vector3Data(x=0.30, y=0.0, z=0.0),
        normal=normal,
        frame_id="probe",
        inlier_count=30,
        sample_count=30,
        inlier_ratio=1.0,
        rmse_m=0.001,
    )
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=plane,
        target_distance_m=0.05,
        maximum_travel_m=0.40,
    )

    expected_distance = 0.30 * math.cos(math.radians(30.0))
    expected_travel = (
        expected_distance - 0.05
    ) / math.cos(math.radians(30.0))
    assert plan.measured_initial_distance_m == pytest.approx(expected_distance)
    assert plan.planned_travel_m == pytest.approx(expected_travel)


def test_evaluation_reports_lateral_and_axis_drift():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(
            x=0.10,
            y=0.03,
            orientation=_yaw(4.0),
        ),
        maximum_step_m=0.01,
        tolerance_m=0.005,
    )

    assert evaluation.lateral_offset_m == pytest.approx(0.03)
    assert evaluation.axis_error_rad == pytest.approx(math.radians(4.0))


def test_evaluation_stops_inside_requested_tolerance():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        surface_plane_probe=_plane(),
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=_pose(x=0.248),
        maximum_step_m=0.01,
        tolerance_m=0.005,
    )

    assert evaluation.estimated_distance_m == pytest.approx(0.052)
    assert evaluation.requested_step_m == pytest.approx(0.0)
    assert evaluation.reached


def test_freeze_requires_fitted_plane():
    with pytest.raises(TypeError, match="fitted SurfacePlane"):
        freeze_probe_surface_approach(
            current_probe_pose_execution=_pose(),
            surface_plane_probe=None,
            target_distance_m=0.05,
            maximum_travel_m=0.30,
        )


def test_freeze_rejects_touch_target():
    with pytest.raises(
        ValueError,
        match="Target surface distance must be positive",
    ):
        freeze_probe_surface_approach(
            current_probe_pose_execution=_pose(),
            surface_plane_probe=_plane(),
            target_distance_m=0.0,
            maximum_travel_m=0.30,
        )


def test_freeze_rejects_required_travel_beyond_limit():
    with pytest.raises(
        ValueError,
        match="exceeds maximum travel",
    ):
        freeze_probe_surface_approach(
            current_probe_pose_execution=_pose(),
            surface_plane_probe=_plane(),
            target_distance_m=0.05,
            maximum_travel_m=0.20,
        )
