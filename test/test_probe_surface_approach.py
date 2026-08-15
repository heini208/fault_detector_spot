"""Tests for frozen kinematic probe surface approach tracking."""

import math

import pytest

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
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


def test_freeze_uses_measured_distance():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(x=1.0, y=2.0),
        measured_initial_distance_m=0.30,
        target_distance_m=0.05,
        maximum_travel_m=0.27,
    )

    assert plan.surface_point_execution == pytest.approx((1.30, 2.0, 0.0))
    assert plan.inward_direction_execution == pytest.approx((1.0, 0.0, 0.0))
    assert plan.planned_travel_m == pytest.approx(0.25)


def test_evaluation_uses_current_tf_without_new_depth():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        measured_initial_distance_m=0.30,
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


def test_evaluation_reports_lateral_and_axis_drift():
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        measured_initial_distance_m=0.30,
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
        measured_initial_distance_m=0.30,
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


def test_freeze_rejects_touch_target():
    with pytest.raises(
        ValueError,
        match="Target surface distance must be positive",
    ):
        freeze_probe_surface_approach(
            current_probe_pose_execution=_pose(),
            measured_initial_distance_m=0.30,
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
            measured_initial_distance_m=0.30,
            target_distance_m=0.05,
            maximum_travel_m=0.20,
        )
