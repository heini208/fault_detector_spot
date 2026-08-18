"""Safety invariants for the standalone close-surface approach."""

import math
import time

import pytest

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.execution.move_close_to_surface_operation import (
    MoveCloseToSurfaceOperation,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


def operation(**kwargs):
    return MoveCloseToSurfaceOperation(
        node=object(),
        state_source=object(),
        robot_command_client=object(),
        **kwargs,
    )


def pose(orientation=None):
    return PoseData(
        position=Vector3Data(x=0.0, y=0.0, z=0.0),
        orientation=orientation or QuaternionData.identity(),
    )


def plane(normal=None):
    return SurfacePlane(
        point=Vector3Data(x=0.10, y=0.0, z=0.0),
        normal=normal or Vector3Data(x=-1.0, y=0.0, z=0.0),
        frame_id="probe",
        inlier_count=50,
        sample_count=50,
        inlier_ratio=1.0,
        rmse_m=0.001,
    )


def test_default_travel_matches_40_ten_millimeter_steps():
    action = operation()

    assert action.maximum_step_m == pytest.approx(0.010)
    assert action.maximum_approach_steps == 40
    assert action.maximum_travel_m == pytest.approx(0.400)


def test_default_surface_sampling_uses_five_frames_over_one_second():
    action = operation()

    assert action.minimum_surface_samples == 5
    assert action.minimum_surface_span_sec == pytest.approx(1.0)


def test_default_force_stale_timeout_tolerates_normal_motion_gaps():
    action = operation()

    assert action.force_stale_timeout_sec == pytest.approx(1.5)


def test_force_guard_uses_configured_stale_timeout_for_sample_age():
    class StateSource:
        maximum_age_sec = None

        def latest_end_effector_force(self, maximum_age_sec=None):
            self.maximum_age_sec = maximum_age_sec
            raise ValueError("No force sample")

    action = operation(
        force_stale_timeout_sec=1.25,
    )
    action._state_source = StateSource()
    action._force_last_receipt = time.monotonic()

    assert action._check_force_guard() is None
    assert action._state_source.maximum_age_sec == pytest.approx(1.25)


def test_force_guard_becomes_more_sensitive_near_target():
    action = operation(
        force_contact_threshold_n=5.0,
        force_near_target_threshold_n=3.0,
        force_near_target_distance_m=0.020,
    )

    assert action._force_threshold_for(0.030) == pytest.approx(5.0)
    assert action._force_threshold_for(0.020) == pytest.approx(5.0)
    assert action._force_threshold_for(0.010) == pytest.approx(4.0)
    assert action._force_threshold_for(0.000) == pytest.approx(3.0)


def test_default_force_baseline_allows_normal_stationary_sensor_noise():
    action = operation()

    assert action.force_baseline_max_component_span_n == pytest.approx(3.0)


def test_configuration_rejects_near_target_threshold_above_normal_threshold():
    with pytest.raises(ValueError, match="Near-target force threshold"):
        operation(
            force_contact_threshold_n=4.0,
            force_near_target_threshold_n=4.1,
        )


def test_configuration_rejects_step_larger_than_ten_millimeters():
    with pytest.raises(ValueError, match="0.010 m"):
        operation(maximum_step_m=0.0101)


def test_configuration_rejects_travel_unreachable_with_step_count():
    with pytest.raises(ValueError, match="step-count limit"):
        operation(
            maximum_step_m=0.005,
            maximum_approach_steps=40,
            maximum_travel_m=0.201,
        )


def test_frozen_plan_records_surface_normal_axis_error():
    tilted_normal = Vector3Data(
        x=-math.cos(math.radians(4.0)),
        y=math.sin(math.radians(4.0)),
        z=0.0,
    )

    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=pose(),
        surface_plane_probe=plane(tilted_normal),
        target_distance_m=0.05,
        maximum_travel_m=0.40,
    )

    assert plan.initial_axis_error_rad == pytest.approx(
        math.radians(4.0)
    )


def test_runtime_axis_error_is_measured_against_surface_normal():
    tilted_normal = Vector3Data(
        x=-math.cos(math.radians(4.0)),
        y=math.sin(math.radians(4.0)),
        z=0.0,
    )
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=pose(),
        surface_plane_probe=plane(tilted_normal),
        target_distance_m=0.05,
        maximum_travel_m=0.40,
    )
    current = pose(
        quaternion_from_euler("z", math.radians(-2.0))
    )

    evaluation = evaluate_probe_surface_approach(
        plan,
        current_probe_pose_execution=current,
        maximum_step_m=0.010,
        tolerance_m=0.005,
    )

    assert evaluation.axis_error_rad == pytest.approx(
        math.radians(2.0)
    )


def test_action_rejects_initial_surface_misalignment_before_force_baseline():
    action = operation(
        maximum_axis_error_rad=math.radians(5.0)
    )
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=pose(),
        surface_plane_probe=plane(
            Vector3Data(
                x=-math.cos(math.radians(6.0)),
                y=math.sin(math.radians(6.0)),
                z=0.0,
            )
        ),
        target_distance_m=0.05,
        maximum_travel_m=0.40,
    )

    assert plan.initial_axis_error_rad > action.maximum_axis_error_rad
