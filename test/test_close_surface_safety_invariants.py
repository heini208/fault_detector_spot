"""Safety invariants for the executor-backed close-surface approach."""

import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MoveCloseToSurfaceConfig,
    MoveCloseToSurfaceExecution,
)


class FakeExecutor:
    active = False
    speed_policy = SimpleNamespace(
        default_speed=SimpleNamespace(angular_speed_rad_s=0.5)
    )

    def cancel(self):
        pass


def execution(**kwargs):
    return MoveCloseToSurfaceExecution(
        FakeExecutor(),
        object(),
        MoveCloseToSurfaceConfig(**kwargs),
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


def test_default_travel_allows_extra_steps_for_contact_mode():
    action = execution()

    assert action.config.maximum_step_m == pytest.approx(0.010)
    assert action.config.maximum_approach_steps == 60
    assert action.config.maximum_travel_m == pytest.approx(0.400)


def test_default_surface_sampling_uses_five_frames_over_one_second():
    action = execution()

    assert action.config.minimum_surface_samples == 5
    assert action.config.minimum_surface_span_sec == pytest.approx(1.0)


def test_force_threshold_becomes_more_sensitive_near_target():
    action = execution(
        force_contact_threshold_n=5.0,
        force_near_target_threshold_n=3.0,
        force_near_target_distance_m=0.020,
    )

    assert action._force_threshold_for(0.030) == pytest.approx(5.0)
    assert action._force_threshold_for(0.020) == pytest.approx(5.0)
    assert action._force_threshold_for(0.010) == pytest.approx(4.0)
    assert action._force_threshold_for(0.000) == pytest.approx(3.0)


def test_approach_speed_decreases_toward_expected_surface():
    action = execution(
        approach_far_speed_mps=0.005,
        approach_near_speed_mps=0.001,
        approach_slowdown_distance_m=0.050,
    )

    far = action._approach_speed_for(0.050).linear_speed_mps
    middle = action._approach_speed_for(0.025).linear_speed_mps
    near = action._approach_speed_for(0.0).linear_speed_mps

    assert far == pytest.approx(0.005)
    assert near == pytest.approx(0.001)
    assert near < middle < far


def test_configuration_rejects_near_speed_above_far_speed():
    with pytest.raises(ValueError, match="Near-surface approach speed"):
        execution(
            approach_far_speed_mps=0.003,
            approach_near_speed_mps=0.004,
        )


def test_configuration_rejects_step_larger_than_ten_millimeters():
    with pytest.raises(ValueError, match="0.010 m"):
        execution(maximum_step_m=0.0101)


@pytest.mark.parametrize("margin_deg", [0.0, -1.0, 10.0, 11.0])
def test_configuration_requires_positive_alignment_margin_below_axis_limit(margin_deg):
    with pytest.raises(ValueError):
        execution(approach_alignment_margin_rad=math.radians(margin_deg))


def test_custom_axis_limit_requires_compatible_alignment_margin():
    with pytest.raises(ValueError, match="margin must be smaller"):
        execution(maximum_approach_axis_error_rad=math.radians(0.5))
    action = execution(
        maximum_approach_axis_error_rad=math.radians(0.5),
        approach_alignment_margin_rad=math.radians(0.1),
    )
    assert action.config.approach_alignment_margin_rad == pytest.approx(math.radians(0.1))


def test_approach_allowances_preserve_observation_and_alignment_quality_limits():
    config = execution().config

    assert config.maximum_approach_axis_error_rad == pytest.approx(math.radians(10.0))
    assert config.maximum_axis_error_rad == pytest.approx(math.radians(5.0))
    assert config.approach_body_position_tolerance_m == pytest.approx(0.020)
    assert config.reference_position_tolerance_m == pytest.approx(0.010)
    assert config.reference_orientation_tolerance_rad == pytest.approx(math.radians(2.0))


@pytest.mark.parametrize("name", [
    "maximum_approach_axis_error_rad", "approach_body_position_tolerance_m",
])
@pytest.mark.parametrize("value", [0.0, -1.0, math.nan, math.inf])
def test_approach_allowances_require_positive_finite_values(name, value):
    with pytest.raises(ValueError, match="positive"):
        execution(**{name: value})


@pytest.mark.parametrize("angle_deg", [90.0, 120.0])
def test_approach_alignment_limit_must_face_toward_the_surface(angle_deg):
    with pytest.raises(ValueError, match="90"):
        execution(maximum_approach_axis_error_rad=math.radians(angle_deg))


def test_configuration_rejects_contact_search_beyond_one_step():
    with pytest.raises(ValueError, match="Contact search overtravel"):
        execution(contact_search_overtravel_m=0.011)


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
