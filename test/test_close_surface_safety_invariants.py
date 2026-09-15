"""Safety invariants for the executor-backed close-surface approach."""

import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MoveCloseToSurfaceBehaviour,
    MoveCloseToSurfaceConfig,
)


class FakeExecutor:
    active = False
    speed_policy = SimpleNamespace(
        default_speed=SimpleNamespace(angular_speed_rad_s=0.5)
    )

    def cancel(self):
        pass


def behaviour(**kwargs):
    action = MoveCloseToSurfaceBehaviour(
        surface_source=object(),
        config=MoveCloseToSurfaceConfig(**kwargs),
    )
    action.executor = FakeExecutor()
    return action


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
    action = behaviour()

    assert action.config.maximum_step_m == pytest.approx(0.010)
    assert action.config.maximum_approach_steps == 60
    assert action.config.maximum_travel_m == pytest.approx(0.400)


def test_default_surface_sampling_uses_five_frames_over_one_second():
    action = behaviour()

    assert action.config.minimum_surface_samples == 5
    assert action.config.minimum_surface_span_sec == pytest.approx(1.0)


def test_force_threshold_becomes_more_sensitive_near_target():
    action = behaviour(
        force_contact_threshold_n=5.0,
        force_near_target_threshold_n=3.0,
        force_near_target_distance_m=0.020,
    )

    assert action._force_threshold_for(0.030) == pytest.approx(5.0)
    assert action._force_threshold_for(0.020) == pytest.approx(5.0)
    assert action._force_threshold_for(0.010) == pytest.approx(4.0)
    assert action._force_threshold_for(0.000) == pytest.approx(3.0)


def test_approach_speed_decreases_toward_expected_surface():
    action = behaviour(
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
        behaviour(
            approach_far_speed_mps=0.003,
            approach_near_speed_mps=0.004,
        )


def test_configuration_rejects_step_larger_than_ten_millimeters():
    with pytest.raises(ValueError, match="0.010 m"):
        behaviour(maximum_step_m=0.0101)


def test_configuration_rejects_contact_search_beyond_one_step():
    with pytest.raises(ValueError, match="Contact search overtravel"):
        behaviour(contact_search_overtravel_m=0.011)


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
