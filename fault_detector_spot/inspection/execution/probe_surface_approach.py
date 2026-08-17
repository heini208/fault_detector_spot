"""Track one frozen close-range surface approach from probe kinematics."""

import math
from dataclasses import dataclass

from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    rotate_vector,
)


@dataclass(frozen=True)
class ProbeSurfaceApproachPlan:
    """Frozen fitted surface geometry for one close-range approach attempt."""

    surface_point_execution: tuple[float, float, float]
    surface_normal_execution: tuple[float, float, float]
    inward_direction_execution: tuple[float, float, float]
    starting_probe_position_execution: tuple[float, float, float]
    measured_initial_distance_m: float
    target_distance_m: float
    planned_travel_m: float
    maximum_travel_m: float

    def surface_point(self) -> Vector3Data:
        return Vector3Data(*self.surface_point_execution)

    def surface_normal(self) -> Vector3Data:
        return Vector3Data(*self.surface_normal_execution)

    def inward_direction(self) -> Vector3Data:
        return Vector3Data(*self.inward_direction_execution)


@dataclass(frozen=True)
class ProbeSurfaceApproachEvaluation:
    """Kinematic progress for one frozen surface approach."""

    estimated_distance_m: float
    remaining_inward_travel_m: float
    traveled_inward_m: float
    lateral_offset_m: float
    axis_error_rad: float
    requested_step_m: float
    reached: bool


def freeze_probe_surface_approach(
    current_probe_pose_execution: PoseData,
    measured_initial_distance_m: float,
    target_distance_m: float,
    maximum_travel_m: float,
    surface_plane_probe: SurfacePlane = None,
) -> ProbeSurfaceApproachPlan:
    """Freeze the fitted surface plane and current probe approach axis."""
    current_probe_pose_execution.validate()
    for value, label in (
        (measured_initial_distance_m, "Measured surface distance"),
        (target_distance_m, "Target surface distance"),
        (maximum_travel_m, "Maximum surface approach travel"),
    ):
        require_positive_finite_distance(value, label)

    measured_initial_distance_m = float(measured_initial_distance_m)
    target_distance_m = float(target_distance_m)
    maximum_travel_m = float(maximum_travel_m)

    if target_distance_m > measured_initial_distance_m:
        raise ValueError(
            "Target surface distance exceeds measured surface distance"
        )

    inward = _normalized(
        rotate_vector(
            current_probe_pose_execution.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        ),
        "Probe inward direction",
    )
    position = current_probe_pose_execution.position

    if surface_plane_probe is None:
        surface_point = Vector3Data(
            x=float(position.x) + inward.x * measured_initial_distance_m,
            y=float(position.y) + inward.y * measured_initial_distance_m,
            z=float(position.z) + inward.z * measured_initial_distance_m,
        )
        surface_normal = Vector3Data(
            x=-inward.x,
            y=-inward.y,
            z=-inward.z,
        )
    else:
        surface_plane_probe.validate()
        surface_point_relative = surface_plane_probe.point
        surface_normal_relative = surface_plane_probe.normal
        surface_point_rotated = rotate_vector(
            current_probe_pose_execution.orientation,
            surface_point_relative,
        )
        surface_normal = _normalized(
            rotate_vector(
                current_probe_pose_execution.orientation,
                surface_normal_relative,
            ),
            "Surface outward normal",
        )
        surface_point = Vector3Data(
            x=position.x + surface_point_rotated.x,
            y=position.y + surface_point_rotated.y,
            z=position.z + surface_point_rotated.z,
        )
        if _dot(surface_normal, inward) >= -1e-6:
            raise ValueError(
                "Fitted surface normal does not oppose probe local +X"
            )

    initial_distance = _plane_distance(
        surface_point,
        surface_normal,
        position,
    )
    if surface_plane_probe is not None:
        measured_initial_distance_m = initial_distance

    alignment = -_dot(surface_normal, inward)
    if alignment <= 1e-6:
        raise ValueError("Probe approach axis does not intersect fitted surface")
    planned_travel_m = (
        measured_initial_distance_m - target_distance_m
    ) / alignment
    if planned_travel_m > maximum_travel_m + 1e-12:
        raise ValueError(
            "Required surface approach exceeds maximum travel"
        )

    return ProbeSurfaceApproachPlan(
        surface_point_execution=(
            surface_point.x,
            surface_point.y,
            surface_point.z,
        ),
        surface_normal_execution=(
            surface_normal.x,
            surface_normal.y,
            surface_normal.z,
        ),
        inward_direction_execution=(inward.x, inward.y, inward.z),
        starting_probe_position_execution=(
            float(position.x),
            float(position.y),
            float(position.z),
        ),
        measured_initial_distance_m=measured_initial_distance_m,
        target_distance_m=target_distance_m,
        planned_travel_m=planned_travel_m,
        maximum_travel_m=maximum_travel_m,
    )


def evaluate_probe_surface_approach(
    plan: ProbeSurfaceApproachPlan,
    current_probe_pose_execution: PoseData,
    maximum_step_m: float,
    tolerance_m: float,
) -> ProbeSurfaceApproachEvaluation:
    """Estimate remaining stand-off from current probe kinematics."""
    if not isinstance(plan, ProbeSurfaceApproachPlan):
        raise TypeError("Expected a ProbeSurfaceApproachPlan")
    current_probe_pose_execution.validate()
    require_positive_finite_distance(
        maximum_step_m,
        "Maximum surface approach step",
    )
    require_positive_finite_distance(
        tolerance_m,
        "Surface approach tolerance",
    )
    maximum_step_m = float(maximum_step_m)
    tolerance_m = float(tolerance_m)

    inward = _normalized(
        Vector3Data(*plan.inward_direction_execution),
        "Frozen inward direction",
    )
    surface = Vector3Data(*plan.surface_point_execution)
    normal = _normalized(
        Vector3Data(*plan.surface_normal_execution),
        "Frozen surface normal",
    )
    start = Vector3Data(*plan.starting_probe_position_execution)
    current = current_probe_pose_execution.position

    current_inward = _normalized(
        rotate_vector(
            current_probe_pose_execution.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        ),
        "Current probe inward direction",
    )
    axis_dot = max(-1.0, min(1.0, _dot(inward, current_inward)))
    axis_error_rad = math.acos(axis_dot)

    estimated_distance_m = _plane_distance(surface, normal, current)
    if estimated_distance_m < -tolerance_m:
        raise ValueError("Probe passed the frozen surface plane")

    from_start = Vector3Data(
        x=current.x - start.x,
        y=current.y - start.y,
        z=current.z - start.z,
    )
    traveled_inward_m = _dot(from_start, inward)
    if traveled_inward_m > plan.maximum_travel_m + tolerance_m:
        raise ValueError("Probe exceeded maximum surface approach travel")

    lateral = Vector3Data(
        x=from_start.x - inward.x * traveled_inward_m,
        y=from_start.y - inward.y * traveled_inward_m,
        z=from_start.z - inward.z * traveled_inward_m,
    )
    lateral_offset_m = _norm(lateral)

    error_m = estimated_distance_m - plan.target_distance_m
    if error_m < -tolerance_m:
        raise ValueError("Probe passed the requested surface stand-off")

    reached = abs(error_m) <= tolerance_m
    if reached:
        requested_step_m = 0.0
        remaining_inward_travel_m = 0.0
    else:
        alignment = -_dot(normal, inward)
        if alignment <= 1e-6:
            raise ValueError(
                "Probe approach axis no longer intersects fitted surface"
            )
        remaining_inward_travel_m = max(0.0, error_m / alignment)
        remaining_guard_m = (
            plan.maximum_travel_m - max(0.0, traveled_inward_m)
        )
        if remaining_guard_m <= 0.0:
            raise ValueError(
                "Surface target is not reachable within maximum travel"
            )
        requested_step_m = min(
            remaining_inward_travel_m,
            maximum_step_m,
            remaining_guard_m,
        )
        if requested_step_m <= 0.0:
            raise ValueError(
                "Surface approach cannot make safe inward progress"
            )

    return ProbeSurfaceApproachEvaluation(
        estimated_distance_m=estimated_distance_m,
        remaining_inward_travel_m=remaining_inward_travel_m,
        traveled_inward_m=traveled_inward_m,
        lateral_offset_m=lateral_offset_m,
        axis_error_rad=axis_error_rad,
        requested_step_m=requested_step_m,
        reached=reached,
    )


def _plane_distance(
    surface_point: Vector3Data,
    outward_normal: Vector3Data,
    point: Vector3Data,
) -> float:
    return _dot(
        Vector3Data(
            x=point.x - surface_point.x,
            y=point.y - surface_point.y,
            z=point.z - surface_point.z,
        ),
        outward_normal,
    )


def _normalized(vector: Vector3Data, label: str) -> Vector3Data:
    vector.validate()
    magnitude = _norm(vector)
    if magnitude <= 1e-12:
        raise ValueError(f"{label} must not be zero")
    return Vector3Data(
        x=vector.x / magnitude,
        y=vector.y / magnitude,
        z=vector.z / magnitude,
    )


def _dot(left: Vector3Data, right: Vector3Data) -> float:
    return (
        left.x * right.x
        + left.y * right.y
        + left.z * right.z
    )


def _norm(vector: Vector3Data) -> float:
    return math.sqrt(_dot(vector, vector))


__all__ = [
    "ProbeSurfaceApproachEvaluation",
    "ProbeSurfaceApproachPlan",
    "evaluate_probe_surface_approach",
    "freeze_probe_surface_approach",
]
