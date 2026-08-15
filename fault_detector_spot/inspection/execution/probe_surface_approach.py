"""Track one frozen close-range surface approach from probe kinematics."""

import math
from dataclasses import dataclass

from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    rotate_vector,
)


@dataclass(frozen=True)
class ProbeSurfaceApproachPlan:
    """Frozen surface geometry for one close-range approach attempt."""

    surface_point_execution: tuple[float, float, float]
    inward_direction_execution: tuple[float, float, float]
    starting_probe_position_execution: tuple[float, float, float]
    measured_initial_distance_m: float
    target_distance_m: float
    planned_travel_m: float
    maximum_travel_m: float

    def surface_point(self) -> Vector3Data:
        """Return the frozen surface point as mutable model data."""
        return Vector3Data(*self.surface_point_execution)

    def inward_direction(self) -> Vector3Data:
        """Return the frozen inward direction as mutable model data."""
        return Vector3Data(*self.inward_direction_execution)


@dataclass(frozen=True)
class ProbeSurfaceApproachEvaluation:
    """Kinematic progress for one frozen surface approach."""

    estimated_distance_m: float
    remaining_inward_travel_m: float
    traveled_inward_m: float
    lateral_offset_m: float
    requested_step_m: float
    reached: bool


def freeze_probe_surface_approach(
    current_probe_pose_execution: PoseData,
    measured_initial_distance_m: float,
    target_distance_m: float,
    maximum_travel_m: float,
) -> ProbeSurfaceApproachPlan:
    """Freeze one measured surface and its inward approach direction."""
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

    planned_travel_m = measured_initial_distance_m - target_distance_m
    if planned_travel_m > maximum_travel_m + 1e-12:
        raise ValueError(
            "Required surface approach exceeds maximum travel"
        )

    inward = rotate_vector(
        current_probe_pose_execution.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    inward = _normalized(inward, "Probe inward direction")
    position = current_probe_pose_execution.position
    surface_point = (
        float(position.x) + inward.x * measured_initial_distance_m,
        float(position.y) + inward.y * measured_initial_distance_m,
        float(position.z) + inward.z * measured_initial_distance_m,
    )

    return ProbeSurfaceApproachPlan(
        surface_point_execution=surface_point,
        inward_direction_execution=(
            inward.x,
            inward.y,
            inward.z,
        ),
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

    inward = Vector3Data(*plan.inward_direction_execution)
    inward = _normalized(inward, "Frozen inward direction")
    surface = Vector3Data(*plan.surface_point_execution)
    start = Vector3Data(*plan.starting_probe_position_execution)
    current = current_probe_pose_execution.position

    to_surface = Vector3Data(
        x=surface.x - current.x,
        y=surface.y - current.y,
        z=surface.z - current.z,
    )
    estimated_distance_m = _dot(to_surface, inward)
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
        remaining_inward_travel_m = max(0.0, error_m)
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
        requested_step_m=requested_step_m,
        reached=reached,
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
