"""Resolve saved probe geometry into one local execution frame."""

from dataclasses import dataclass

from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    compose_poses,
    derive_aligned_preapproach_pose,
    probe_pose_to_hand_pose,
    rotate_vector,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    MotionAttachmentSnapshot,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
    validate_surface_distance_pair,
)


@dataclass(frozen=True)
class ProbeExecutionTarget:
    """Validated probe and hand targets for one saved point."""

    object_id: str
    routine_id: str
    probe_point_id: str
    sensor_id: str
    attachment_revision: int
    probe_frame: str
    execution_frame: str
    safe_approach_probe_pose_execution: PoseData
    safe_approach_hand_pose_execution: PoseData
    aligned_preapproach_probe_pose_execution: PoseData
    aligned_preapproach_hand_pose_execution: PoseData
    nominal_probe_pose_execution: PoseData
    nominal_hand_pose_execution: PoseData
    inward_direction_execution: Vector3Data
    target_surface_distance_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    measurement_duration_sec: float
    aligned_preapproach_distance_m: float
    sensor_path: str | None


def resolve_probe_execution_target(
    inspection_object: InspectionObject,
    routine_id: str,
    probe_point_id: str,
    attachment: MotionAttachmentSnapshot,
    object_pose_execution: PoseData,
    execution_frame: str = "odom",
) -> ProbeExecutionTarget:
    """Compose one saved probe point with active sensor or bare-hand geometry."""
    if not isinstance(attachment, MotionAttachmentSnapshot):
        raise TypeError("Expected a motion attachment snapshot")
    inspection_object.validate()
    object_pose_execution.validate()

    if not execution_frame or execution_frame != execution_frame.strip():
        raise ValueError("Execution frame must not be empty or padded")

    routine = inspection_object.get_routine(routine_id)
    if routine is None:
        raise ValueError(
            f"Unknown routine {routine_id} for object "
            f"{inspection_object.object_id}"
        )

    probe_point = routine.get_probe_point(probe_point_id)
    if probe_point is None:
        raise ValueError(
            f"Unknown probe point {probe_point_id} for routine "
            f"{routine_id}"
        )

    return resolve_probe_execution_geometry(
        object_id=inspection_object.object_id,
        routine_id=routine.routine_id,
        probe_point_id=probe_point.probe_point_id,
        sensor_id=attachment.sensor_id,
        attachment_revision=attachment.attachment_revision,
        safe_approach_pose_object=probe_point.safe_approach_pose_object,
        probe_pose_object=probe_point.probe_pose_object,
        hand_to_probe=attachment.hand_to_probe(),
        target_surface_distance_m=probe_point.target_surface_distance_m,
        position_tolerance_m=probe_point.position_tolerance_m,
        orientation_tolerance_rad=probe_point.orientation_tolerance_rad,
        measurement_duration_sec=probe_point.measurement_duration_sec,
        aligned_preapproach_distance_m=(
            probe_point.aligned_preapproach_distance_m
        ),
        sensor_path=probe_point.sensor_path,
        object_pose_execution=object_pose_execution,
        execution_frame=execution_frame,
    )


def resolve_probe_execution_geometry(
    object_id: str,
    routine_id: str,
    probe_point_id: str,
    sensor_id: str,
    safe_approach_pose_object: PoseData,
    probe_pose_object: PoseData,
    hand_to_probe: PoseData,
    target_surface_distance_m: float,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
    measurement_duration_sec: float,
    aligned_preapproach_distance_m: float,
    sensor_path: str | None,
    object_pose_execution: PoseData,
    execution_frame: str = "odom",
    attachment_revision: int = 0,
) -> ProbeExecutionTarget:
    """Resolve one validated immutable geometry snapshot."""
    for value, label in (
        (object_id, "Object ID"),
        (routine_id, "Routine ID"),
        (probe_point_id, "Probe point ID"),
        (execution_frame, "Execution frame"),
    ):
        if not value or value != value.strip():
            raise ValueError(f"{label} must not be empty or padded")
    if sensor_id != sensor_id.strip():
        raise ValueError("Sensor ID must not be padded")
    if (
        isinstance(attachment_revision, bool)
        or not isinstance(attachment_revision, int)
        or attachment_revision < 0
    ):
        raise ValueError("Attachment revision must be a non-negative integer")
    safe_approach_pose_object.validate()
    probe_pose_object.validate()
    hand_to_probe.validate()
    object_pose_execution.validate()
    validate_surface_distance_pair(
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    )
    for value, label in (
        (position_tolerance_m, "Position tolerance"),
        (orientation_tolerance_rad, "Orientation tolerance"),
        (measurement_duration_sec, "Measurement duration"),
    ):
        require_positive_finite_distance(value, label)

    safe_probe_pose = compose_poses(
        object_pose_execution,
        safe_approach_pose_object,
    )
    aligned_probe_pose_object = derive_aligned_preapproach_pose(
        probe_pose_object,
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    )
    aligned_probe_pose = compose_poses(
        object_pose_execution,
        aligned_probe_pose_object,
    )
    nominal_probe_pose = compose_poses(
        object_pose_execution,
        probe_pose_object,
    )
    safe_hand_pose = probe_pose_to_hand_pose(
        safe_probe_pose,
        hand_to_probe,
    )
    aligned_hand_pose = probe_pose_to_hand_pose(
        aligned_probe_pose,
        hand_to_probe,
    )
    nominal_hand_pose = probe_pose_to_hand_pose(
        nominal_probe_pose,
        hand_to_probe,
    )
    inward_direction = rotate_vector(
        nominal_probe_pose.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    motion_sensor_id = sensor_id or BARE_HAND_MOTION_ID

    return ProbeExecutionTarget(
        object_id=object_id,
        routine_id=routine_id,
        probe_point_id=probe_point_id,
        sensor_id=sensor_id,
        attachment_revision=attachment_revision,
        probe_frame=sensor_probe_frame(motion_sensor_id),
        execution_frame=execution_frame,
        safe_approach_probe_pose_execution=safe_probe_pose,
        safe_approach_hand_pose_execution=safe_hand_pose,
        aligned_preapproach_probe_pose_execution=aligned_probe_pose,
        aligned_preapproach_hand_pose_execution=aligned_hand_pose,
        nominal_probe_pose_execution=nominal_probe_pose,
        nominal_hand_pose_execution=nominal_hand_pose,
        inward_direction_execution=inward_direction,
        target_surface_distance_m=target_surface_distance_m,
        position_tolerance_m=position_tolerance_m,
        orientation_tolerance_rad=orientation_tolerance_rad,
        measurement_duration_sec=measurement_duration_sec,
        aligned_preapproach_distance_m=aligned_preapproach_distance_m,
        sensor_path=sensor_path,
    )
