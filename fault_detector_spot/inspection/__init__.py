"""Inspection authoring, execution, sensing, and persistence."""

from fault_detector_spot.inspection.execution.live_object_pose_resolver import (
    LiveObjectPoseResolver,
)
from fault_detector_spot.inspection.ros.live_object_state_adapter import (
    live_object_pose_to_msg,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    PoseData,
    ProbePoint,
    QuaternionData,
    ReferenceTag,
    ReferenceView,
    Vector3Data,
    Waypoint,
)
from fault_detector_spot.inspection.repository.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.execution.probe_execution_target import (
    ProbeExecutionTarget,
    resolve_probe_execution_geometry,
    resolve_probe_execution_target,
)
from fault_detector_spot.inspection.execution.probe_execution_session import (
    FrozenPoseData,
    ProbeExecutionConfiguration,
    ProbeExecutionSession,
    ProbeExecutionStage,
)
from fault_detector_spot.inspection.model.resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SENSOR_PARENT_FRAME,
    SensorDefinition,
    sensor_definition_from_values,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    matrix_to_pose,
    pose_data_to_pose,
    pose_to_matrix,
    pose_to_pose_data,
    relative_pose,
)


def __getattr__(name):
    if name == "MapRepository":
        from fault_detector_spot.mapping.repository.map_repository import (
            MapRepository,
        )

        return MapRepository
    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r}"
    )

__all__ = [
    "ImagePoint",
    "InspectionObject",
    "InspectionRoutine",
    "LiveObjectPoseResolver",
    "LocalizationLandmark",
    "MapDefinition",
    "MapRepository",
    "ObjectApproach",
    "ObjectPoseState",
    "ObjectRepository",
    "PoseData",
    "FrozenPoseData",
    "ProbeExecutionTarget",
    "ProbeExecutionConfiguration",
    "ProbeExecutionSession",
    "ProbeExecutionStage",
    "ProbePoint",
    "QuaternionData",
    "ReferenceTag",
    "ReferenceView",
    "ResolvedObjectPose",
    "SENSOR_PARENT_FRAME",
    "SensorDefinition",
    "SensorRepository",
    "Vector3Data",
    "Waypoint",
    "compose_poses",
    "inverse_pose",
    "live_object_pose_to_msg",
    "matrix_to_pose",
    "pose_data_to_pose",
    "pose_to_matrix",
    "pose_to_pose_data",
    "relative_pose",
    "resolve_probe_execution_target",
    "resolve_probe_execution_geometry",
    "sensor_definition_from_values",
    "sensor_probe_frame",
]
