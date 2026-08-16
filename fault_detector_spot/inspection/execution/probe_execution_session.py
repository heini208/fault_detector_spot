"""Immutable configuration and ordered recovery for probe execution."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    MotionAttachmentSnapshot,
)
from fault_detector_spot.inspection.repository.object_repository import (
    ObjectRepository,
)
from .probe_execution_target import (
    ProbeExecutionTarget,
    resolve_probe_execution_geometry,
)


class ProbeExecutionStage(str, Enum):
    """Externally visible stages of one probe-point run."""

    LOADED = "loaded"
    WAITING_FOR_OBJECT = "waiting_for_object"
    MOVING_SAFE_APPROACH = "moving_safe_approach"
    MOVING_ALIGNED_PREAPPROACH = "moving_aligned_preapproach"
    CONVERGING_SURFACE_DISTANCE = "converging_surface_distance"
    MEASURING = "measuring"
    RETRACTING_ALIGNED_PREAPPROACH = "retracting_aligned_preapproach"
    RETRACTING_SAFE_APPROACH = "retracting_safe_approach"
    RECOVERING_ALIGNED_PREAPPROACH = (
        "recovering_aligned_preapproach"
    )
    RECOVERING_SAFE_APPROACH = "recovering_safe_approach"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


_NORMAL_TRANSITIONS = {
    ProbeExecutionStage.LOADED: ProbeExecutionStage.WAITING_FOR_OBJECT,
    ProbeExecutionStage.WAITING_FOR_OBJECT: (
        ProbeExecutionStage.MOVING_SAFE_APPROACH
    ),
    ProbeExecutionStage.MOVING_SAFE_APPROACH: (
        ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH
    ),
    ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH: (
        ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE
    ),
    ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE: (
        ProbeExecutionStage.MEASURING
    ),
    ProbeExecutionStage.MEASURING: (
        ProbeExecutionStage.RETRACTING_ALIGNED_PREAPPROACH
    ),
    ProbeExecutionStage.RETRACTING_ALIGNED_PREAPPROACH: (
        ProbeExecutionStage.RETRACTING_SAFE_APPROACH
    ),
    ProbeExecutionStage.RETRACTING_SAFE_APPROACH: (
        ProbeExecutionStage.SUCCEEDED
    ),
}

_RECOVERY_ENTRY = {
    ProbeExecutionStage.MOVING_SAFE_APPROACH: (
        ProbeExecutionStage.RECOVERING_SAFE_APPROACH
    ),
    ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH: (
        ProbeExecutionStage.RECOVERING_SAFE_APPROACH
    ),
    ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE: (
        ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
    ),
    ProbeExecutionStage.MEASURING: (
        ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
    ),
    ProbeExecutionStage.RETRACTING_ALIGNED_PREAPPROACH: (
        ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
    ),
    ProbeExecutionStage.RETRACTING_SAFE_APPROACH: (
        ProbeExecutionStage.RECOVERING_SAFE_APPROACH
    ),
}


@dataclass(frozen=True)
class FrozenPoseData:
    """Tuple-backed pose that cannot be mutated through nested fields."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]

    @classmethod
    def from_pose(cls, pose: PoseData) -> "FrozenPoseData":
        pose.validate()
        return cls(
            position=(
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ),
            orientation=(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

    def to_pose(self) -> PoseData:
        return PoseData(
            position=Vector3Data(*self.position),
            orientation=QuaternionData(*self.orientation),
        )


@dataclass(frozen=True)
class ProbeExecutionConfiguration:
    """Minimal immutable definition snapshot loaded for one action goal."""

    object_id: str
    routine_id: str
    probe_point_id: str
    reference_tag_id: int
    reference_tag_family: str
    sensor_id: str
    attachment_revision: int
    safe_approach_pose_object: FrozenPoseData
    probe_pose_object: FrozenPoseData
    hand_to_probe: FrozenPoseData
    target_surface_distance_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    measurement_duration_sec: float
    aligned_preapproach_distance_m: float
    sensor_path: Optional[str]

    @classmethod
    def load(
        cls,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        object_repository: ObjectRepository,
        attachment: MotionAttachmentSnapshot,
    ) -> "ProbeExecutionConfiguration":
        """Freeze selected point geometry with the active sensor snapshot."""
        if not isinstance(attachment, MotionAttachmentSnapshot):
            raise TypeError("Expected a motion attachment snapshot")
        inspection_object = object_repository.load(object_id)
        inspection_object.validate()
        routine = inspection_object.get_routine(routine_id)
        if routine is None:
            raise ValueError(
                "Unknown routine for probe execution: "
                f"{object_id}/{routine_id}"
            )
        probe_point = routine.get_probe_point(probe_point_id)
        if probe_point is None:
            raise ValueError(
                "Unknown probe point for probe execution: "
                f"{object_id}/{routine_id}/{probe_point_id}"
            )
        return cls(
            object_id=inspection_object.object_id,
            routine_id=routine.routine_id,
            probe_point_id=probe_point.probe_point_id,
            reference_tag_id=inspection_object.reference_tag.tag_id,
            reference_tag_family=inspection_object.reference_tag.tag_family,
            sensor_id=attachment.sensor_id,
            attachment_revision=attachment.attachment_revision,
            safe_approach_pose_object=FrozenPoseData.from_pose(
                probe_point.safe_approach_pose_object
            ),
            probe_pose_object=FrozenPoseData.from_pose(
                probe_point.probe_pose_object
            ),
            hand_to_probe=FrozenPoseData.from_pose(
                attachment.hand_to_probe()
            ),
            target_surface_distance_m=float(
                probe_point.target_surface_distance_m
            ),
            position_tolerance_m=float(
                probe_point.position_tolerance_m
            ),
            orientation_tolerance_rad=float(
                probe_point.orientation_tolerance_rad
            ),
            measurement_duration_sec=float(
                probe_point.measurement_duration_sec
            ),
            aligned_preapproach_distance_m=float(
                probe_point.aligned_preapproach_distance_m
            ),
            sensor_path=probe_point.sensor_path,
        )

    def resolve_target(
        self,
        object_pose_execution: PoseData,
        execution_frame: str = "odom",
    ) -> ProbeExecutionTarget:
        """Resolve frozen geometry against the current live object pose."""
        return resolve_probe_execution_geometry(
            object_id=self.object_id,
            routine_id=self.routine_id,
            probe_point_id=self.probe_point_id,
            sensor_id=self.sensor_id,
            attachment_revision=self.attachment_revision,
            safe_approach_pose_object=(
                self.safe_approach_pose_object.to_pose()
            ),
            probe_pose_object=self.probe_pose_object.to_pose(),
            hand_to_probe=self.hand_to_probe.to_pose(),
            target_surface_distance_m=self.target_surface_distance_m,
            position_tolerance_m=self.position_tolerance_m,
            orientation_tolerance_rad=self.orientation_tolerance_rad,
            measurement_duration_sec=self.measurement_duration_sec,
            aligned_preapproach_distance_m=(
                self.aligned_preapproach_distance_m
            ),
            sensor_path=self.sensor_path,
            object_pose_execution=object_pose_execution,
            execution_frame=execution_frame,
        )


@dataclass
class ProbeExecutionSession:
    """Enforce normal execution and stage-derived recovery order."""

    configuration: ProbeExecutionConfiguration
    stage: ProbeExecutionStage = ProbeExecutionStage.LOADED
    detail: str = ""
    stopped_stage: Optional[ProbeExecutionStage] = None
    _requested_outcome: Optional[ProbeExecutionStage] = field(
        default=None,
        init=False,
        repr=False,
    )

    @classmethod
    def load(
        cls,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        object_repository: ObjectRepository,
        attachment: MotionAttachmentSnapshot,
    ) -> "ProbeExecutionSession":
        return cls(
            configuration=ProbeExecutionConfiguration.load(
                object_id,
                routine_id,
                probe_point_id,
                object_repository,
                attachment,
            )
        )

    @property
    def is_terminal(self) -> bool:
        return self.stage in {
            ProbeExecutionStage.SUCCEEDED,
            ProbeExecutionStage.FAILED,
            ProbeExecutionStage.CANCELLED,
        }

    @property
    def recovery_required(self) -> bool:
        return self.stage in {
            ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH,
            ProbeExecutionStage.RECOVERING_SAFE_APPROACH,
        }

    @property
    def requested_outcome(self) -> Optional[ProbeExecutionStage]:
        return self._requested_outcome

    def advance(self, next_stage: ProbeExecutionStage) -> None:
        if self.is_terminal or self.recovery_required:
            raise RuntimeError(
                f"Cannot advance probe execution from {self.stage.value}"
            )
        expected = _NORMAL_TRANSITIONS.get(self.stage)
        if next_stage != expected:
            expected_text = expected.value if expected else "none"
            raise RuntimeError(
                f"Expected stage {expected_text}, got {next_stage.value}"
            )
        self.stage = next_stage
        self.detail = ""

    def fail(self, reason: str) -> None:
        self._begin_terminal_transition(
            ProbeExecutionStage.FAILED,
            reason,
        )

    def cancel(self, reason: str) -> None:
        self._begin_terminal_transition(
            ProbeExecutionStage.CANCELLED,
            reason,
        )

    def complete_recovery_stage(self) -> None:
        if self.stage == (
            ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
        ):
            self.stage = ProbeExecutionStage.RECOVERING_SAFE_APPROACH
            return
        if self.stage == ProbeExecutionStage.RECOVERING_SAFE_APPROACH:
            if self._requested_outcome is None:
                raise RuntimeError("Probe execution has no recovery outcome")
            self.stage = self._requested_outcome
            return
        raise RuntimeError("Probe execution is not recovering")

    def recovery_failed(self, reason: str) -> None:
        if not self.recovery_required:
            raise RuntimeError("Probe execution is not recovering")
        self.stage = ProbeExecutionStage.FAILED
        self.detail = (
            f"{self.detail}; recovery failed: {self._reason(reason)}"
        )

    def _begin_terminal_transition(
        self,
        outcome: ProbeExecutionStage,
        reason: str,
    ) -> None:
        if self.is_terminal:
            raise RuntimeError(
                f"Probe execution is already {self.stage.value}"
            )
        if self.recovery_required:
            raise RuntimeError("Probe execution is already recovering")
        self.stopped_stage = self.stage
        self.detail = self._reason(reason)
        self._requested_outcome = outcome
        recovery_stage = _RECOVERY_ENTRY.get(self.stage)
        self.stage = recovery_stage or outcome

    @staticmethod
    def _reason(reason: str) -> str:
        value = reason.strip()
        if not value:
            raise ValueError("Execution outcome requires a reason")
        return value
