"""Frozen configuration and state for one probe-point execution."""

from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .models import InspectionObject, PoseData
from .object_repository import ObjectRepository
from .probe_execution_target import (
    ProbeExecutionTarget,
    resolve_probe_execution_target,
)
from .sensor_models import SensorDefinition
from .sensor_repository import SensorRepository


class ProbeExecutionStage(str, Enum):
    """Externally visible stages of one probe-point run."""

    LOADED = "loaded"
    WAITING_FOR_OBJECT = "waiting_for_object"
    MOVING_SAFE_APPROACH = "moving_safe_approach"
    MOVING_ALIGNED_PREAPPROACH = "moving_aligned_preapproach"
    CONVERGING_SURFACE_DISTANCE = "converging_surface_distance"
    MEASURING = "measuring"
    RETRACTING_ALIGNED_PREAPPROACH = (
        "retracting_aligned_preapproach"
    )
    RETRACTING_SAFE_APPROACH = "retracting_safe_approach"
    RECOVERING = "recovering"
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


@dataclass(frozen=True)
class ProbeExecutionConfiguration:
    """Definitions frozen when an execution command is dispatched."""

    inspection_object: InspectionObject
    sensor_definition: SensorDefinition
    routine_id: str
    probe_point_id: str

    @classmethod
    def load(
        cls,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        object_repository: ObjectRepository,
        sensor_repository: SensorRepository,
    ) -> "ProbeExecutionConfiguration":
        """Load and validate the selected definitions exactly once."""
        inspection_object = object_repository.load(object_id)
        routine = inspection_object.get_routine(routine_id)
        if routine is None:
            raise ValueError(
                "Unknown routine for probe execution: "
                f"{object_id}/{routine_id}"
            )
        if routine.get_probe_point(probe_point_id) is None:
            raise ValueError(
                "Unknown probe point for probe execution: "
                f"{object_id}/{routine_id}/{probe_point_id}"
            )
        sensor_definition = sensor_repository.load(routine.sensor_id)
        inspection_object.validate()
        sensor_definition.validate()
        return cls(
            inspection_object=deepcopy(inspection_object),
            sensor_definition=deepcopy(sensor_definition),
            routine_id=routine_id,
            probe_point_id=probe_point_id,
        )

    def resolve_target(
        self,
        object_pose_execution: PoseData,
        execution_frame: str = "odom",
    ) -> ProbeExecutionTarget:
        """Resolve the frozen geometry against the current object pose."""
        return resolve_probe_execution_target(
            self.inspection_object,
            self.routine_id,
            self.probe_point_id,
            self.sensor_definition,
            object_pose_execution,
            execution_frame,
        )


@dataclass
class ProbeExecutionSession:
    """Apply explicit normal, failure, cancellation, and recovery rules."""

    configuration: ProbeExecutionConfiguration
    stage: ProbeExecutionStage = ProbeExecutionStage.LOADED
    detail: str = ""
    stopped_stage: Optional[ProbeExecutionStage] = None
    _recovery_outcome: Optional[ProbeExecutionStage] = field(
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
        sensor_repository: SensorRepository,
    ) -> "ProbeExecutionSession":
        """Create one run with an immutable definition snapshot."""
        return cls(
            configuration=ProbeExecutionConfiguration.load(
                object_id,
                routine_id,
                probe_point_id,
                object_repository,
                sensor_repository,
            )
        )

    @property
    def is_terminal(self) -> bool:
        """Return whether execution has a final outcome."""
        return self.stage in {
            ProbeExecutionStage.SUCCEEDED,
            ProbeExecutionStage.FAILED,
            ProbeExecutionStage.CANCELLED,
        }

    @property
    def recovery_required(self) -> bool:
        """Return whether retraction must finish before the outcome."""
        return self.stage == ProbeExecutionStage.RECOVERING

    def advance(self, next_stage: ProbeExecutionStage) -> None:
        """Advance by exactly one successful execution stage."""
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

    def fail(self, reason: str, recovery_required: bool) -> None:
        """Fail immediately or enter recovery before reporting failure."""
        self._begin_terminal_transition(
            ProbeExecutionStage.FAILED,
            reason,
            recovery_required,
        )

    def cancel(self, reason: str, recovery_required: bool) -> None:
        """Cancel immediately or enter recovery before reporting it."""
        self._begin_terminal_transition(
            ProbeExecutionStage.CANCELLED,
            reason,
            recovery_required,
        )

    def complete_recovery(self) -> None:
        """Finish recovery with the outcome that initiated it."""
        if not self.recovery_required or self._recovery_outcome is None:
            raise RuntimeError("Probe execution is not recovering")
        self.stage = self._recovery_outcome
        self._recovery_outcome = None

    def recovery_failed(self, reason: str) -> None:
        """End a failed recovery as a terminal execution failure."""
        if not self.recovery_required:
            raise RuntimeError("Probe execution is not recovering")
        self.stage = ProbeExecutionStage.FAILED
        self.detail = (
            f"{self.detail}; recovery failed: {self._reason(reason)}"
        )
        self._recovery_outcome = None

    def _begin_terminal_transition(
        self,
        outcome: ProbeExecutionStage,
        reason: str,
        recovery_required: bool,
    ) -> None:
        if self.is_terminal:
            raise RuntimeError(
                f"Probe execution is already {self.stage.value}"
            )
        if self.recovery_required:
            raise RuntimeError("Probe execution is already recovering")
        self.stopped_stage = self.stage
        self.detail = self._reason(reason)
        if recovery_required:
            self._recovery_outcome = outcome
            self.stage = ProbeExecutionStage.RECOVERING
            return
        self.stage = outcome

    @staticmethod
    def _reason(reason: str) -> str:
        value = reason.strip()
        if not value:
            raise ValueError("Execution outcome requires a reason")
        return value
