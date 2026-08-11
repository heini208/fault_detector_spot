"""Internal probe draft and immutable public snapshots."""

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Optional, Tuple

from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeGeometryResult,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    ProbeRefinementSession,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    ProbeSurfaceVerificationSession,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
)


@dataclass
class ProbeSetupDraft:
    """Hold mutable server-owned state for one probe setup context."""

    context: SetupContextSnapshot
    selected_object_id: str = ""
    selected_routine_id: str = ""
    selected_reference_view_id: str = ""
    reference_pixel: Optional[ImagePoint] = None
    geometry: Optional[ProbeGeometryResult] = None
    setup: Optional[ReferenceProbeSetup] = None
    refinement: Optional[ProbeRefinementSession] = None
    surface_verification: Optional[ProbeSurfaceVerificationSession] = None
    dirty: bool = False
    validation_error: str = ""

    def clear_selection(self) -> None:
        self.selected_object_id = ""
        self.selected_routine_id = ""
        self.selected_reference_view_id = ""
        self.reference_pixel = None
        self.geometry = None
        self.setup = None
        self.refinement = None
        self.surface_verification = None
        self.dirty = False
        self.validation_error = ""

    def clear_geometry(self) -> None:
        self.reference_pixel = None
        self.geometry = None
        self.setup = None
        self.refinement = None
        self.surface_verification = None
        self.dirty = False
        self.validation_error = ""


@dataclass(frozen=True)
class ProbeSetupSnapshot:
    """Expose one isolated immutable probe authoring snapshot."""

    context: SetupContextSnapshot
    selected_object_id: str
    selected_routine_id: str
    selected_reference_view_id: str
    selected_reference_tag_id: int
    selected_reference_tag_family: str
    selected_sensor_id: str
    object_ids: Tuple[str, ...]
    routine_ids: Tuple[str, ...]
    reference_view_ids: Tuple[str, ...]
    reference_camera_ids: Tuple[str, ...]
    sensor_ids: Tuple[str, ...]
    probe_point_ids: Tuple[str, ...]
    reference_pixel: Optional[ImagePoint]
    geometry: Optional[ProbeGeometryResult]
    setup: Optional[ReferenceProbeSetup]
    refinement: Optional[ProbeRefinementSession]
    dirty: bool
    validation_error: str
    surface_verification: Optional[ProbeSurfaceVerificationSession] = None

    @classmethod
    def from_draft(
        cls,
        draft: ProbeSetupDraft,
        object_ids,
        routine_ids,
        reference_view_ids,
        reference_camera_ids,
        selected_reference_tag_id,
        selected_reference_tag_family,
        selected_sensor_id,
        sensor_ids,
        probe_point_ids,
    ) -> "ProbeSetupSnapshot":
        return cls(
            context=draft.context,
            selected_object_id=draft.selected_object_id,
            selected_routine_id=draft.selected_routine_id,
            selected_reference_view_id=(
                draft.selected_reference_view_id
            ),
            selected_reference_tag_id=selected_reference_tag_id,
            selected_reference_tag_family=selected_reference_tag_family,
            selected_sensor_id=selected_sensor_id,
            object_ids=tuple(object_ids),
            routine_ids=tuple(routine_ids),
            reference_view_ids=tuple(reference_view_ids),
            reference_camera_ids=tuple(reference_camera_ids),
            sensor_ids=tuple(sensor_ids),
            probe_point_ids=tuple(probe_point_ids),
            reference_pixel=deepcopy(draft.reference_pixel),
            geometry=deepcopy(draft.geometry),
            setup=deepcopy(draft.setup),
            refinement=deepcopy(draft.refinement),
            surface_verification=deepcopy(
                draft.surface_verification
            ),
            dirty=bool(draft.dirty),
            validation_error=draft.validation_error,
        )


__all__ = ["ProbeSetupDraft", "ProbeSetupSnapshot"]
