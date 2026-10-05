"""Internal probe draft and immutable public snapshots."""

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Optional, Tuple

from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.inspection.model.models import ImagePoint, PreApproachPathPoint
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeGeometryResult,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    ProbeRefinementSession,
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
    pre_approach_path: list[PreApproachPathPoint] = field(default_factory=list)
    aligned_position_tolerance_m: Optional[float] = None
    dirty: bool = False
    validation_error: str = ""

    def clear_selection(self) -> None:
        self.selected_object_id = ""
        self.selected_routine_id = ""
        self.selected_reference_view_id = ""
        self.reference_pixel = None
        self.pre_approach_path.clear()
        self.aligned_position_tolerance_m = None
        self.geometry = None
        self.setup = None
        self.refinement = None
        self.dirty = False
        self.validation_error = ""

    def clear_geometry(self) -> None:
        self.reference_pixel = None
        self.pre_approach_path.clear()
        self.aligned_position_tolerance_m = None
        self.geometry = None
        self.setup = None
        self.refinement = None
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
    object_ids: Tuple[str, ...]
    routine_ids: Tuple[str, ...]
    reference_view_ids: Tuple[str, ...]
    reference_camera_ids: Tuple[str, ...]
    probe_point_ids: Tuple[str, ...]
    reference_pixel: Optional[ImagePoint]
    geometry: Optional[ProbeGeometryResult]
    setup: Optional[ReferenceProbeSetup]
    refinement: Optional[ProbeRefinementSession]
    dirty: bool
    validation_error: str
    probe_point_target_surface_distances_m: Tuple[float, ...] = ()
    pre_approach_path: Tuple[PreApproachPathPoint, ...] = ()
    has_base_position: bool = False
    has_routine_safe_approach_pose: bool = False
    routine_safe_position_tolerance_m: float = .1

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
        probe_point_ids,
        probe_point_target_surface_distances_m=(),
        has_base_position=False,
        has_routine_safe_approach_pose=False,
        routine_safe_position_tolerance_m=.1,
    ) -> "ProbeSetupSnapshot":
        return cls(
            pre_approach_path=tuple(deepcopy(draft.pre_approach_path)),
            context=draft.context,
            selected_object_id=draft.selected_object_id,
            selected_routine_id=draft.selected_routine_id,
            selected_reference_view_id=(
                draft.selected_reference_view_id
            ),
            selected_reference_tag_id=selected_reference_tag_id,
            selected_reference_tag_family=selected_reference_tag_family,
            object_ids=tuple(object_ids),
            routine_ids=tuple(routine_ids),
            reference_view_ids=tuple(reference_view_ids),
            reference_camera_ids=tuple(reference_camera_ids),
            probe_point_ids=tuple(probe_point_ids),
            probe_point_target_surface_distances_m=tuple(probe_point_target_surface_distances_m),
            routine_safe_position_tolerance_m=routine_safe_position_tolerance_m,
            has_base_position=bool(has_base_position),
            has_routine_safe_approach_pose=bool(has_routine_safe_approach_pose),
            reference_pixel=deepcopy(draft.reference_pixel),
            geometry=deepcopy(draft.geometry),
            setup=deepcopy(draft.setup),
            refinement=deepcopy(draft.refinement),
            dirty=bool(draft.dirty),
            validation_error=draft.validation_error,
        )


__all__ = ["ProbeSetupDraft", "ProbeSetupSnapshot"]
