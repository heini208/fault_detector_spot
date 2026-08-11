"""Coordinate synchronous probe authoring and persistence."""

from copy import deepcopy
from functools import wraps
from threading import RLock

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.setup.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    ProbePoint,
    ReferenceTag,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupDraft,
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeSetupGeometry,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
)
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


def _serialized_transaction(method):
    @wraps(method)
    def wrapped(self, context, *args, **kwargs):
        with self._context_lock(context):
            return method(self, context, *args, **kwargs)

    return wrapped


class ProbeSetupCoordinator:
    """Own probe drafts, synchronous calculations, and repository writes."""

    def __init__(
        self,
        setup_coordinator: SetupCoordinator,
        reference_repository,
        sensor_repository: SensorRepository,
        geometry=None,
    ):
        self.setup_coordinator = setup_coordinator
        self.reference_repository = reference_repository
        self.object_repository = reference_repository.object_repository
        self.sensor_repository = sensor_repository
        self.geometry = geometry or ProbeSetupGeometry(reference_repository)
        self._lock = RLock()
        self._drafts = {}
        self._context_locks = {}

    def open_context(self, client_id: str) -> ProbeSetupSnapshot:
        """Open one independent server-owned probe setup draft."""
        context = self.setup_coordinator.open_context(
            CommandOrigin.PROBE_SETUP,
            client_id,
        )
        with self._lock:
            self._drafts[context.context_id] = ProbeSetupDraft(
                context=context
            )
            self._context_locks[context.context_id] = RLock()
        return self.snapshot(context)

    def context(
        self,
        context_id: str,
        client_id: str,
    ) -> SetupContextSnapshot:
        """Resolve one current context and enforce client ownership."""
        with self._lock:
            draft = self._drafts.get(context_id.strip())
        if draft is None:
            raise LookupError(f"Unknown probe context: {context_id}")
        self.setup_coordinator.require_current(draft.context)
        if draft.context.client_id != client_id.strip():
            raise ValueError("Client ID does not own the probe context")
        return draft.context

    @_serialized_transaction
    def close_context(self, context: SetupContextSnapshot) -> None:
        """Close one probe draft and invalidate delayed callers."""
        draft = self._draft(context)
        with self._lock:
            self._drafts.pop(draft.context.context_id, None)
            self._context_locks.pop(draft.context.context_id, None)
        if self.setup_coordinator.is_current(draft.context):
            self.setup_coordinator.close_context(draft.context)

    @_serialized_transaction
    def snapshot(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Build an immutable snapshot from draft and repository state."""
        draft = self._draft(context)
        object_ids = tuple(self.object_repository.list_object_ids())
        routine_ids, view_ids, camera_ids, probe_ids = (
            self._selected_definition_lists(draft, object_ids)
        )
        return ProbeSetupSnapshot.from_draft(
            draft=draft,
            object_ids=object_ids,
            routine_ids=routine_ids,
            reference_view_ids=view_ids,
            reference_camera_ids=camera_ids,
            sensor_ids=self.sensor_repository.list_sensor_ids(),
            probe_point_ids=probe_ids,
        )

    @_serialized_transaction
    def refresh(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Refresh repository-derived lists without changing a draft."""
        self._draft(context)
        return self.snapshot(context)

    @_serialized_transaction
    def select_routine(
        self,
        context: SetupContextSnapshot,
        object_id: str,
        routine_id: str,
    ) -> ProbeSetupSnapshot:
        """Select one existing object and routine and reset dependencies."""
        draft = self._draft(context)
        object_name = self._name(object_id, "object ID")
        routine_name = self._name(routine_id, "routine ID")
        definition = self.object_repository.load(object_name)
        routine = definition.get_routine(routine_name)
        if routine is None:
            raise LookupError(
                f"Unknown inspection routine: {object_name}/{routine_name}"
            )
        with self._lock:
            draft.selected_object_id = object_name
            draft.selected_routine_id = routine_name
            draft.selected_reference_view_id = ""
            draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def create_object(
        self,
        context: SetupContextSnapshot,
        object_id: str,
        display_name: str,
        reference_tag_id: int,
        reference_tag_family: str,
    ) -> ProbeSetupSnapshot:
        """Create one map-independent inspection object."""
        draft = self._draft(context)
        if (
            isinstance(reference_tag_id, bool)
            or not isinstance(reference_tag_id, int)
        ):
            raise TypeError("Reference tag ID must be an integer")
        definition = InspectionObject(
            object_id=self._name(object_id, "object ID"),
            display_name=self._text(display_name, "object display name"),
            reference_tag=ReferenceTag(
                tag_id=reference_tag_id,
                tag_family=self._text(
                    reference_tag_family,
                    "reference tag family",
                ),
            ),
        )
        self.object_repository.create(definition)
        with self._lock:
            draft.selected_object_id = definition.object_id
            draft.selected_routine_id = ""
            draft.selected_reference_view_id = ""
            draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def delete_object(
        self,
        context: SetupContextSnapshot,
        object_id: str,
    ) -> ProbeSetupSnapshot:
        """Delete one object and every artifact owned by it."""
        draft = self._draft(context)
        object_name = self._name(object_id, "object ID")
        if not self.object_repository.delete_object(object_name):
            raise FileNotFoundError(
                f"Unknown inspection object: {object_name}"
            )
        with self._lock:
            if draft.selected_object_id == object_name:
                draft.clear_selection()
        return self._advance(draft)

    @_serialized_transaction
    def create_routine(
        self,
        context: SetupContextSnapshot,
        object_id: str,
        routine_id: str,
        display_name: str,
        sensor_id: str,
    ) -> ProbeSetupSnapshot:
        """Create one sensor-specific inspection routine."""
        draft = self._draft(context)
        object_name = self._name(object_id, "object ID")
        sensor_name = self._name(sensor_id, "sensor ID")
        self.sensor_repository.load(sensor_name)
        routine = InspectionRoutine(
            routine_id=self._name(routine_id, "routine ID"),
            display_name=self._text(display_name, "routine display name"),
            sensor_id=sensor_name,
        )
        self.object_repository.add_routine(object_name, routine)
        with self._lock:
            draft.selected_object_id = object_name
            draft.selected_routine_id = routine.routine_id
            draft.selected_reference_view_id = ""
            draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def delete_routine(
        self,
        context: SetupContextSnapshot,
        object_id: str,
        routine_id: str,
    ) -> ProbeSetupSnapshot:
        """Delete one routine and its reference datasets."""
        draft = self._draft(context)
        object_name = self._name(object_id, "object ID")
        routine_name = self._name(routine_id, "routine ID")
        self.object_repository.delete_routine(object_name, routine_name)
        with self._lock:
            if (
                draft.selected_object_id == object_name
                and draft.selected_routine_id == routine_name
            ):
                draft.selected_routine_id = ""
                draft.selected_reference_view_id = ""
                draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def select_reference_pixel(
        self,
        context: SetupContextSnapshot,
        reference_view_id: str,
        pixel: ImagePoint,
        approach_mode: str,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
    ) -> ProbeSetupSnapshot:
        """Calculate a complete draft from one original-image pixel."""
        draft = self._selected_draft(context)
        pixel.validate()
        view_id = self._text(reference_view_id, "reference view ID")
        geometry = self._resolve_geometry(
            draft,
            view_id,
            pixel,
            approach_mode,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        with self._lock:
            draft.selected_reference_view_id = view_id
            draft.reference_pixel = deepcopy(pixel)
            draft.geometry = geometry
            draft.setup = deepcopy(geometry.probe_setup)
            draft.dirty = True
            draft.validation_error = ""
        return self._advance(draft)

    @_serialized_transaction
    def clear_reference_pixel(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Clear the selected pixel and every geometry dependency."""
        draft = self._selected_draft(context)
        with self._lock:
            draft.selected_reference_view_id = ""
            draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def update_geometry(
        self,
        context: SetupContextSnapshot,
        approach_mode: str,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
    ) -> ProbeSetupSnapshot:
        """Recalculate distances while retaining safe and aligned approval."""
        draft = self._selected_draft(context)
        if draft.reference_pixel is None:
            raise ValueError("No reference pixel is selected")
        previous = deepcopy(draft.setup)
        geometry = self._resolve_geometry(
            draft,
            draft.selected_reference_view_id,
            draft.reference_pixel,
            approach_mode,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        setup = self._retained_distance_approvals(
            geometry.probe_setup,
            previous,
        )
        with self._lock:
            draft.geometry = geometry
            draft.setup = setup
            draft.dirty = True
            draft.validation_error = ""
        return self._advance(draft)

    @_serialized_transaction
    def approve_safe_pose(
        self,
        context: SetupContextSnapshot,
        pose_object,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved obstacle-safe pose."""
        return self._approve(
            context,
            approve_safe_approach_pose,
            pose_object,
        )

    @_serialized_transaction
    def approve_aligned_pose(
        self,
        context: SetupContextSnapshot,
        pose_object,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved surface-aligned pre-approach pose."""
        return self._approve(
            context,
            approve_surface_alignment_pose,
            pose_object,
        )

    @_serialized_transaction
    def approve_probe_pose(
        self,
        context: SetupContextSnapshot,
        pose_object,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved final probe pose."""
        return self._approve(context, approve_probe_pose, pose_object)

    @_serialized_transaction
    def save_probe_point(
        self,
        context: SetupContextSnapshot,
        probe_point_id: str,
        display_name: str,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        measurement_duration_sec: float,
    ) -> ProbeSetupSnapshot:
        """Atomically append one fully approved probe point."""
        draft = self._selected_draft(context)
        setup = draft.setup
        if setup is None:
            raise ValueError("No calculated probe setup is available")
        if not all((
            setup.safe_approach_approved,
            setup.surface_alignment_approved,
            setup.probe_pose_approved,
        )):
            raise ValueError("All three probe poses must be approved")
        point = self._build_probe_point(
            draft,
            probe_point_id,
            display_name,
            position_tolerance_m,
            orientation_tolerance_rad,
            measurement_duration_sec,
        )
        self.object_repository.add_probe_point(
            draft.selected_object_id,
            draft.selected_routine_id,
            point,
        )
        with self._lock:
            draft.dirty = False
            draft.validation_error = ""
        return self._advance(draft)

    def _build_probe_point(
        self,
        draft,
        probe_point_id,
        display_name,
        position_tolerance_m,
        orientation_tolerance_rad,
        measurement_duration_sec,
    ):
        setup = draft.setup
        point = ProbePoint(
            probe_point_id=self._name(probe_point_id, "probe point ID"),
            display_name=self._text(
                display_name,
                "probe point display name",
            ),
            safe_approach_pose_object=deepcopy(
                setup.safe_approach_pose_object
            ),
            probe_pose_object=deepcopy(setup.probe_pose_object),
            target_surface_distance_m=(
                setup.surface_target.target_surface_distance_m
            ),
            position_tolerance_m=float(position_tolerance_m),
            orientation_tolerance_rad=float(orientation_tolerance_rad),
            measurement_duration_sec=float(measurement_duration_sec),
            aligned_preapproach_distance_m=(
                setup.surface_target.aligned_preapproach_distance_m
            ),
            reference_pixel=deepcopy(draft.reference_pixel),
            reference_view_id=draft.selected_reference_view_id,
        )
        point.validate()
        return point

    def close(self) -> None:
        """Close every probe context owned by this coordinator."""
        with self._lock:
            contexts = tuple(draft.context for draft in self._drafts.values())
        for context in contexts:
            if self.setup_coordinator.is_current(context):
                self.close_context(context)

    def _approve(self, context, operation, pose_object):
        draft = self._selected_draft(context)
        if draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        pose_object.validate()
        setup = operation(draft.setup, deepcopy(pose_object))
        with self._lock:
            draft.setup = setup
            draft.dirty = True
            draft.validation_error = ""
        return self._advance(draft)

    def _selected_definition_lists(self, draft, object_ids):
        if draft.selected_object_id not in object_ids:
            return (), (), (), ()
        definition = self.object_repository.load(draft.selected_object_id)
        routine_ids = tuple(
            routine.routine_id for routine in definition.routines
        )
        routine = definition.get_routine(draft.selected_routine_id)
        if routine is None:
            return routine_ids, (), (), ()
        return (
            routine_ids,
            tuple(view.view_id for view in routine.reference_views),
            tuple(view.camera_id for view in routine.reference_views),
            tuple(point.probe_point_id for point in routine.probe_points),
        )

    @staticmethod
    def _retained_distance_approvals(setup, previous):
        setup = deepcopy(setup)
        if previous is None:
            return setup
        if previous.safe_approach_approved:
            setup = approve_safe_approach_pose(
                setup,
                previous.safe_approach_pose_object,
            )
        if previous.surface_alignment_approved:
            setup = approve_surface_alignment_pose(
                setup,
                previous.aligned_preapproach_pose_object,
            )
        return setup

    def _selected_draft(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupDraft:
        draft = self._draft(context)
        if not draft.selected_object_id or not draft.selected_routine_id:
            raise ValueError("No inspection object and routine are selected")
        definition = self.object_repository.load(draft.selected_object_id)
        if definition.get_routine(draft.selected_routine_id) is None:
            raise LookupError("Selected inspection routine no longer exists")
        return draft

    def _resolve_geometry(
        self,
        draft,
        reference_view_id,
        pixel,
        approach_mode,
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    ):
        definition = self.object_repository.load(draft.selected_object_id)
        routine = definition.get_routine(draft.selected_routine_id)
        sensor = self.sensor_repository.load(routine.sensor_id)
        return self.geometry.resolve(
            object_id=draft.selected_object_id,
            routine_id=draft.selected_routine_id,
            reference_view_id=reference_view_id,
            pixel=deepcopy(pixel),
            approach_mode=approach_mode,
            target_surface_distance_m=target_surface_distance_m,
            aligned_preapproach_distance_m=(
                aligned_preapproach_distance_m
            ),
            hand_to_probe_pose=sensor.hand_to_probe,
        )

    def _draft(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupDraft:
        self.setup_coordinator.require_current(context)
        with self._lock:
            draft = self._drafts.get(context.context_id)
        if draft is None or draft.context != context:
            raise LookupError("Probe setup context is stale")
        return draft

    def _context_lock(self, context: SetupContextSnapshot) -> RLock:
        if not isinstance(context, SetupContextSnapshot):
            raise TypeError("Expected a SetupContextSnapshot")
        with self._lock:
            lock = self._context_locks.get(context.context_id)
        if lock is None:
            raise LookupError("Probe setup context is closed")
        return lock

    def _advance(self, draft: ProbeSetupDraft) -> ProbeSetupSnapshot:
        updated = self.setup_coordinator.advance_context(draft.context)
        with self._lock:
            draft.context = updated
        return self.snapshot(updated)

    @staticmethod
    def _name(value: str, label: str) -> str:
        return validate_storage_name(value.strip(), label)

    @staticmethod
    def _text(value: str, label: str) -> str:
        if not isinstance(value, str):
            raise TypeError(f"{label.title()} must be text")
        normalized = value.strip()
        if not normalized:
            raise ValueError(f"{label.title()} must not be empty")
        if normalized != value:
            raise ValueError(
                f"{label.title()} must not contain surrounding whitespace"
            )
        return normalized


__all__ = ["ProbeSetupCoordinator"]
