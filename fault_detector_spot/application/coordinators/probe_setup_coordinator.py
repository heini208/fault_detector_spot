"""Coordinate probe authoring and single-step setup movement."""

from copy import deepcopy
from dataclasses import dataclass
from functools import wraps
from threading import RLock

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
    SetupOperation,
    SetupOperationStatus,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.application.setup.setup_context_access import (
    SetupContextAccess,
)
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.application.coordinators.probe_finalization_controller import (
    ProbeFinalizationController,
)
from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.application.coordinators.probe_surface_verification_controller import (
    ProbeSurfaceVerificationController,
)
from fault_detector_spot.inspection.setup.probe_definition_service import (
    ProbeDefinitionService,
)
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupDraft,
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_geometry_editor import (
    ProbeGeometryEditor,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeSetupGeometry,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionRequest,
    ProbeSetupMotionCommandFactory,
)


def _serialized_transaction(method):
    @wraps(method)
    def wrapped(self, context, *args, **kwargs):
        with self._context_lock(context):
            if method.__name__ not in {"snapshot", "close_context"}:
                self._require_idle(context)
            return method(self, context, *args, **kwargs)

    return wrapped


@dataclass(frozen=True)
class ProbeSetupMotionOperation:
    """Bind one setup motion primitive to its shared command request."""

    operation: SetupOperation
    motion: ProbeMotionRequest

    @property
    def request_id(self) -> str:
        return self.operation.request_id


@dataclass(frozen=True)
class ProbeSetupMotionStatus:
    """Expose one correlated probe setup motion transition."""

    request_id: str
    motion: ProbeMotionRequest
    state: CommandControllerState
    detail: str
    snapshot: ProbeSetupSnapshot


class ProbeSetupCoordinator:
    """Own probe drafts, synchronous calculations, and repository writes."""

    def __init__(
        self,
        setup_coordinator: SetupCoordinator,
        reference_repository,
        sensor_repository: SensorRepository,
        geometry=None,
        motion_state_source=None,
        motion_command_factory=None,
    ):
        self.setup_coordinator = setup_coordinator
        self.reference_repository = reference_repository
        self.object_repository = reference_repository.object_repository
        self.sensor_repository = sensor_repository
        self.definition_service = ProbeDefinitionService(
            self.object_repository,
            sensor_repository,
        )
        self.geometry = geometry or ProbeSetupGeometry(reference_repository)
        self.geometry_editor = ProbeGeometryEditor(
            self.object_repository,
            sensor_repository,
            self.geometry,
        )
        self.motion_state_source = motion_state_source
        self.motion_command_factory = (
            motion_command_factory or ProbeSetupMotionCommandFactory()
        )
        self._lock = RLock()
        self._context_access = SetupContextAccess(setup_coordinator)
        self.refinement_controller = ProbeRefinementController(
            setup_coordinator=setup_coordinator,
            object_repository=self.object_repository,
            sensor_repository=sensor_repository,
            motion_state_source=motion_state_source,
            motion_command_factory=self.motion_command_factory,
            state_lock=self._lock,
        )
        self.surface_controller = ProbeSurfaceVerificationController(
            self.refinement_controller,
            self._lock,
        )
        self.finalization_controller = ProbeFinalizationController(
            self.object_repository,
            self.refinement_controller,
            self._lock,
        )
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
        self.setup_coordinator.add_operation_listener(
            context,
            self._handle_operation_status,
        )
        return self.snapshot(context)

    def context(
        self,
        context_id: str,
        client_id: str,
    ) -> SetupContextSnapshot:
        """Resolve one current context and enforce client ownership."""
        try:
            context = self._context_access.resolve(
                context_id,
                client_id,
                CommandOrigin.PROBE_SETUP,
            )
        except LookupError as exception:
            raise LookupError(
                f"Unknown probe context: {context_id}"
            ) from exception
        with self._lock:
            draft = self._drafts.get(context.context_id)
        if draft is None or draft.context != context:
            raise LookupError(f"Unknown probe context: {context_id}")
        return context

    @_serialized_transaction
    def close_context(self, context: SetupContextSnapshot) -> None:
        """Close one probe draft and invalidate delayed callers."""
        draft = self._draft(context)
        request_ids = (
            self.refinement_controller.request_ids_for(context)
        )
        for request_id in request_ids:
            try:
                self.setup_coordinator.command_controller.cancel(request_id)
            except LookupError:
                pass
        with self._lock:
            self._drafts.pop(draft.context.context_id, None)
            self._context_locks.pop(draft.context.context_id, None)
        self.refinement_controller.discard_context(draft.context)
        self.finalization_controller.discard_context(draft.context)
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
        (
            routine_ids,
            view_ids,
            camera_ids,
            probe_ids,
            reference_tag_id,
            reference_tag_family,
            selected_sensor_id,
        ) = (
            self.definition_service.selected_definition_lists(
                draft.selected_object_id,
                draft.selected_routine_id,
                object_ids,
            )
        )
        return ProbeSetupSnapshot.from_draft(
            draft=draft,
            object_ids=object_ids,
            routine_ids=routine_ids,
            reference_view_ids=view_ids,
            reference_camera_ids=camera_ids,
            selected_reference_tag_id=reference_tag_id,
            selected_reference_tag_family=reference_tag_family,
            selected_sensor_id=selected_sensor_id,
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
        object_name, routine_name = (
            self.definition_service.select_routine(
                object_id,
                routine_id,
            )
        )
        with self._lock:
            draft.selected_object_id = object_name
            draft.selected_routine_id = routine_name
            draft.selected_reference_view_id = ""
            draft.clear_geometry()
        return self._advance(draft)

    @_serialized_transaction
    def select_object(
        self,
        context: SetupContextSnapshot,
        object_id: str,
    ) -> ProbeSetupSnapshot:
        """Select one existing object and clear routine dependencies."""
        draft = self._draft(context)
        object_name = self.definition_service.select_object(
            object_id
        )
        with self._lock:
            draft.selected_object_id = object_name
            draft.selected_routine_id = ""
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
        definition = self.definition_service.create_object(
            object_id,
            display_name,
            reference_tag_id,
            reference_tag_family,
        )
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
        object_name = self.definition_service.delete_object(
            object_id
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
        object_name, routine = self.definition_service.create_routine(
            object_id,
            routine_id,
            display_name,
            sensor_id,
        )
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
        object_name, routine_name = (
            self.definition_service.delete_routine(
                object_id,
                routine_id,
            )
        )
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
        self.geometry_editor.select_reference_pixel(
            draft,
            reference_view_id,
            pixel,
            approach_mode,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        return self._advance(draft)

    @_serialized_transaction
    def clear_reference_pixel(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Clear the selected pixel and every geometry dependency."""
        draft = self._selected_draft(context)
        self.geometry_editor.clear_reference_pixel(draft)
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
        self.geometry_editor.update_geometry(
            draft,
            approach_mode,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        return self._advance(draft)

    @_serialized_transaction
    def approve_safe_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved obstacle-safe pose."""
        draft = self._selected_draft(context)
        self.refinement_controller.approve(
            draft,
            RefinementStage.SAFE_APPROACH,
        )
        return self._advance(draft)

    @_serialized_transaction
    def approve_aligned_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved surface-aligned pre-approach pose."""
        draft = self._selected_draft(context)
        self.refinement_controller.approve(
            draft,
            RefinementStage.ALIGNMENT,
        )
        return self._advance(draft)

    @_serialized_transaction
    def approve_probe_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved final probe pose."""
        draft = self._selected_draft(context)
        self.refinement_controller.approve(
            draft,
            RefinementStage.PROBE,
        )
        return self._advance(draft)

    @_serialized_transaction
    def begin_refinement(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Start one server-owned supervised refinement session."""
        draft = self._selected_draft(context)
        self.refinement_controller.begin(draft)
        return self._advance(draft)

    @_serialized_transaction
    def end_refinement(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Discard unapproved candidates and close refinement state."""
        draft = self._selected_draft(context)
        if not self.refinement_controller.end(draft):
            return self.snapshot(context)
        return self._advance(draft)

    def begin_surface_verification(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Start one server-owned closed-loop surface verification."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            self._require_idle(context)
            self.refinement_controller.require_physical_lane_idle()
            draft = self._selected_draft(context)
            self.surface_controller.begin(draft, request_id)
            return self.snapshot(context)

    def evaluate_surface_verification(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        samples,
        achieved_pose_object,
    ):
        """Evaluate one authoritative surface-distance sample window."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            decision = self.surface_controller.evaluate(
                draft,
                request_id,
                samples,
                achieved_pose_object,
            )
            return decision, self.snapshot(context)

    def mark_surface_correction_started(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Record that a verification correction may have moved the probe."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.mark_correction_started(
                draft,
                request_id,
            )
            return self.snapshot(context)

    def mark_surface_correction_succeeded(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Enter the settle gate after one correction succeeds."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.mark_correction_succeeded(
                draft,
                request_id,
            )
            return self.snapshot(context)

    def resume_surface_sampling(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Resume fresh sampling after the server settle gate passes."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.resume_sampling(
                draft,
                request_id,
            )
            return self.snapshot(context)

    def fail_surface_sampling(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        detail: str,
    ) -> ProbeSetupSnapshot:
        """Fail a verification whose fresh-depth gate timed out."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.fail_sampling(
                draft,
                request_id,
                detail,
            )
            return self.snapshot(context)

    def fail_surface_correction(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        detail: str,
    ) -> ProbeSetupSnapshot:
        """Fail one correlated axial correction."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.fail_correction(
                draft,
                request_id,
                detail,
            )
            return self.snapshot(context)

    def abort_surface_verification(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        detail: str,
    ) -> ProbeSetupSnapshot:
        """Fail any active verification while preserving recovery state."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.abort(
                draft,
                request_id,
                detail,
            )
            return self.snapshot(context)

    def cancel_surface_verification(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Cancel verification and preserve any required recovery state."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.surface_controller.cancel(
                draft,
                request_id,
            )
            return self.snapshot(context)

    def begin_finalization(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        save_requested: bool,
    ) -> ProbeSetupSnapshot:
        """Lock one refinement for server-owned save and retraction."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            self._require_idle(
                context,
                finalization_request_id=request_id,
            )
            draft = self._selected_draft(context)
            self.finalization_controller.begin(
                context,
                draft,
                request_id,
                save_requested,
            )
            return self.snapshot(context)

    def approve_verified_probe_for_finalization(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Approve the converged probe candidate without another TF sample."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.finalization_controller.approve_verified_probe(
                context,
                draft,
                request_id,
            )
            return self._advance(draft)

    def save_probe_point_for_finalization(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        probe_point_id: str,
        display_name: str,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        measurement_duration_sec: float,
    ) -> ProbeSetupSnapshot:
        """Persist the approved point while the finalization lock is held."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.finalization_controller.save_for_finalization(
                context,
                draft,
                request_id,
                probe_point_id,
                display_name,
                position_tolerance_m,
                orientation_tolerance_rad,
                measurement_duration_sec,
            )
            return self._advance(draft)

    def complete_finalization(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> ProbeSetupSnapshot:
        """Close refinement only after both retraction motions succeeded."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.finalization_controller.complete(
                context,
                draft,
                request_id,
            )
            return self._advance(draft)

    def fail_finalization(
        self,
        context: SetupContextSnapshot,
        request_id: str,
        detail: str,
    ) -> ProbeSetupSnapshot:
        """Release the action lock while preserving mandatory recovery."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            draft = self._selected_draft(context)
            self.finalization_controller.fail(
                context,
                draft,
                request_id,
                detail,
            )
            return self._advance(draft)

    def prepare_motion(
        self,
        context: SetupContextSnapshot,
        motion: ProbeMotionRequest,
        surface_verification_request_id: str = "",
        finalization_request_id: str = "",
    ) -> ProbeSetupMotionOperation:
        """Prepare one motion primitive for the shared command lane."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            self._require_idle(
                context,
                surface_verification_request_id,
                finalization_request_id,
            )
            draft = self._selected_draft(context)
            operation = self.refinement_controller.prepare_motion(
                context,
                draft,
                motion,
            )
            return ProbeSetupMotionOperation(operation, motion)

    def submit_motion(
        self,
        operation: ProbeSetupMotionOperation,
    ) -> str:
        """Submit one prepared motion through the shared command lane."""
        if not isinstance(operation, ProbeSetupMotionOperation):
            raise TypeError("Expected a ProbeSetupMotionOperation")
        tracked = self.refinement_controller.tracked_operation(
            operation.operation,
            operation.motion,
        )
        try:
            return self.setup_coordinator.submit(
                operation.operation
            )
        except Exception:
            with self._context_lock(tracked.context):
                draft = self._draft(tracked.context)
                self.refinement_controller.fail_submission(
                    operation.request_id,
                    draft,
                )
            raise

    def cancel_motion(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> str:
        """Cancel one motion owned by the current probe context."""
        return self.refinement_controller.cancel_motion(
            context,
            request_id,
        )

    def add_motion_status_listener(self, listener) -> None:
        """Register one probe motion status listener."""
        self.refinement_controller.add_listener(listener)

    def remove_motion_status_listener(self, listener) -> None:
        """Remove one probe motion status listener."""
        self.refinement_controller.remove_listener(listener)

    @_serialized_transaction
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
        self.finalization_controller.save_probe_point(
            draft,
            probe_point_id,
            display_name,
            position_tolerance_m,
            orientation_tolerance_rad,
            measurement_duration_sec,
        )
        return self._advance(draft)



    def close(self) -> None:
        """Close every probe context owned by this coordinator."""
        with self._lock:
            contexts = tuple(
                draft.context
                for draft in self._drafts.values()
            )
        for context in contexts:
            if self.setup_coordinator.is_current(context):
                self.close_context(context)
        self.refinement_controller.clear()
        self.finalization_controller.clear()

    def _handle_operation_status(
        self,
        status: SetupOperationStatus,
    ) -> None:
        tracked = self.refinement_controller.tracked_status(
            status
        )
        if tracked is None:
            return
        context = tracked.context
        motion = tracked.payload[0]
        terminal = status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }
        if terminal:
            with self._context_lock(context):
                draft = self._draft(context)
                handled = (
                    self.refinement_controller.handle_terminal_status(
                        status,
                        draft,
                    )
                )
                if handled is None:
                    return
                motion, status = handled
                snapshot = self._advance(draft)
        else:
            snapshot = self.snapshot(context)
        self.refinement_controller.emit(
            ProbeSetupMotionStatus(
                request_id=status.operation.request_id,
                motion=motion,
                state=status.state,
                detail=status.detail,
                snapshot=snapshot,
            )
        )





    def _require_idle(
        self,
        context: SetupContextSnapshot,
        surface_verification_request_id: str = "",
        finalization_request_id: str = "",
    ) -> None:
        with self._lock:
            draft = self._drafts.get(context.context_id)
            verification_request_id = (
                self.surface_controller.active_request_id(draft)
                if draft is not None
                else ""
            )
            if (
                verification_request_id
                and verification_request_id
                != surface_verification_request_id
            ):
                raise RuntimeError(
                    "Probe setup context has active surface verification"
                )
            active_finalization = (
                self.finalization_controller.active_request_id(
                    context
                )
            )
            if (
                active_finalization
                and active_finalization
                != finalization_request_id
            ):
                raise RuntimeError(
                    "Probe setup context has active finalization"
                )
            if (
                draft is not None
                and draft.refinement is not None
                and draft.refinement.recovery_required
                and not (
                    surface_verification_request_id
                    or finalization_request_id
                )
            ):
                raise RuntimeError(
                    "Probe refinement requires retraction before editing"
                )
        if self.refinement_controller.has_context(context):
            raise RuntimeError(
                "Probe setup context already has an active motion"
            )




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




__all__ = [
    "ProbeSetupCoordinator",
    "ProbeSetupMotionOperation",
    "ProbeSetupMotionStatus",
]
