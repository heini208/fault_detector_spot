"""Coordinate probe authoring and single-step setup movement."""

import math
from copy import deepcopy
from dataclasses import dataclass
from functools import wraps
from threading import RLock

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.setup.setup_coordinator import (
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
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
    ProbeSetupMotionCommandFactory,
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
        self.geometry = geometry or ProbeSetupGeometry(reference_repository)
        self.motion_state_source = motion_state_source
        self.motion_command_factory = (
            motion_command_factory or ProbeSetupMotionCommandFactory()
        )
        self._lock = RLock()
        self._drafts = {}
        self._context_locks = {}
        self._pending = {}
        self._motion_listeners = []

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
            request_ids = tuple(
                request_id
                for request_id, pending in self._pending.items()
                if pending[1].context_id == context.context_id
            )
        for request_id in request_ids:
            try:
                self.setup_coordinator.command_controller.cancel(request_id)
            except LookupError:
                pass
        with self._lock:
            self._drafts.pop(draft.context.context_id, None)
            self._context_locks.pop(draft.context.context_id, None)
            self._pending = {
                request_id: pending
                for request_id, pending in self._pending.items()
                if pending[1].context_id != context.context_id
            }
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
            self._selected_definition_lists(draft, object_ids)
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
    def select_object(
        self,
        context: SetupContextSnapshot,
        object_id: str,
    ) -> ProbeSetupSnapshot:
        """Select one existing object and clear routine dependencies."""
        draft = self._draft(context)
        object_name = self._name(object_id, "object ID")
        self.object_repository.load(object_name)
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
            if draft.refinement is not None:
                draft.refinement = (
                    draft.refinement.with_updated_surface_geometry(
                        geometry.probe_setup,
                        setup,
                    )
                )
            draft.dirty = True
            draft.validation_error = ""
        return self._advance(draft)

    @_serialized_transaction
    def approve_safe_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved obstacle-safe pose."""
        return self._approve(
            context,
            approve_safe_approach_pose,
            RefinementStage.SAFE_APPROACH,
        )

    @_serialized_transaction
    def approve_aligned_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved surface-aligned pre-approach pose."""
        return self._approve(
            context,
            approve_surface_alignment_pose,
            RefinementStage.ALIGNMENT,
        )

    @_serialized_transaction
    def approve_probe_pose(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Approve one achieved final probe pose."""
        return self._approve(
            context,
            approve_probe_pose,
            RefinementStage.PROBE,
        )

    @_serialized_transaction
    def begin_refinement(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Start one server-owned supervised refinement session."""
        self._require_physical_lane_idle()
        draft = self._selected_draft(context)
        if draft.geometry is None or draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        if draft.refinement is not None:
            raise RuntimeError("Probe refinement is already active")
        refinement = ProbeRefinementSession.create(
            draft.geometry.probe_setup,
            draft.setup,
        )
        refinement.active_stage = RefinementStage.SAFE_APPROACH
        with self._lock:
            draft.refinement = refinement
        return self._advance(draft)

    @_serialized_transaction
    def end_refinement(
        self,
        context: SetupContextSnapshot,
    ) -> ProbeSetupSnapshot:
        """Discard unapproved candidates and close refinement state."""
        draft = self._selected_draft(context)
        if draft.refinement is None:
            return self.snapshot(context)
        draft.refinement.discard_unapproved_candidates()
        with self._lock:
            draft.refinement = None
        return self._advance(draft)

    def prepare_motion(
        self,
        context: SetupContextSnapshot,
        motion: ProbeMotionRequest,
    ) -> ProbeSetupMotionOperation:
        """Prepare one motion primitive for the shared command lane."""
        with self._context_lock(context):
            self.setup_coordinator.require_current(context)
            self._require_idle(context)
            self._require_physical_lane_idle()
            motion.validate()
            draft = self._selected_draft(context)
            refinement = self._require_refinement(draft)
            stage = self._motion_stage(motion.kind)
            refinement.active_stage = stage
            if motion.relative:
                command = self._relative_motion_command(draft, motion)
                target = refinement.candidate_pose(stage)
                purpose = f"{stage.value} adjustment"
            else:
                target = refinement.candidate_pose(stage)
                command = self._absolute_motion_command(draft, target)
                purpose = stage.value
            operation = self.setup_coordinator.prepare_command(
                context,
                command,
            )
            pending_motion = PendingRefinementMotion(
                request_id=operation.request_id,
                stage=stage,
                purpose=purpose,
                target_pose_object=deepcopy(target),
                updates_candidate=motion.relative,
                verify_achieved_pose=not motion.relative,
            )
            refinement.begin_motion(pending_motion)
            with self._lock:
                self._pending[operation.request_id] = (
                    motion,
                    context,
                    stage,
                )
            return ProbeSetupMotionOperation(operation, motion)

    def submit_motion(
        self,
        operation: ProbeSetupMotionOperation,
    ) -> str:
        """Submit one prepared motion through the shared command lane."""
        if not isinstance(operation, ProbeSetupMotionOperation):
            raise TypeError("Expected a ProbeSetupMotionOperation")
        with self._lock:
            pending = self._pending.get(operation.request_id)
        if pending is None or pending[0] != operation.motion:
            raise ValueError("Probe setup motion was not prepared here")
        try:
            return self.setup_coordinator.submit(operation.operation)
        except Exception:
            motion, context, _stage = pending
            with self._context_lock(context):
                draft = self._draft(context)
                refinement = self._require_refinement(draft)
                refinement.fail_motion(
                    operation.request_id,
                    "Motion submission failed",
                )
                with self._lock:
                    self._pending.pop(operation.request_id, None)
            raise

    def cancel_motion(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> str:
        """Cancel one motion owned by the current probe context."""
        self.setup_coordinator.require_current(context)
        normalized = request_id.strip()
        with self._lock:
            pending = self._pending.get(normalized)
        if pending is None or pending[1] != context:
            raise LookupError(f"Unknown probe setup motion: {request_id}")
        return self.setup_coordinator.command_controller.cancel(normalized)

    def add_motion_status_listener(self, listener) -> None:
        """Register one probe motion status listener."""
        if not callable(listener):
            raise TypeError("Listener must be callable")
        with self._lock:
            if listener not in self._motion_listeners:
                self._motion_listeners.append(listener)

    def remove_motion_status_listener(self, listener) -> None:
        """Remove one probe motion status listener."""
        with self._lock:
            if listener in self._motion_listeners:
                self._motion_listeners.remove(listener)

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
        with self._lock:
            self._pending.clear()
            self._motion_listeners.clear()

    def _approve(self, context, operation, stage):
        self._require_physical_lane_idle()
        draft = self._selected_draft(context)
        if draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        refinement = self._require_refinement(draft)
        if (
            stage != RefinementStage.SAFE_APPROACH
            and refinement.motion_states[stage]
            != RefinementMotionState.REACHED
        ):
            raise RuntimeError(
                f"Reach the {stage.value} pose before approval"
            )
        pose_object = self._current_probe_pose(draft)
        if (
            stage == RefinementStage.SAFE_APPROACH
            and refinement.motion_states[stage]
            != RefinementMotionState.REACHED
        ):
            refinement.set_candidate(stage, pose_object)
            refinement.motion_states[stage] = RefinementMotionState.REACHED
        refinement.approve(stage, pose_object)
        setup = operation(draft.setup, deepcopy(pose_object))
        with self._lock:
            draft.setup = setup
            draft.dirty = True
            draft.validation_error = ""
        return self._advance(draft)

    def _handle_operation_status(
        self,
        status: SetupOperationStatus,
    ) -> None:
        with self._lock:
            pending = self._pending.get(status.operation.request_id)
        if pending is None:
            return
        motion, context, _stage = pending
        terminal = status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }
        if terminal:
            with self._context_lock(context):
                draft = self._draft(context)
                refinement = self._require_refinement(draft)
                if status.state == CommandControllerState.SUCCEEDED:
                    try:
                        achieved = self._current_probe_pose(draft)
                        self._verify_achieved_motion(
                            refinement.pending_motion,
                            motion,
                            achieved,
                        )
                        refinement.complete_motion(
                            status.operation.request_id,
                            achieved,
                        )
                    except Exception as exception:
                        refinement.fail_motion(
                            status.operation.request_id,
                            str(exception),
                        )
                        status = SetupOperationStatus(
                            operation=status.operation,
                            state=CommandControllerState.FAILED,
                            detail=str(exception),
                            buffered_command_count=(
                                status.buffered_command_count
                            ),
                        )
                else:
                    refinement.fail_motion(
                        status.operation.request_id,
                        status.detail,
                    )
                with self._lock:
                    self._pending.pop(status.operation.request_id, None)
                snapshot = self._advance(draft)
        else:
            snapshot = self.snapshot(context)
        motion_status = ProbeSetupMotionStatus(
            request_id=status.operation.request_id,
            motion=motion,
            state=status.state,
            detail=status.detail,
            snapshot=snapshot,
        )
        with self._lock:
            listeners = tuple(self._motion_listeners)
        for listener in listeners:
            listener(motion_status)

    def _absolute_motion_command(self, draft, target):
        definition = self.object_repository.load(draft.selected_object_id)
        routine = definition.get_routine(draft.selected_routine_id)
        sensor = self.sensor_repository.load(routine.sensor_id)
        tag = self._motion_state_source().reference_tag(
            definition.reference_tag.tag_id
        )
        return self.motion_command_factory.absolute(
            target,
            sensor.hand_to_probe,
            tag,
        )

    def _relative_motion_command(self, draft, motion):
        definition = self.object_repository.load(draft.selected_object_id)
        routine = definition.get_routine(draft.selected_routine_id)
        frame_id = self.motion_command_factory.frame_id(
            motion.frame,
            routine.sensor_id,
            definition.reference_tag.tag_id,
        )
        return self.motion_command_factory.relative(
            frame_id,
            motion.translation,
            motion.pitch_rad,
            motion.yaw_rad,
        )

    def _current_probe_pose(self, draft):
        definition = self.object_repository.load(draft.selected_object_id)
        routine = definition.get_routine(draft.selected_routine_id)
        return self._motion_state_source().current_probe_pose_object(
            definition.reference_tag.tag_id,
            routine.sensor_id,
        )

    def _motion_state_source(self):
        if self.motion_state_source is None:
            raise RuntimeError("Probe setup motion state is unavailable")
        return self.motion_state_source

    @staticmethod
    def _require_refinement(draft):
        if draft.refinement is None:
            raise RuntimeError("Probe refinement is not active")
        return draft.refinement

    @staticmethod
    def _motion_stage(kind):
        if kind in {
            ProbeMotionKind.MOVE_SAFE_APPROACH,
            ProbeMotionKind.ADJUST_SAFE_APPROACH,
        }:
            return RefinementStage.SAFE_APPROACH
        if kind in {
            ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
            ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
        }:
            return RefinementStage.ALIGNMENT
        raise ValueError(f"Unsupported probe setup motion: {kind}")

    @staticmethod
    def _verify_achieved_motion(pending, motion, achieved):
        if pending is None or not pending.verify_achieved_pose:
            return
        target = pending.target_pose_object
        position_error = math.sqrt(
            (target.position.x - achieved.position.x) ** 2
            + (target.position.y - achieved.position.y) ** 2
            + (target.position.z - achieved.position.z) ** 2
        )
        dot = abs(
            target.orientation.x * achieved.orientation.x
            + target.orientation.y * achieved.orientation.y
            + target.orientation.z * achieved.orientation.z
            + target.orientation.w * achieved.orientation.w
        )
        orientation_error = 2.0 * math.acos(
            max(-1.0, min(1.0, dot))
        )
        if position_error > motion.position_tolerance_m:
            raise RuntimeError(
                "Achieved probe position missed the target by "
                f"{position_error:.4f} m"
            )
        if orientation_error > motion.orientation_tolerance_rad:
            raise RuntimeError(
                "Achieved probe orientation missed the target by "
                f"{math.degrees(orientation_error):.2f} deg"
            )

    def _selected_definition_lists(self, draft, object_ids):
        if draft.selected_object_id not in object_ids:
            return (), (), (), (), -1, "", ""
        definition = self.object_repository.load(draft.selected_object_id)
        routine_ids = tuple(
            routine.routine_id for routine in definition.routines
        )
        routine = definition.get_routine(draft.selected_routine_id)
        if routine is None:
            return (
                routine_ids,
                (),
                (),
                (),
                definition.reference_tag.tag_id,
                definition.reference_tag.tag_family,
                "",
            )
        return (
            routine_ids,
            tuple(view.view_id for view in routine.reference_views),
            tuple(view.camera_id for view in routine.reference_views),
            tuple(point.probe_point_id for point in routine.probe_points),
            definition.reference_tag.tag_id,
            definition.reference_tag.tag_family,
            routine.sensor_id,
        )

    def _require_idle(self, context: SetupContextSnapshot) -> None:
        with self._lock:
            if any(
                pending[1].context_id == context.context_id
                for pending in self._pending.values()
            ):
                raise RuntimeError(
                    "Probe setup context already has an active motion"
                )

    def _require_physical_lane_idle(self) -> None:
        controller = self.setup_coordinator.command_controller
        if controller.active_request_id or controller.queued_request_ids:
            raise RuntimeError(
                "Robot command lane must be idle for probe refinement"
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


__all__ = [
    "ProbeSetupCoordinator",
    "ProbeSetupMotionOperation",
    "ProbeSetupMotionStatus",
]
