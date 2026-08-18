"""Probe refinement and motion orchestration."""

import math
from copy import deepcopy
from dataclasses import replace

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupOperationStatus,
)
from fault_detector_spot.application.setup.setup_operation_registry import (
    SetupOperationRegistry,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeAlignmentOrientationMode,
    ProbeMotionKind,
)
from fault_detector_spot.inspection.setup.alignment_orientation import (
    tag_aligned_probe_orientation,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    add_vectors,
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    rotate_vector,
    scale_vector,
)


_TERMINAL_STATES = {
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
}


class ProbeRefinementController:
    """Own supervised probe refinement and its physical motions."""

    def __init__(
        self,
        setup_coordinator,
        object_repository,
        motion_state_source,
        motion_command_factory,
        state_lock,
        sensor_attachment_controller=None,
    ):
        self.setup_coordinator = setup_coordinator
        self.object_repository = object_repository
        self.motion_state_source = motion_state_source
        self.motion_command_factory = motion_command_factory
        self.state_lock = state_lock
        self.sensor_attachment_controller = sensor_attachment_controller
        self._operations = SetupOperationRegistry()
        self._attachment_reservations = {}

    def begin(self, draft) -> None:
        self.require_physical_lane_idle()
        if draft.geometry is None or draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        if draft.refinement is not None:
            raise RuntimeError("Probe refinement is already active")
        reservation = self._acquire_attachment()
        attachment = reservation.attachment
        try:
            self._ensure_minimum_camera_clearance_geometry(
                draft,
                attachment,
            )
            refinement = ProbeRefinementSession.create(
                draft.geometry.probe_setup,
                draft.setup,
            )
            if not draft.setup.safe_approach_approved:
                refinement.seed_safe_approach_from_current_pose(
                    self.current_probe_pose(draft, attachment)
                )
            refinement.active_stage = RefinementStage.SAFE_APPROACH
            with self.state_lock:
                draft.refinement = refinement
                self._attachment_reservations[id(draft)] = reservation
        except Exception:
            reservation.release()
            raise

    def end(self, draft) -> bool:
        if draft.refinement is None:
            return False
        draft.refinement.discard_unapproved_candidates()
        with self.state_lock:
            draft.refinement = None
        self._release_attachment(draft)
        return True

    def abort(self, draft) -> None:
        """Discard runtime refinement and release its frozen attachment."""
        with self.state_lock:
            draft.refinement = None
        self._release_attachment(draft)

    def approve(self, draft, stage: RefinementStage) -> None:
        self.require_physical_lane_idle()
        if draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        attachment = self._active_attachment(draft)
        if stage is RefinementStage.ALIGNMENT:
            self._ensure_minimum_camera_clearance_geometry(
                draft,
                attachment,
            )
        refinement = self.require_refinement(draft)
        if stage is RefinementStage.ALIGNMENT:
            if refinement.motion_states[stage] not in {
                RefinementMotionState.REACHED,
                RefinementMotionState.FAILED,
            }:
                raise RuntimeError(
                    "Reach or attempt the alignment pose before approval"
                )
            self._require_live_alignment_camera_clearance()
        elif (
            stage != RefinementStage.SAFE_APPROACH
            and refinement.motion_states[stage]
            != RefinementMotionState.REACHED
        ):
            raise RuntimeError(
                f"Reach the {stage.value} pose before approval"
            )
        pose_object = self.current_probe_pose(draft, attachment)
        if (
            stage == RefinementStage.SAFE_APPROACH
            and refinement.motion_states[stage]
            != RefinementMotionState.REACHED
        ):
            refinement.set_candidate(stage, pose_object)
            refinement.motion_states[stage] = (
                RefinementMotionState.REACHED
            )
        refinement.approve(stage, pose_object)
        if stage is RefinementStage.ALIGNMENT:
            refinement.motion_states[stage] = RefinementMotionState.REACHED
        operation = self._approval_operation(stage)
        setup = operation(draft.setup, deepcopy(pose_object))
        with self.state_lock:
            draft.setup = setup
            draft.dirty = True
            draft.validation_error = ""

    def prepare_motion(self, context, draft, motion):
        self.require_physical_lane_idle()
        motion.validate()
        attachment = self._active_attachment(draft)
        stage = self.motion_stage(motion.kind)
        if stage is RefinementStage.ALIGNMENT:
            self._ensure_minimum_camera_clearance_geometry(
                draft,
                attachment,
            )
        refinement = self.require_refinement(draft)
        refinement.active_stage = stage
        self._invalidate_downstream_motion_state(refinement, stage)
        surface_correction = (
            motion.kind is ProbeMotionKind.ADJUST_PROBE_DISTANCE
        )
        if motion.relative:
            command = self._relative_motion_command(
                draft,
                motion,
                attachment,
            )
            target = refinement.candidate_pose(stage)
            purpose = (
                "surface-distance correction"
                if surface_correction
                else f"{stage.value} adjustment"
            )
        else:
            target = refinement.candidate_pose(stage)
            if stage is RefinementStage.ALIGNMENT:
                candidate = self._alignment_target(
                    draft,
                    target,
                    motion,
                    attachment,
                )
                refinement.set_candidate(stage, candidate)
                target = candidate
                if motion.orientation_only:
                    target = deepcopy(candidate)
                    target.position = deepcopy(
                        self.current_probe_pose(
                            draft,
                            attachment,
                        ).position
                    )
            command = self._absolute_motion_command(
                draft,
                target,
                attachment,
            )
            purpose = (
                "alignment orientation"
                if (
                    stage is RefinementStage.ALIGNMENT
                    and motion.orientation_only
                )
                else stage.value
            )
        operation = self.setup_coordinator.prepare_command(
            context,
            command,
        )
        pending_motion = PendingRefinementMotion(
            request_id=operation.request_id,
            stage=stage,
            purpose=purpose,
            target_pose_object=deepcopy(target),
            updates_candidate=(
                motion.relative and not surface_correction
            ),
            verify_achieved_pose=not motion.relative,
        )
        refinement.begin_motion(pending_motion)
        self._operations.register(
            operation.request_id,
            context,
            (motion, stage),
        )
        return operation

    def require_operation(self, operation):
        tracked = self._operations.owned(
            operation.request_id,
            operation.context,
        )
        if tracked is None:
            raise ValueError(
                "Probe setup motion was not prepared here"
            )
        return tracked

    def fail_submission(self, request_id: str, draft) -> None:
        refinement = self.require_refinement(draft)
        refinement.fail_motion(
            request_id,
            "Motion submission failed",
        )
        self._operations.pop(request_id)

    def cancel_motion(self, context, request_id: str) -> str:
        self.setup_coordinator.require_current(context)
        normalized = request_id.strip()
        if self._operations.owned(normalized, context) is None:
            raise LookupError(
                f"Unknown probe setup motion: {request_id}"
            )
        return self.setup_coordinator.cancel_operation(
            context,
            normalized,
        )

    def tracked_status(self, status):
        return self._operations.get(status.operation.request_id)

    def handle_terminal_status(self, status, draft):
        tracked = self._operations.get(status.operation.request_id)
        if tracked is None:
            return None
        motion, stage = tracked.payload
        if status.state not in _TERMINAL_STATES:
            return motion, status
        refinement = self.require_refinement(draft)
        if status.state == CommandControllerState.SUCCEEDED:
            try:
                attachment = self._active_attachment(draft)
                achieved = self.current_probe_pose(
                    draft,
                    attachment,
                )
                self.verify_achieved_motion(
                    refinement.pending_motion,
                    motion,
                    achieved,
                )
                if (
                    stage is RefinementStage.ALIGNMENT
                    and not motion.orientation_only
                ):
                    self._require_live_alignment_camera_clearance()
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
        self._operations.pop(status.operation.request_id)
        return motion, status

    @staticmethod
    def _invalidate_downstream_motion_state(refinement, stage) -> None:
        if stage is RefinementStage.SAFE_APPROACH:
            downstream = (
                RefinementStage.ALIGNMENT,
                RefinementStage.PROBE,
            )
        elif stage is RefinementStage.ALIGNMENT:
            downstream = (RefinementStage.PROBE,)
        else:
            downstream = ()
        for downstream_stage in downstream:
            refinement.motion_states[downstream_stage] = (
                RefinementMotionState.NOT_TESTED
            )
        if downstream:
            refinement.surface_distance_verified = False

    def add_listener(self, listener) -> None:
        self._operations.add_listener(listener)

    def remove_listener(self, listener) -> None:
        self._operations.remove_listener(listener)

    def emit(self, status) -> None:
        self._operations.emit(status)

    def request_ids_for(self, context) -> tuple:
        return self._operations.request_ids_for(context)

    def discard_context(self, context) -> None:
        self._operations.discard_context(context)

    def has_context(self, context) -> bool:
        return self._operations.has_context(context)

    def clear(self) -> None:
        self._operations.clear()
        with self.state_lock:
            reservations = tuple(self._attachment_reservations.values())
            self._attachment_reservations.clear()
        for reservation in reservations:
            reservation.release()

    def require_physical_lane_idle(self) -> None:
        self.setup_coordinator.require_command_lane_idle(
            "Robot command lane must be idle for probe refinement"
        )

    @staticmethod
    def require_refinement(draft):
        if draft.refinement is None:
            raise RuntimeError("Probe refinement is not active")
        return draft.refinement

    def current_probe_pose(self, draft, attachment=None):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        active = attachment or self._active_attachment(draft)
        source = self._motion_state_source()
        return source.current_probe_pose_object(
            definition.reference_tag.tag_id,
            active.motion_sensor_id,
        )

    @staticmethod
    def motion_stage(kind):
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
        if kind is ProbeMotionKind.ADJUST_PROBE_DISTANCE:
            return RefinementStage.PROBE
        raise ValueError(
            f"Unsupported probe setup motion: {kind}"
        )

    @staticmethod
    def verify_achieved_motion(pending, motion, achieved):
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

    def _ensure_minimum_camera_clearance_geometry(
        self,
        draft,
        attachment=None,
    ):
        if draft.geometry is None or draft.setup is None:
            return None
        source = self.motion_state_source
        resolver = getattr(
            source,
            "minimum_aligned_probe_distance_m",
            None,
        )
        if resolver is None or self.object_repository is None:
            return None
        object_id = getattr(draft, "selected_object_id", "")
        routine_id = getattr(draft, "selected_routine_id", "")
        if not object_id or not routine_id:
            return None
        active = attachment or self._active_attachment(draft)
        minimum_distance_m = float(resolver(active.motion_sensor_id))
        calculated = draft.geometry.probe_setup
        configured_distance_m = float(
            calculated.surface_target.aligned_preapproach_distance_m
        )
        if configured_distance_m + 1e-9 >= minimum_distance_m:
            return minimum_distance_m

        updated_calculated = self._setup_with_aligned_distance(
            calculated,
            minimum_distance_m,
            invalidate_alignment=False,
        )
        updated_setup = self._setup_with_aligned_distance(
            draft.setup,
            minimum_distance_m,
            invalidate_alignment=True,
        )
        draft.geometry = replace(
            draft.geometry,
            surface_target=updated_calculated.surface_target,
            probe_setup=updated_calculated,
        )
        draft.setup = updated_setup
        if draft.refinement is not None:
            previous = draft.refinement
            previous_candidate = previous.candidate_pose(
                RefinementStage.ALIGNMENT
            )
            updated_refinement = previous.with_updated_surface_geometry(
                updated_calculated,
                updated_setup,
            )
            shifted_candidate = self._shift_alignment_pose(
                previous_candidate,
                configured_distance_m,
                minimum_distance_m,
            )
            updated_refinement.set_candidate(
                RefinementStage.ALIGNMENT,
                shifted_candidate,
            )
            updated_refinement.motion_states[
                RefinementStage.ALIGNMENT
            ] = RefinementMotionState.NOT_TESTED
            updated_refinement.motion_states[
                RefinementStage.PROBE
            ] = RefinementMotionState.NOT_TESTED
            draft.refinement = updated_refinement
        draft.dirty = True
        draft.validation_error = ""
        return minimum_distance_m

    def _require_live_alignment_camera_clearance(self):
        source = self._motion_state_source()
        gate = getattr(source, "require_hand_camera_clearance", None)
        if gate is None:
            return None
        return gate()

    @classmethod
    def _setup_with_aligned_distance(
        cls,
        setup,
        aligned_distance_m: float,
        invalidate_alignment: bool,
    ):
        target = setup.surface_target
        previous_distance_m = float(
            target.aligned_preapproach_distance_m
        )
        shifted_alignment = cls._shift_alignment_pose(
            setup.aligned_preapproach_pose_object,
            previous_distance_m,
            aligned_distance_m,
        )
        target_alignment_position = add_vectors(
            target.surface_point_object,
            scale_vector(
                target.outward_direction_object,
                aligned_distance_m,
            ),
        )
        updated_target = replace(
            target,
            aligned_preapproach_pose_object=PoseData(
                position=target_alignment_position,
                orientation=deepcopy(
                    target.aligned_preapproach_pose_object.orientation
                ),
            ),
            aligned_preapproach_distance_m=float(aligned_distance_m),
        )
        inward = rotate_vector(
            shifted_alignment.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        probe_distance_delta = (
            float(aligned_distance_m)
            - float(target.target_surface_distance_m)
        )
        updated_probe = PoseData(
            position=add_vectors(
                shifted_alignment.position,
                scale_vector(inward, probe_distance_delta),
            ),
            orientation=deepcopy(shifted_alignment.orientation),
        )
        return replace(
            setup,
            surface_target=updated_target,
            aligned_preapproach_pose_object=shifted_alignment,
            probe_pose_object=updated_probe,
            surface_alignment_approved=(
                False
                if invalidate_alignment
                else setup.surface_alignment_approved
            ),
            probe_pose_approved=False,
        )

    @staticmethod
    def _shift_alignment_pose(
        pose: PoseData,
        previous_distance_m: float,
        updated_distance_m: float,
    ) -> PoseData:
        inward = rotate_vector(
            pose.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        shift_m = float(previous_distance_m) - float(updated_distance_m)
        result = PoseData(
            position=add_vectors(
                pose.position,
                scale_vector(inward, shift_m),
            ),
            orientation=deepcopy(pose.orientation),
        )
        result.validate()
        return result

    def _alignment_target(
        self,
        draft,
        candidate,
        motion,
        attachment,
    ):
        result = deepcopy(candidate)
        if (
            motion.alignment_orientation_mode
            is ProbeAlignmentOrientationMode.TAG
        ):
            result.orientation = tag_aligned_probe_orientation(
                attachment.hand_to_probe().orientation
            )
        elif (
            motion.alignment_orientation_mode
            is ProbeAlignmentOrientationMode.CALCULATED_SURFACE
        ):
            result.orientation = deepcopy(
                motion.calculated_surface_orientation_object
            )
        else:
            raise ValueError("Unsupported alignment orientation mode")
        result.validate()
        return result

    def _absolute_motion_command(
        self,
        draft,
        target,
        attachment,
    ):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        tag = self._motion_state_source().reference_tag(
            definition.reference_tag.tag_id
        )
        return self.motion_command_factory.absolute(
            target,
            tag,
            attachment.motion_sensor_id,
        )

    def _relative_motion_command(
        self,
        draft,
        motion,
        attachment,
    ):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        frame_id = self.motion_command_factory.frame_id(
            motion.frame,
            attachment.motion_sensor_id,
            definition.reference_tag.tag_id,
        )
        return self.motion_command_factory.relative(
            frame_id,
            motion.translation,
            motion.pitch_rad,
            motion.yaw_rad,
            attachment.motion_sensor_id,
        )

    def motion_attachment(self):
        return self._active_attachment()

    def _active_attachment(self, draft=None):
        controller = self.sensor_attachment_controller
        if controller is None:
            raise RuntimeError(
                "Active sensor attachment state is unavailable"
            )
        if draft is not None and draft.refinement is None:
            return controller.require_motion_attachment()
        with self.state_lock:
            if draft is not None:
                reservation = getattr(
                    self,
                    "_attachment_reservations",
                    {},
                ).get(id(draft))
                if reservation is not None:
                    return reservation.attachment
                if draft.refinement is not None:
                    raise RuntimeError(
                        "Probe refinement has no attachment reservation"
                    )
            active = tuple(
                reservation
                for reservation in self._attachment_reservations.values()
                if not reservation.released
            )
        if len(active) == 1:
            return active[0].attachment
        if active:
            raise RuntimeError(
                "Multiple probe refinements have attachment reservations"
            )
        raise RuntimeError(
            "Probe refinement has no attachment reservation"
        )

    def _acquire_attachment(self):
        controller = self.sensor_attachment_controller
        if controller is None:
            raise RuntimeError(
                "Active sensor attachment state is unavailable"
            )
        return controller.acquire_motion_attachment()

    def _release_attachment(self, draft) -> bool:
        with self.state_lock:
            reservation = self._attachment_reservations.pop(
                id(draft),
                None,
            )
        return reservation.release() if reservation is not None else False

    def _motion_state_source(self):
        if self.motion_state_source is None:
            raise RuntimeError(
                "Probe setup motion state is unavailable"
            )
        return self.motion_state_source

    @staticmethod
    def _approval_operation(stage):
        operations = {
            RefinementStage.SAFE_APPROACH: (
                approve_safe_approach_pose
            ),
            RefinementStage.ALIGNMENT: (
                approve_surface_alignment_pose
            ),
            RefinementStage.PROBE: approve_probe_pose,
        }
        return operations[stage]


__all__ = ["ProbeRefinementController"]
