"""Probe refinement and motion orchestration."""

import math
from copy import deepcopy

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupOperationStatus,
)
from fault_detector_spot.application.setup.setup_operation_registry import (
    SetupOperationRegistry,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
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
        sensor_repository,
        motion_state_source,
        motion_command_factory,
        state_lock,
    ):
        self.setup_coordinator = setup_coordinator
        self.object_repository = object_repository
        self.sensor_repository = sensor_repository
        self.motion_state_source = motion_state_source
        self.motion_command_factory = motion_command_factory
        self.state_lock = state_lock
        self._operations = SetupOperationRegistry()

    def begin(self, draft) -> None:
        self.require_physical_lane_idle()
        if draft.geometry is None or draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        if draft.refinement is not None:
            raise RuntimeError("Probe refinement is already active")
        refinement = ProbeRefinementSession.create(
            draft.geometry.probe_setup,
            draft.setup,
        )
        refinement.active_stage = RefinementStage.SAFE_APPROACH
        with self.state_lock:
            draft.refinement = refinement
            draft.surface_verification = None

    def end(self, draft) -> bool:
        if draft.refinement is None:
            return False
        draft.refinement.discard_unapproved_candidates()
        with self.state_lock:
            draft.refinement = None
            draft.surface_verification = None
        return True

    def approve(self, draft, stage: RefinementStage) -> None:
        self.require_physical_lane_idle()
        if draft.setup is None:
            raise ValueError("No calculated probe setup is available")
        refinement = self.require_refinement(draft)
        if (
            stage != RefinementStage.SAFE_APPROACH
            and refinement.motion_states[stage]
            != RefinementMotionState.REACHED
        ):
            raise RuntimeError(
                f"Reach the {stage.value} pose before approval"
            )
        pose_object = self.current_probe_pose(draft)
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
        operation = self._approval_operation(stage)
        setup = operation(draft.setup, deepcopy(pose_object))
        with self.state_lock:
            draft.setup = setup
            draft.dirty = True
            draft.validation_error = ""

    def prepare_motion(self, context, draft, motion):
        self.require_physical_lane_idle()
        motion.validate()
        refinement = self.require_refinement(draft)
        stage = self.motion_stage(motion.kind)
        refinement.active_stage = stage
        surface_correction = (
            motion.kind is ProbeMotionKind.ADJUST_PROBE_DISTANCE
        )
        if motion.relative:
            command = self._relative_motion_command(draft, motion)
            target = refinement.candidate_pose(stage)
            purpose = (
                "surface-distance correction"
                if surface_correction
                else f"{stage.value} adjustment"
            )
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
        """Return registry metadata for one prepared probe motion."""
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
        return self.setup_coordinator.command_controller.cancel(
            normalized
        )

    def tracked_status(self, status):
        return self._operations.get(status.operation.request_id)

    def handle_terminal_status(self, status, draft):
        tracked = self._operations.get(status.operation.request_id)
        if tracked is None:
            return None
        motion, _stage = tracked.payload
        if status.state not in _TERMINAL_STATES:
            return motion, status
        refinement = self.require_refinement(draft)
        if status.state == CommandControllerState.SUCCEEDED:
            try:
                achieved = self.current_probe_pose(draft)
                self.verify_achieved_motion(
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
        self._operations.pop(status.operation.request_id)
        return motion, status

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

    def require_physical_lane_idle(self) -> None:
        controller = self.setup_coordinator.command_controller
        if controller.active_request_id or controller.queued_request_ids:
            raise RuntimeError(
                "Robot command lane must be idle for probe refinement"
            )

    @staticmethod
    def require_refinement(draft):
        if draft.refinement is None:
            raise RuntimeError("Probe refinement is not active")
        return draft.refinement

    def current_probe_pose(self, draft):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        routine = definition.get_routine(
            draft.selected_routine_id
        )
        source = self._motion_state_source()
        return source.current_probe_pose_object(
            definition.reference_tag.tag_id,
            routine.sensor_id,
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

    def _absolute_motion_command(self, draft, target):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        routine = definition.get_routine(
            draft.selected_routine_id
        )
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
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        routine = definition.get_routine(
            draft.selected_routine_id
        )
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
