"""Finalization state and probe-point persistence for probe setup."""

from copy import deepcopy
from dataclasses import replace

from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.inspection.model.models import ProbePoint
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage, RefinementMotionState,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_surface_alignment_pose,
)
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class ProbeFinalizationController:
    """Own finalization locks, persistence, and recovery transitions."""

    def __init__(
        self,
        object_repository,
        refinement_controller,
        state_lock,
    ):
        self.object_repository = object_repository
        self.refinement_controller = refinement_controller
        self.state_lock = state_lock
        self._active = {}

    def begin(
        self,
        context,
        draft,
        request_id: str,
        save_requested: bool,
    ) -> None:
        normalized = validate_request_id(request_id)
        if not isinstance(save_requested, bool):
            raise TypeError(
                "Save requested flag must be a boolean"
            )
        self.refinement_controller.require_physical_lane_idle()
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        for stage, label in (
            (RefinementStage.SAFE_APPROACH, "safe approach"),
            (RefinementStage.ALIGNMENT, "aligned pre-approach"),
        ):
            if not refinement.stage_is_approved(stage):
                raise ValueError(
                    f"Approve the {label} before finalization"
                )
        if save_requested and draft.setup.fully_custom:
            if (not refinement.stage_is_approved(RefinementStage.PROBE)
                    or refinement.motion_states[RefinementStage.PROBE] is not RefinementMotionState.REACHED):
                raise ValueError("Reach and save the custom final candidate before finalization")
        with self.state_lock:
            self._active[context.context_id] = normalized

    def approve_probe_geometry(
        self,
        context,
        draft,
        request_id: str,
    ) -> None:
        """Approve the probe geometry derived from the aligned pose."""
        self.require(context, request_id)
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        setup = draft.setup
        if setup is None:
            raise ValueError("No calculated probe setup is available")
        if setup.fully_custom:
            if not refinement.stage_is_approved(RefinementStage.PROBE):
                raise ValueError("Save the custom final probe candidate before finalization")
            return
        execution_setup = approve_surface_alignment_pose(
            setup,
            setup.aligned_preapproach_pose_object,
        )
        refinement.approve(
            RefinementStage.PROBE,
            execution_setup.probe_pose_object,
        )
        with self.state_lock:
            draft.setup = replace(
                execution_setup,
                probe_pose_approved=True,
            )
            draft.dirty = True
            draft.validation_error = ""

    def save_for_finalization(
        self,
        context,
        draft,
        request_id: str,
        probe_point_id: str,
        display_name: str,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        measurement_duration_sec: float,
    ) -> None:
        self.require(context, request_id)
        self.save_probe_point(
            draft,
            probe_point_id,
            display_name,
            position_tolerance_m,
            orientation_tolerance_rad,
            measurement_duration_sec,
        )

    def complete(
        self,
        context,
        draft,
        request_id: str,
    ) -> None:
        self.require(context, request_id)
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        refinement.complete_retraction()
        if not self.refinement_controller.end(draft):
            raise RuntimeError("Probe refinement ended during finalization")
        with self.state_lock:
            self._active.pop(context.context_id, None)

    def complete_saved_after_retraction_failure(
        self,
        context,
        draft,
        request_id: str,
    ) -> None:
        self.require(context, request_id)
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        if not refinement.saved:
            raise RuntimeError(
                "Cannot finish failed retraction before probe point save"
            )
        self.refinement_controller.abort(draft)
        with self.state_lock:
            self._active.pop(context.context_id, None)

    def fail(
        self,
        context,
        draft,
        request_id: str,
        detail: str,
    ) -> None:
        self.require(context, request_id)
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        refinement.require_recovery(detail)
        with self.state_lock:
            self._active.pop(context.context_id, None)

    def save_probe_point(
        self,
        draft,
        probe_point_id: str,
        display_name: str,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        measurement_duration_sec: float,
    ) -> None:
        setup = draft.setup
        if setup is None:
            raise ValueError(
                "No calculated probe setup is available"
            )
        if not all((
            setup.safe_approach_approved,
            setup.surface_alignment_approved,
            setup.probe_pose_approved,
        )):
            raise ValueError(
                "All three setup requirements must be approved: "
                "safe approach, aligned pre-approach, and probe geometry"
            )
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
        with self.state_lock:
            draft.dirty = False
            draft.validation_error = ""
            if draft.refinement is not None:
                draft.refinement.saved = True

    def require(
        self,
        context,
        request_id: str,
    ) -> str:
        normalized = validate_request_id(request_id)
        with self.state_lock:
            active = self._active.get(context.context_id)
        if active != normalized:
            raise RuntimeError(
                "Probe refinement finalization request does not match"
            )
        return normalized

    def active_request_id(self, context) -> str:
        with self.state_lock:
            return self._active.get(context.context_id, "")

    def discard_context(self, context) -> None:
        with self.state_lock:
            self._active.pop(context.context_id, None)

    def clear(self) -> None:
        with self.state_lock:
            self._active.clear()

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
            pre_approach_path=deepcopy(draft.pre_approach_path),
            fully_custom=setup.fully_custom,
            final_probe_pose_object=deepcopy(setup.probe_pose_object) if setup.fully_custom else None,
            final_probe_path=deepcopy(draft.final_probe_path),
            pre_approach_speed_scale=draft.pre_approach_speed_scale,
            final_probe_speed_scale=draft.final_probe_speed_scale,
            final_position_tolerance_m=draft.final_position_tolerance_m,
            probe_point_id=self._name(
                probe_point_id,
                "probe point ID",
            ),
            display_name=self._text(
                display_name,
                "probe point display name",
            ),
            aligned_preapproach_pose_object=deepcopy(
                setup.aligned_preapproach_pose_object
            ),
            target_surface_distance_m=(
                0.0 if setup.fully_custom else setup.surface_target.target_surface_distance_m
            ),
            position_tolerance_m=float(
                draft.aligned_position_tolerance_m
                if draft.aligned_position_tolerance_m is not None else position_tolerance_m
            ),
            orientation_tolerance_rad=float(
                orientation_tolerance_rad
            ),
            measurement_duration_sec=float(
                measurement_duration_sec
            ),
            aligned_preapproach_distance_m=(
                0.0 if setup.fully_custom else setup.surface_target.aligned_preapproach_distance_m
            ),
            reference_pixel=deepcopy(
                draft.reference_pixel
            ),
            reference_view_id=(
                draft.selected_reference_view_id or None
            ),
        )
        point.validate()
        return point

    @staticmethod
    def _name(value: str, label: str) -> str:
        return validate_storage_name(
            value.strip(),
            label,
        )

    @staticmethod
    def _text(value: str, label: str) -> str:
        if not isinstance(value, str):
            raise TypeError(
                f"{label.title()} must be text"
            )
        normalized = value.strip()
        if not normalized:
            raise ValueError(
                f"{label.title()} must not be empty"
            )
        if normalized != value:
            raise ValueError(
                f"{label.title()} must not contain surrounding whitespace"
            )
        return normalized


__all__ = ["ProbeFinalizationController"]
