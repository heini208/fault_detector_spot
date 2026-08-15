"""Geometry calculation and draft editing for probe setup."""

from copy import deepcopy
import math

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    add_vectors,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    rotate_vector,
    scale_vector,
)


class ProbeGeometryEditor:
    """Own reference-pixel geometry calculation and draft mutation."""

    def __init__(
        self,
        object_repository,
        sensor_repository,
        geometry,
    ):
        self.object_repository = object_repository
        self.sensor_repository = sensor_repository
        self.geometry = geometry

    def select_reference_pixel(
        self,
        draft,
        reference_view_id: str,
        pixel: ImagePoint,
        approach_mode: str,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
    ) -> None:
        pixel.validate()
        view_id = self._text(
            reference_view_id,
            "reference view ID",
        )
        geometry = self._resolve_geometry(
            draft,
            view_id,
            pixel,
            approach_mode,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        draft.selected_reference_view_id = view_id
        draft.reference_pixel = deepcopy(pixel)
        draft.geometry = geometry
        draft.setup = deepcopy(geometry.probe_setup)
        draft.dirty = True
        draft.validation_error = ""

    @staticmethod
    def clear_reference_pixel(draft) -> None:
        draft.selected_reference_view_id = ""
        draft.clear_geometry()

    def update_geometry(
        self,
        draft,
        approach_mode: str,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
    ) -> None:
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
        aligned_distance_changed = self._aligned_distance_changed(
            previous,
            geometry.probe_setup,
        )
        setup = self._retained_distance_approvals(
            geometry.probe_setup,
            previous,
        )
        draft.geometry = geometry
        draft.setup = setup
        if draft.refinement is not None:
            previous_refinement = draft.refinement
            updated_refinement = (
                previous_refinement.with_updated_surface_geometry(
                    geometry.probe_setup,
                    setup,
                )
            )
            if aligned_distance_changed:
                previous_candidate = previous_refinement.candidate_pose(
                    RefinementStage.ALIGNMENT
                )
                shifted_candidate = self._shift_aligned_candidate(
                    previous_candidate,
                    previous.surface_target.aligned_preapproach_distance_m,
                    geometry.probe_setup.surface_target
                    .aligned_preapproach_distance_m,
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

    def _resolve_geometry(
        self,
        draft,
        reference_view_id,
        pixel,
        approach_mode,
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    ):
        definition = self.object_repository.load(
            draft.selected_object_id
        )
        routine = definition.get_routine(
            draft.selected_routine_id
        )
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

    @classmethod
    def _retained_distance_approvals(cls, setup, previous):
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

    @staticmethod
    def _aligned_distance_changed(previous, current) -> bool:
        if previous is None:
            return False
        return not math.isclose(
            previous.surface_target.aligned_preapproach_distance_m,
            current.surface_target.aligned_preapproach_distance_m,
            rel_tol=0.0,
            abs_tol=1e-9,
        )

    @staticmethod
    def _shift_aligned_candidate(
        candidate: PoseData,
        previous_distance_m: float,
        updated_distance_m: float,
    ) -> PoseData:
        inward = rotate_vector(
            candidate.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        shift_m = float(previous_distance_m) - float(updated_distance_m)
        result = PoseData(
            position=add_vectors(
                candidate.position,
                scale_vector(inward, shift_m),
            ),
            orientation=deepcopy(candidate.orientation),
        )
        result.validate()
        return result

    @staticmethod
    def _text(value: str, label: str) -> str:
        if not isinstance(value, str):
            raise TypeError(f"{label.title()} must be text")
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


__all__ = ["ProbeGeometryEditor"]
