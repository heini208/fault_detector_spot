"""Geometry calculation and draft editing for probe setup."""

from copy import deepcopy

from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
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
        setup = self._retained_distance_approvals(
            geometry.probe_setup,
            previous,
        )
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
