"""Resolve one saved reference pixel into probe setup geometry."""

from dataclasses import dataclass
from typing import Optional

from fault_detector_spot.inspection.model.models import ImagePoint, PoseData
from fault_detector_spot.inspection.repository import (
    multi_reference_view_repository as reference_repository,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup import (
    reference_view_approach_direction as approach_geometry,
)
from fault_detector_spot.inspection.setup import (
    reference_view_depth_projection as depth_projection,
)
from fault_detector_spot.inspection.setup import (
    reference_view_surface_normal as surface_normal_geometry,
)
from fault_detector_spot.inspection.setup import (
    reference_view_surface_target as surface_target_geometry,
)


@dataclass(frozen=True)
class ProbeGeometryResult:
    """Contain one deterministic reference-pixel calculation."""

    capture: reference_repository.CapturedReferenceView
    projected_point: depth_projection.ProjectedReferencePoint
    surface_normal: Optional[surface_normal_geometry.ReferenceSurfaceNormal]
    surface_normal_error: str
    approach_direction: approach_geometry.ReferenceApproachDirection
    surface_target: surface_target_geometry.ReferenceSurfaceTarget
    probe_setup: ReferenceProbeSetup


class ProbeSetupGeometry:
    """Load reference data and perform only pure geometry composition."""

    def __init__(self, repository):
        self.repository = repository

    def resolve(
        self,
        object_id: str,
        routine_id: str,
        reference_view_id: str,
        pixel: ImagePoint,
        approach_mode: str,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
        hand_to_probe_pose: PoseData,
    ) -> ProbeGeometryResult:
        capture = self._capture(
            object_id,
            routine_id,
            reference_view_id,
        )
        projected, normal, normal_error = self._surface_geometry(
            capture,
            pixel,
        )
        approach = self._approach_direction(
            capture,
            projected,
            normal,
            normal_error,
            approach_mode,
        )
        target = self._surface_target(
            capture,
            approach,
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )
        setup = initialize_reference_probe_setup(
            target,
            hand_to_probe_pose,
        )
        return ProbeGeometryResult(
            capture=capture,
            projected_point=projected,
            surface_normal=normal,
            surface_normal_error=normal_error,
            approach_direction=approach,
            surface_target=target,
            probe_setup=setup,
        )

    @staticmethod
    def _surface_geometry(capture, pixel):
        projected = depth_projection.project_reference_pixel(
            pixel,
            capture.depth_image,
            capture.depth_camera_info,
            rgb_size=(capture.rgb_image.width, capture.rgb_image.height),
            rgb_camera_info=capture.rgb_camera_info,
        )
        normal = None
        normal_error = ""
        try:
            normal = (
                surface_normal_geometry.estimate_reference_surface_normal(
                    projected,
                    capture.depth_image,
                    capture.depth_camera_info,
                )
            )
        except ValueError as exception:
            normal_error = str(exception)
        return projected, normal, normal_error

    @staticmethod
    def _approach_direction(
        capture,
        projected,
        normal,
        normal_error,
        approach_mode,
    ):
        reference_view = capture.reference_view
        if reference_view.controlled_frame != projected.frame_id:
            raise ValueError(
                "Saved reference-view frame does not match registered depth"
            )
        return approach_geometry.resolve_reference_approach_direction(
            projected_point=projected,
            surface_normal=normal,
            controlled_frame_pose_object=(
                reference_view.controlled_frame_pose_object
            ),
            mode=approach_mode,
            surface_normal_unavailable_reason=normal_error,
        )

    @staticmethod
    def _surface_target(
        capture,
        approach,
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    ):
        return surface_target_geometry.resolve_reference_surface_target(
            approach_direction=approach,
            controlled_frame_pose_object=(
                capture.reference_view.controlled_frame_pose_object
            ),
            target_surface_distance_m=target_surface_distance_m,
            aligned_preapproach_distance_m=(
                aligned_preapproach_distance_m
            ),
        )

    def _capture(
        self,
        object_id: str,
        routine_id: str,
        reference_view_id: str,
    ) -> reference_repository.CapturedReferenceView:
        view_id = reference_view_id.strip()
        if not view_id:
            raise ValueError("Reference view ID must not be empty")
        captures = self.repository.load_reference_views(
            object_id,
            routine_id,
        )
        for capture in captures:
            if capture.reference_view.view_id == view_id:
                return capture
        raise LookupError(
            f"Unknown reference view: {object_id}/{routine_id}/{view_id}"
        )


__all__ = ["ProbeGeometryResult", "ProbeSetupGeometry"]
