"""Resolve one saved reference pixel into probe setup geometry."""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from fault_detector_spot.inspection.geometry import (
    surface_normal as surface_normal_geometry,
)
from fault_detector_spot.inspection.geometry.rotation import (
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.repository import (
    multi_reference_view_repository as reference_repository,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    validate_surface_distance_pair,
)
from fault_detector_spot.inspection.setup.alignment_orientation import (
    surface_aligned_probe_orientation,
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
    reference_view_surface_target as surface_target_geometry,
)


@dataclass(frozen=True)
class ProbeGeometryResult:
    """Contain one deterministic reference-pixel calculation."""

    capture: reference_repository.CapturedReferenceView
    projected_point: depth_projection.ProjectedReferencePoint
    surface_normal: Optional[surface_normal_geometry.SurfaceNormalEstimate]
    surface_normal_error: str
    approach_direction: approach_geometry.ReferenceApproachDirection
    surface_target: surface_target_geometry.ReferenceSurfaceTarget
    probe_setup: ReferenceProbeSetup


class ProbeSetupGeometry:
    """Load reference data and derive probe poses from surface geometry."""

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
        reference_view = capture.reference_view
        if reference_view.controlled_frame != projected.frame_id:
            raise ValueError(
                "Saved reference-view frame does not match registered depth"
            )

        outward_camera, direction_source = (
            self._resolve_outward_camera_direction(
                projected_point=projected,
                surface_normal=normal,
                controlled_frame_pose_object=(
                    reference_view.controlled_frame_pose_object
                ),
                mode=approach_mode,
                surface_normal_unavailable_reason=normal_error,
            )
        )
        target = self._build_surface_target(
            projected_point=projected,
            outward_direction_camera=outward_camera,
            controlled_frame_pose_object=(
                reference_view.controlled_frame_pose_object
            ),
            target_surface_distance_m=target_surface_distance_m,
            aligned_preapproach_distance_m=(
                aligned_preapproach_distance_m
            ),
            hand_to_probe_pose=hand_to_probe_pose,
            direction_source=direction_source,
        )
        setup = initialize_reference_probe_setup(target)
        approach = approach_geometry.ReferenceApproachDirection(
            projected_point=projected,
            direction_camera=outward_camera,
            source=direction_source,
            surface_normal=(
                normal
                if direction_source
                == approach_geometry.APPROACH_SOURCE_SURFACE_FIT
                else None
            ),
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
            normal = surface_normal_geometry.estimate_surface_normal(
                projected,
                capture.depth_image,
                capture.depth_camera_info,
            )
        except ValueError as exception:
            normal_error = str(exception)
        return projected, normal, normal_error

    @staticmethod
    def _resolve_outward_camera_direction(
        projected_point: depth_projection.ProjectedReferencePoint,
        surface_normal: Optional[
            surface_normal_geometry.SurfaceNormalEstimate
        ],
        controlled_frame_pose_object: PoseData,
        mode: str = approach_geometry.APPROACH_MODE_AUTOMATIC,
        surface_normal_unavailable_reason: str = "",
    ) -> tuple[Vector3Data, str]:
        if projected_point is None:
            raise ValueError("No projected surface point is available")
        projected_point.requested_pixel.validate()
        projected_point.point_camera.validate()
        if mode not in approach_geometry.VALID_APPROACH_MODES:
            raise ValueError(
                f"Unsupported approach-direction mode: {mode}"
            )

        if mode == approach_geometry.APPROACH_MODE_TAG_X:
            direction = ProbeSetupGeometry._object_positive_x_in_camera(
                controlled_frame_pose_object
            )
            return (
                ProbeSetupGeometry._vector_data(direction),
                approach_geometry.APPROACH_SOURCE_TAG_X_SELECTED,
            )

        if surface_normal is None:
            reason = surface_normal_unavailable_reason.strip()
            if not reason:
                reason = "No valid local surface normal is available"
            raise ValueError(
                f"Surface-fit approach direction is unavailable: {reason}"
            )

        ProbeSetupGeometry._validate_surface_normal_for_point(
            projected_point,
            surface_normal,
        )
        normal = surface_normal.normal_camera
        normal.validate()
        direction = ProbeSetupGeometry._normalized_vector(
            [normal.x, normal.y, normal.z],
            "Surface normal",
        )
        return (
            ProbeSetupGeometry._vector_data(direction),
            approach_geometry.APPROACH_SOURCE_SURFACE_FIT,
        )

    @staticmethod
    def _build_surface_target(
        projected_point: depth_projection.ProjectedReferencePoint,
        outward_direction_camera: Vector3Data,
        controlled_frame_pose_object: PoseData,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
        hand_to_probe_pose: PoseData,
        direction_source: str,
    ) -> surface_target_geometry.ReferenceSurfaceTarget:
        if projected_point is None:
            raise ValueError("No projected surface point is available")
        projected_point.point_camera.validate()
        outward_direction_camera.validate()
        controlled_frame_pose_object.validate()
        hand_to_probe_pose.validate()
        validate_surface_distance_pair(
            target_surface_distance_m,
            aligned_preapproach_distance_m,
        )

        rotation_object_camera = rotation_from_quaternion(
            controlled_frame_pose_object.orientation
        )
        camera_origin_object = ProbeSetupGeometry._vector_array(
            controlled_frame_pose_object.position,
            "Reference camera position",
        )
        surface_point_camera = ProbeSetupGeometry._vector_array(
            projected_point.point_camera,
            "Selected surface point",
        )
        outward_camera = ProbeSetupGeometry._vector_array(
            outward_direction_camera,
            "Surface outward direction",
        )

        surface_point_object_array = (
            camera_origin_object
            + rotation_object_camera.apply(surface_point_camera)
        )
        outward_object_array = ProbeSetupGeometry._normalized_vector(
            rotation_object_camera.apply(outward_camera),
            "Object-frame surface outward direction",
        )

        # These are probe-frame origins. The motion planner applies the full
        # inverse hand-to-probe transform when deriving the hand target.
        target_position = (
            surface_point_object_array
            + outward_object_array * target_surface_distance_m
        )
        preapproach_position = (
            surface_point_object_array
            + outward_object_array * aligned_preapproach_distance_m
        )

        surface_point_object = ProbeSetupGeometry._vector_data(
            surface_point_object_array
        )
        outward_direction_object = ProbeSetupGeometry._vector_data(
            outward_object_array
        )
        # Saved reference views have no gravity pose; object +Z resolves roll.
        # Live alignment uses measured gravity expressed in the object frame.
        orientation = surface_aligned_probe_orientation(
            outward_direction_object,
            hand_to_probe_pose.orientation,
            Vector3Data(x=0.0, y=0.0, z=1.0),
        )
        target_pose = PoseData(
            position=ProbeSetupGeometry._vector_data(target_position),
            orientation=orientation,
        )
        preapproach_pose = PoseData(
            position=ProbeSetupGeometry._vector_data(preapproach_position),
            orientation=orientation,
        )

        surface_point_object.validate()
        outward_direction_object.validate()
        target_pose.validate()
        preapproach_pose.validate()

        return surface_target_geometry.ReferenceSurfaceTarget(
            surface_point_object=surface_point_object,
            outward_direction_object=outward_direction_object,
            target_pose_object=target_pose,
            aligned_preapproach_pose_object=preapproach_pose,
            target_surface_distance_m=float(target_surface_distance_m),
            aligned_preapproach_distance_m=float(
                aligned_preapproach_distance_m
            ),
            direction_source=direction_source,
        )

    @staticmethod
    def _validate_surface_normal_for_point(
        projected_point: depth_projection.ProjectedReferencePoint,
        surface_normal: surface_normal_geometry.SurfaceNormalEstimate,
    ) -> None:
        normal_point = surface_normal.projected_point
        if normal_point.frame_id != projected_point.frame_id:
            raise ValueError(
                "Surface normal and projected point frames do not match"
            )
        if normal_point.requested_pixel != projected_point.requested_pixel:
            raise ValueError(
                "Surface normal belongs to a different reference pixel"
            )

    @staticmethod
    def _object_positive_x_in_camera(
        controlled_frame_pose_object: PoseData,
    ) -> np.ndarray:
        controlled_frame_pose_object.validate()
        rotation_object_camera = rotation_from_quaternion(
            controlled_frame_pose_object.orientation
        )
        direction = rotation_object_camera.inv().apply(
            np.array([1.0, 0.0, 0.0], dtype=float)
        )
        return ProbeSetupGeometry._normalized_vector(
            direction,
            "Tag-frame +X direction",
        )

    @staticmethod
    def _vector_array(
        vector: Vector3Data,
        label: str,
    ) -> np.ndarray:
        vector.validate()
        return ProbeSetupGeometry._shape_array(
            [vector.x, vector.y, vector.z],
            label,
        )

    @staticmethod
    def _shape_array(values, label: str) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise ValueError(
                f"{label} must contain three finite values"
            )
        return array

    @staticmethod
    def _normalized_vector(values, label: str) -> np.ndarray:
        array = ProbeSetupGeometry._shape_array(values, label)
        norm = float(np.linalg.norm(array))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError(f"{label} cannot be normalized")
        return array / norm

    @staticmethod
    def _vector_data(values) -> Vector3Data:
        array = ProbeSetupGeometry._shape_array(values, "Vector")
        return Vector3Data(
            x=float(array[0]),
            y=float(array[1]),
            z=float(array[2]),
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
