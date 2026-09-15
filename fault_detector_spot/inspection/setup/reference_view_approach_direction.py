"""Approach-direction model for probe setup geometry."""

from dataclasses import dataclass
from typing import Optional

from fault_detector_spot.inspection.geometry.surface_normal import (
    SurfaceNormalEstimate,
)
from fault_detector_spot.inspection.model.models import Vector3Data
from .reference_view_depth_projection import ProjectedReferencePoint


APPROACH_MODE_AUTOMATIC = "automatic"
APPROACH_MODE_SURFACE_FIT = "surface_fit"
APPROACH_MODE_TAG_X = "tag_x"

APPROACH_SOURCE_SURFACE_FIT = "surface_fit"
APPROACH_SOURCE_TAG_X_SELECTED = "tag_x_selected"

VALID_APPROACH_MODES = {
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
}


@dataclass(frozen=True)
class ReferenceApproachDirection:
    """Camera-frame direction pointing outward from the inspected object."""

    projected_point: ProjectedReferencePoint
    direction_camera: Vector3Data
    source: str
    surface_normal: Optional[SurfaceNormalEstimate] = None


__all__ = [
    "APPROACH_MODE_AUTOMATIC",
    "APPROACH_MODE_SURFACE_FIT",
    "APPROACH_MODE_TAG_X",
    "APPROACH_SOURCE_SURFACE_FIT",
    "APPROACH_SOURCE_TAG_X_SELECTED",
    "ReferenceApproachDirection",
    "VALID_APPROACH_MODES",
]
