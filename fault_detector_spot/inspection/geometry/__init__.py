"""Geometry adapters backed by external 3D libraries."""

from .rotation import (
    quaternion_from_matrix,
    quaternion_from_rotation,
    quaternion_to_rpy,
    rotate_vector,
    rotation_from_quaternion,
)
from .surface_plane import SurfacePlane, fit_surface_plane

__all__ = [
    "SurfacePlane",
    "fit_surface_plane",
    "quaternion_from_matrix",
    "quaternion_from_rotation",
    "quaternion_to_rpy",
    "rotate_vector",
    "rotation_from_quaternion",
]
