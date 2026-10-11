"""Collision assets and placement metadata for a sensor mount."""

from dataclasses import dataclass
import hashlib
import math
import re
import struct
from typing import Any, Dict, Optional

import numpy as np

from fault_detector_spot.shared.geometry.models import PoseData


MAX_COLLISION_BYTES = 32 * 1024 * 1024
MAX_MESH_CHUNK_BYTES = 48 * 1024
MAX_MESH_TRIANGLES = 200_000
MOUNTING_TEMPLATE_ID = "spot-hand-reference-v1"
MOUNTING_TEMPLATE_REVISION = 1
REFERENCE_MODEL_FINGERPRINT = (
    "a5746b4090dafe81c140c62d69be15dc682d5835d5384c6bbda8592e53391f46"
)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_STL_TRIANGLE_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])


@dataclass(frozen=True)
class SensorHeadGeometry:
    """Reference a managed collision mesh and its pose relative to the hand."""

    collision_sha256: str
    collision_byte_count: int
    hand_to_geometry: PoseData
    mounting_template_id: str
    mounting_template_revision: int
    reference_model_fingerprint: str
    mounting_template_verified: bool
    clearance_m: float = 0.0
    configuration_method: str = "manual"

    @property
    def asset_path(self) -> str:
        """Return the content-addressed path relative to the sensor registry."""
        return f"assets/{self.collision_sha256}.stl"

    def validate(self) -> None:
        """Validate supported mount metadata without requiring physical approval."""
        if (
            not isinstance(self.collision_sha256, str)
            or _SHA256_PATTERN.fullmatch(self.collision_sha256) is None
        ):
            raise ValueError("Collision checksum must be a lowercase SHA-256 digest")
        if (
            type(self.collision_byte_count) is not int
            or not 84 <= self.collision_byte_count <= MAX_COLLISION_BYTES
        ):
            raise ValueError("Collision byte count must be between 84 bytes and 32 MiB")
        if not isinstance(self.hand_to_geometry, PoseData):
            raise ValueError("Collision placement must be a pose")
        self.hand_to_geometry.validate()
        if (
            self.mounting_template_id != MOUNTING_TEMPLATE_ID
            or type(self.mounting_template_revision) is not int
            or self.mounting_template_revision != MOUNTING_TEMPLATE_REVISION
            or self.reference_model_fingerprint != REFERENCE_MODEL_FINGERPRINT
        ):
            raise ValueError("Unsupported sensor-head mounting template or reference model")
        if type(self.mounting_template_verified) is not bool:
            raise ValueError("Mounting template verification must be a boolean")
        if (
            type(self.clearance_m) not in (int, float)
            or not math.isfinite(self.clearance_m)
            or self.clearance_m != 0
        ):
            raise ValueError("Only zero collision clearance is currently supported")
        if self.configuration_method != "manual":
            raise ValueError("Unsupported sensor-head configuration method")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize metadata; mesh bytes are stored separately in the registry."""
        return {
            "collision_sha256": self.collision_sha256,
            "collision_byte_count": self.collision_byte_count,
            "hand_to_geometry": self.hand_to_geometry.to_dict(),
            "mounting_template_id": self.mounting_template_id,
            "mounting_template_revision": self.mounting_template_revision,
            "reference_model_fingerprint": self.reference_model_fingerprint,
            "mounting_template_verified": self.mounting_template_verified,
            "clearance_m": self.clearance_m,
            "configuration_method": self.configuration_method,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SensorHeadGeometry":
        """Restore metadata from a persisted sensor definition."""
        if not isinstance(data, dict):
            raise ValueError("Sensor head geometry must be an object")
        geometry = cls(
            collision_sha256=data["collision_sha256"],
            collision_byte_count=data["collision_byte_count"],
            hand_to_geometry=PoseData.from_dict(data["hand_to_geometry"]),
            mounting_template_id=data["mounting_template_id"],
            mounting_template_revision=data["mounting_template_revision"],
            reference_model_fingerprint=data["reference_model_fingerprint"],
            mounting_template_verified=data["mounting_template_verified"],
            clearance_m=data.get("clearance_m", 0.0),
            configuration_method=data.get("configuration_method", "manual"),
        )
        geometry.validate()
        return geometry


def validate_collision_mesh(
    data: bytes,
    expected_sha256: str = "",
    expected_byte_count: Optional[int] = None,
) -> None:
    """Check binary STL integrity, closed winding and nonzero component volumes.

    Coordinates are in metres. Touching closed parts are supported; intersections
    between parts are not checked. The source mesh is never repaired or rescaled.
    """
    if not isinstance(data, bytes) or not 84 <= len(data) <= MAX_COLLISION_BYTES:
        raise ValueError("Collision mesh must be binary STL between 84 bytes and 32 MiB")
    if expected_byte_count is not None and (
        type(expected_byte_count) is not int or len(data) != expected_byte_count
    ):
        raise ValueError("Collision mesh byte count does not match its metadata")
    if expected_sha256 and hashlib.sha256(data).hexdigest() != expected_sha256:
        raise ValueError("Collision mesh SHA-256 checksum does not match its metadata")
    count = struct.unpack_from("<I", data, 80)[0]
    if not 1 <= count <= MAX_MESH_TRIANGLES or len(data) != 84 + count * 50:
        raise ValueError("Collision STL triangle count or file length is invalid")
    records = np.frombuffer(data, dtype=_STL_TRIANGLE_DTYPE, offset=84)
    triangles = records["vertices"].astype(float)
    if not np.isfinite(triangles).all() or not np.isfinite(records["normal"]).all():
        raise ValueError("Collision mesh coordinates and normals must be finite")
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    if (np.linalg.norm(normals, axis=1) == 0).any():
        raise ValueError("Collision mesh contains degenerate triangles")
    vertices, vertex_ids = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = vertex_ids.reshape(-1, 3)
    edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    _, edge_ids = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True)
    directions = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    # A shared edge may belong to several touching closed parts.
    if (np.bincount(edge_ids, weights=directions) != 0).any():
        raise ValueError("Collision mesh must be closed with consistent triangle winding")
    _validate_component_volumes(triangles, vertices, faces)


def _validate_component_volumes(triangles, vertices, faces) -> None:
    """Group touching triangles without a graph dependency and check each volume."""
    parents = np.arange(len(vertices))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for a, b, c in faces:
        parent = root(a)
        parents[root(b)] = parent
        parents[root(c)] = parent
    roots = np.array([root(face[0]) for face in faces])
    _, components = np.unique(roots, return_inverse=True)
    # Use a nearby origin to avoid cancellation for meshes far from the origin.
    local = triangles - vertices[roots, None, :]
    volumes = np.einsum("ij,ij->i", local[:, 0], np.cross(local[:, 1], local[:, 2])) / 6
    signed = np.bincount(components, weights=volumes)
    magnitude = np.bincount(components, weights=np.abs(volumes))
    if (np.abs(signed) <= magnitude * 1e-12).any():
        raise ValueError("Each collision mesh component must enclose nonzero volume")
