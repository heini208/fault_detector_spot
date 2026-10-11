"""Managed, content-addressed collision meshes for the sensor registry."""

import os
from pathlib import Path
import re
import stat
from threading import Lock

from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_COLLISION_BYTES,
    MAX_MESH_CHUNK_BYTES,
    SensorHeadGeometry,
    validate_collision_mesh,
)


class SensorMeshStore:
    """Stage bounded uploads and publish only complete, validated STL assets."""

    def __init__(self, root_dir):
        self.root_dir = Path(root_dir)
        self._lock = Lock()

    def store_chunk(
        self, sha256: str, byte_count: int, offset: int, data: bytes,
    ) -> bool:
        """Append an upload chunk; return true when the asset is available.

        Offset zero starts or restarts an upload. Existing complete assets are
        verified and reused. An invalid final mesh discards the partial upload.
        """
        self._validate_description(sha256, byte_count)
        if (
            type(offset) is not int or offset < 0
            or not isinstance(data, bytes)
            or not 1 <= len(data) <= MAX_MESH_CHUNK_BYTES
            or offset + len(data) > byte_count
        ):
            raise ValueError("Invalid collision mesh chunk size or offset")
        with self._lock:
            self._check_directory()
            self.root_dir.mkdir(parents=True, exist_ok=True)
            asset = self.root_dir / f"{sha256}.stl"
            self._check_file(asset)
            if asset.exists():
                self._read_verified(sha256, byte_count)
                return True

            partial = self.root_dir / f"{sha256}.part"
            self._check_file(partial)
            if offset and (not partial.exists() or partial.stat().st_size != offset):
                raise ValueError(
                    "Collision upload offset does not match the partial file"
                )
            flags = os.O_RDWR | os.O_NOFOLLOW
            if offset == 0:
                flags |= os.O_CREAT | os.O_TRUNC
            complete = offset + len(data) == byte_count
            try:
                with os.fdopen(os.open(partial, flags, 0o600), "r+b") as stream:
                    stream.seek(offset)
                    stream.write(data)
                    if complete:
                        stream.flush()
                        os.fsync(stream.fileno())
                        stream.seek(0)
                        validate_collision_mesh(
                            stream.read(MAX_COLLISION_BYTES + 1), sha256, byte_count,
                        )
                if complete:
                    os.replace(partial, asset)
                return complete
            except Exception:
                partial.unlink(missing_ok=True)
                raise

    def read(self, geometry: SensorHeadGeometry) -> bytes:
        """Read and verify a managed asset referenced by a definition."""
        geometry.validate()
        with self._lock:
            self._check_directory()
            return self._read_verified(
                geometry.collision_sha256, geometry.collision_byte_count,
            )

    def _read_verified(self, sha256: str, byte_count: int) -> bytes:
        path = self.root_dir / f"{sha256}.stl"
        self._check_file(path)
        with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
            data = stream.read(MAX_COLLISION_BYTES + 1)
        validate_collision_mesh(data, sha256, byte_count)
        return data

    def _check_directory(self) -> None:
        if self.root_dir.is_symlink():
            raise ValueError("Collision asset directory must not be a symbolic link")
        if self.root_dir.exists() and not self.root_dir.is_dir():
            raise ValueError("Collision asset path must be a directory")

    @staticmethod
    def _check_file(path: Path) -> None:
        try:
            mode = path.lstat().st_mode
        except FileNotFoundError:
            return
        if not stat.S_ISREG(mode):
            raise ValueError("Collision asset must be a regular file, not a link")

    @staticmethod
    def _validate_description(sha256: str, byte_count: int) -> None:
        if not isinstance(sha256, str) or re.fullmatch(r"[0-9a-f]{64}", sha256) is None:
            raise ValueError("Collision checksum must be a lowercase SHA-256 digest")
        if type(byte_count) is not int or not 84 <= byte_count <= MAX_COLLISION_BYTES:
            raise ValueError("Collision byte count must be between 84 bytes and 32 MiB")
