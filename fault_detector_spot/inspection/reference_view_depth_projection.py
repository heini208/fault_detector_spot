"""Project selected reference-image pixels through registered depth."""

import math
import struct
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from sensor_msgs.msg import CameraInfo, Image

from .models import ImagePoint, Vector3Data


@dataclass(frozen=True)
class ProjectedReferencePoint:
    """One selected surface point expressed in the depth optical frame."""

    requested_pixel: ImagePoint
    sampled_pixel: ImagePoint
    point_camera: Vector3Data
    frame_id: str
    depth_m: float


def project_reference_pixel(
    pixel: ImagePoint,
    depth_image: Image,
    camera_info: CameraInfo,
    search_radius_px: int = 2,
) -> ProjectedReferencePoint:
    """Resolve a selected reference pixel into a camera-frame point."""
    if pixel is None:
        raise ValueError("No reference pixel is selected")
    pixel.validate()
    _validate_search_radius(search_radius_px)
    _validate_depth_image(depth_image)
    fx, fy, cx, cy = _camera_intrinsics(camera_info, depth_image)
    _validate_pixel_bounds(pixel, depth_image)
    frame_id = _resolve_frame_id(depth_image, camera_info)

    sampled_pixel, depth_m = _nearest_valid_depth(
        pixel,
        depth_image,
        search_radius_px,
    )
    x_m = (pixel.u - cx) * depth_m / fx
    y_m = (pixel.v - cy) * depth_m / fy
    return ProjectedReferencePoint(
        requested_pixel=ImagePoint(u=pixel.u, v=pixel.v),
        sampled_pixel=sampled_pixel,
        point_camera=Vector3Data(x=x_m, y=y_m, z=depth_m),
        frame_id=frame_id,
        depth_m=depth_m,
    )


def _validate_search_radius(search_radius_px: int) -> None:
    if (
        isinstance(search_radius_px, bool)
        or not isinstance(search_radius_px, int)
        or search_radius_px < 0
    ):
        raise ValueError("Depth search radius must be a non-negative integer")


def _validate_depth_image(depth_image: Image) -> None:
    if depth_image.width <= 0 or depth_image.height <= 0:
        raise ValueError("Depth image dimensions must be positive")
    encoding = depth_image.encoding.strip().lower()
    bytes_per_pixel = {"16uc1": 2, "32fc1": 4}.get(encoding)
    if bytes_per_pixel is None:
        raise ValueError(
            f"Unsupported depth encoding: {depth_image.encoding}"
        )
    minimum_step = depth_image.width * bytes_per_pixel
    if depth_image.step < minimum_step:
        raise ValueError("Depth image step is too small for its encoding")
    if len(depth_image.data) < depth_image.step * depth_image.height:
        raise ValueError("Depth image data is shorter than expected")


def _camera_intrinsics(
    camera_info: CameraInfo,
    depth_image: Image,
) -> Tuple[float, float, float, float]:
    if len(camera_info.k) != 9:
        raise ValueError("Camera matrix must contain nine values")
    if camera_info.width not in (0, depth_image.width):
        raise ValueError("Camera info width does not match depth image")
    if camera_info.height not in (0, depth_image.height):
        raise ValueError("Camera info height does not match depth image")
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError("Camera intrinsics contain a non-finite value")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("Camera focal lengths must be positive")
    return fx, fy, cx, cy


def _validate_pixel_bounds(pixel: ImagePoint, depth_image: Image) -> None:
    if pixel.u >= depth_image.width or pixel.v >= depth_image.height:
        raise ValueError("Reference pixel is outside the depth image")


def _resolve_frame_id(
    depth_image: Image,
    camera_info: CameraInfo,
) -> str:
    depth_frame = depth_image.header.frame_id.strip()
    camera_frame = camera_info.header.frame_id.strip()
    if depth_frame and camera_frame and depth_frame != camera_frame:
        raise ValueError("Depth image and camera info frames do not match")
    frame_id = depth_frame or camera_frame
    if not frame_id:
        raise ValueError("Depth projection frame is empty")
    return frame_id


def _nearest_valid_depth(
    requested_pixel: ImagePoint,
    depth_image: Image,
    search_radius_px: int,
) -> Tuple[ImagePoint, float]:
    data = memoryview(depth_image.data)
    for u, v in _candidate_pixels(
        requested_pixel,
        depth_image.width,
        depth_image.height,
        search_radius_px,
    ):
        depth_m = _read_depth_m(depth_image, data, u, v)
        if depth_m is not None:
            return ImagePoint(u=u, v=v), depth_m
    raise ValueError(
        f"No valid depth within {search_radius_px} px of "
        f"u={requested_pixel.u}, v={requested_pixel.v}"
    )


def _candidate_pixels(
    requested_pixel: ImagePoint,
    width: int,
    height: int,
    radius: int,
) -> Iterable[Tuple[int, int]]:
    candidates = []
    for v in range(
        max(0, requested_pixel.v - radius),
        min(height, requested_pixel.v + radius + 1),
    ):
        for u in range(
            max(0, requested_pixel.u - radius),
            min(width, requested_pixel.u + radius + 1),
        ):
            distance_squared = (
                (u - requested_pixel.u) ** 2
                + (v - requested_pixel.v) ** 2
            )
            if distance_squared <= radius ** 2:
                candidates.append((distance_squared, v, u))
    for _, v, u in sorted(candidates):
        yield u, v


def _read_depth_m(
    depth_image: Image,
    data: memoryview,
    u: int,
    v: int,
) -> Optional[float]:
    encoding = depth_image.encoding.strip().lower()
    if encoding == "16uc1":
        offset = v * depth_image.step + u * 2
        byteorder = "big" if depth_image.is_bigendian else "little"
        raw_value = int.from_bytes(
            data[offset:offset + 2],
            byteorder=byteorder,
            signed=False,
        )
        if raw_value == 0:
            return None
        return raw_value * 0.001

    offset = v * depth_image.step + u * 4
    prefix = ">" if depth_image.is_bigendian else "<"
    raw_value = struct.unpack_from(f"{prefix}f", data, offset)[0]
    if not math.isfinite(raw_value) or raw_value <= 0.0:
        return None
    return float(raw_value)
