"""Project selected RGB reference pixels through registered depth."""

import copy
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.open3d_depth import (
    OrganizedDepthPointCloud,
    camera_intrinsics,
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.model.models import ImagePoint, Vector3Data


class RegisteredDepthSupportNotReady(ValueError):
    """Indicate that a later depth frame may provide usable support."""


@dataclass(frozen=True)
class ProjectedReferencePoint:
    """One selected surface point expressed in the depth optical frame."""

    requested_pixel: ImagePoint
    sampled_pixel: ImagePoint
    point_camera: Vector3Data
    frame_id: str
    depth_m: float
    mapped_pixel: Optional[ImagePoint] = None

    def __post_init__(self):
        if self.mapped_pixel is None:
            object.__setattr__(self, "mapped_pixel", self.requested_pixel)


@dataclass(frozen=True)
class ImageRegion:
    """Inclusive source-image region covered by registered depth."""

    x: int
    y: int
    width: int
    height: int

    def validate(self) -> None:
        values = (self.x, self.y, self.width, self.height)
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in values
        ):
            raise ValueError("Image region values must be integers")
        if self.x < 0 or self.y < 0:
            raise ValueError("Image region origin must not be negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Image region dimensions must be positive")

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "width": self.width,
            "height": self.height,
        }


def project_reference_pixel(
    pixel: ImagePoint,
    depth_image: Image,
    depth_camera_info: CameraInfo,
    search_radius_px: int = 2,
    rgb_size: Optional[Tuple[int, int]] = None,
    rgb_camera_info: Optional[CameraInfo] = None,
) -> ProjectedReferencePoint:
    """Map an RGB ray into registered depth and project it with Open3D."""
    if pixel is None:
        raise ValueError("No reference pixel is selected")
    pixel.validate()
    _validate_search_radius(search_radius_px)
    depth_size = (depth_image.width, depth_image.height)
    resolved_rgb_size = _resolve_rgb_size(rgb_size, depth_image)
    _validate_pixel_bounds(pixel, resolved_rgb_size, "RGB reference image")
    mapped_pixel = map_rgb_pixel_to_depth(
        pixel,
        resolved_rgb_size,
        depth_size,
        rgb_camera_info=rgb_camera_info,
        depth_camera_info=(
            depth_camera_info if rgb_camera_info is not None else None
        ),
    )
    frame_id = _resolve_frame_id(depth_image, depth_camera_info)
    point_cloud = create_organized_depth_point_cloud(
        depth_image,
        depth_camera_info,
    )
    sampled_pixel, depth_m = _nearest_valid_depth(
        mapped_pixel,
        point_cloud,
        search_radius_px,
    )
    point = point_cloud.point_camera(sampled_pixel.u, sampled_pixel.v)
    return ProjectedReferencePoint(
        requested_pixel=ImagePoint(u=pixel.u, v=pixel.v),
        mapped_pixel=mapped_pixel,
        sampled_pixel=sampled_pixel,
        point_camera=Vector3Data(
            x=float(point[0]),
            y=float(point[1]),
            z=float(point[2]),
        ),
        frame_id=frame_id,
        depth_m=depth_m,
    )


def map_rgb_pixel_to_depth(
    pixel: ImagePoint,
    rgb_size: Tuple[int, int],
    depth_size: Tuple[int, int],
    rgb_camera_info: Optional[CameraInfo] = None,
    depth_camera_info: Optional[CameraInfo] = None,
) -> ImagePoint:
    """Map one RGB ray into the registered-depth raster."""
    pixel.validate()
    rgb_width, rgb_height = _validate_image_size(
        rgb_size,
        "RGB image size",
    )
    depth_width, depth_height = _validate_image_size(
        depth_size,
        "Depth image size",
    )
    _validate_pixel_bounds(
        pixel,
        (rgb_width, rgb_height),
        "RGB reference image",
    )

    if (rgb_camera_info is None) != (depth_camera_info is None):
        raise ValueError("RGB and depth CameraInfo must be provided together")
    if rgb_camera_info is None:
        if (rgb_width, rgb_height) != (depth_width, depth_height):
            raise ValueError(
                "Different RGB and registered-depth resolutions require "
                "both CameraInfo messages"
            )
        return ImagePoint(u=pixel.u, v=pixel.v)

    rgb_model = _pinhole_camera_model(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_model = _pinhole_camera_model(
        depth_camera_info,
        (depth_width, depth_height),
        "Depth CameraInfo",
    )
    mapped = _map_registered_ray(pixel, rgb_model, depth_model)
    if not (
        0 <= mapped.u < depth_width
        and 0 <= mapped.v < depth_height
    ):
        raise ValueError(
            "Reference pixel is outside the registered-depth field of view"
        )
    return mapped


def rgb_depth_overlap_region(
    rgb_size: Tuple[int, int],
    depth_size: Tuple[int, int],
    rgb_camera_info: CameraInfo,
    depth_camera_info: CameraInfo,
) -> ImageRegion:
    """Return the RGB region whose rays land inside registered depth."""
    rgb_width, rgb_height = _validate_image_size(
        rgb_size,
        "RGB image size",
    )
    depth_width, depth_height = _validate_image_size(
        depth_size,
        "Depth image size",
    )
    rgb_model = _pinhole_camera_model(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_model = _pinhole_camera_model(
        depth_camera_info,
        (depth_width, depth_height),
        "Depth CameraInfo",
    )
    valid_u = [
        u
        for u in range(rgb_width)
        if _continuous_coordinate_in_raster(
            _project_registered_ray(
                ImagePoint(u=u, v=0),
                rgb_model,
                depth_model,
            )[0],
            depth_width,
        )
    ]
    valid_v = [
        v
        for v in range(rgb_height)
        if _continuous_coordinate_in_raster(
            _project_registered_ray(
                ImagePoint(u=0, v=v),
                rgb_model,
                depth_model,
            )[1],
            depth_height,
        )
    ]
    if not valid_u or not valid_v:
        raise ValueError(
            "RGB and registered depth CameraInfo have no common field of view"
        )
    region = ImageRegion(
        x=valid_u[0],
        y=valid_v[0],
        width=valid_u[-1] - valid_u[0] + 1,
        height=valid_v[-1] - valid_v[0] + 1,
    )
    region.validate()
    return region


def rgb_depth_selectable_region(
    rgb_size: Tuple[int, int],
    depth_image: Image,
    rgb_camera_info: CameraInfo,
    depth_camera_info: CameraInfo,
) -> ImageRegion:
    """Return the RGB region backed by calibrated, non-empty depth."""
    rgb_width, rgb_height = _validate_image_size(
        rgb_size,
        "RGB image size",
    )
    depth_size = (depth_image.width, depth_image.height)
    depth_width, depth_height = _validate_image_size(
        depth_size,
        "Depth image size",
    )
    rgb_model = _pinhole_camera_model(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_model = _pinhole_camera_model(
        depth_camera_info,
        (depth_width, depth_height),
        "Depth CameraInfo",
    )
    point_cloud = create_organized_depth_point_cloud(
        depth_image,
        depth_camera_info,
    )
    depth_support = _depth_valid_support_region(point_cloud)
    valid_u = [
        u
        for u in range(rgb_width)
        if _continuous_coordinate_in_region(
            _project_registered_ray(
                ImagePoint(u=u, v=0),
                rgb_model,
                depth_model,
            )[0],
            depth_support.x,
            depth_support.width,
        )
    ]
    valid_v = [
        v
        for v in range(rgb_height)
        if _continuous_coordinate_in_region(
            _project_registered_ray(
                ImagePoint(u=0, v=v),
                rgb_model,
                depth_model,
            )[1],
            depth_support.y,
            depth_support.height,
        )
    ]
    if not valid_u or not valid_v:
        raise RegisteredDepthSupportNotReady(
            "RGB image has no selectable region with valid registered depth"
        )
    region = ImageRegion(
        x=valid_u[0],
        y=valid_v[0],
        width=valid_u[-1] - valid_u[0] + 1,
        height=valid_v[-1] - valid_v[0] + 1,
    )
    region.validate()
    return region


def _pinhole_camera_model(
    camera_info: CameraInfo,
    image_size: Tuple[int, int],
    label: str,
):
    camera_intrinsics(camera_info, image_size, label)
    try:
        from image_geometry import PinholeCameraModel
    except ImportError as exception:
        raise RuntimeError(
            "ROS image_geometry is required for registered RGB/depth mapping"
        ) from exception
    normalized = copy.deepcopy(camera_info)
    if not any(abs(float(value)) > 1e-12 for value in normalized.p):
        normalized.p = [
            normalized.k[0],
            normalized.k[1],
            normalized.k[2],
            0.0,
            normalized.k[3],
            normalized.k[4],
            normalized.k[5],
            0.0,
            normalized.k[6],
            normalized.k[7],
            normalized.k[8],
            0.0,
        ]
    model = PinholeCameraModel()
    model.fromCameraInfo(normalized)
    return model


def _project_registered_ray(pixel, source_model, target_model):
    ray = source_model.projectPixelTo3dRay(
        (float(pixel.u), float(pixel.v))
    )
    projected = target_model.project3dToPixel(ray)
    u = float(projected[0])
    v = float(projected[1])
    if not math.isfinite(u) or not math.isfinite(v):
        raise ValueError(
            "Registered-depth projection produced a non-finite pixel"
        )
    return u, v


def _map_registered_ray(pixel, source_model, target_model) -> ImagePoint:
    projected_u, projected_v = _project_registered_ray(
        pixel,
        source_model,
        target_model,
    )
    return ImagePoint(
        u=_round_pixel_coordinate(projected_u),
        v=_round_pixel_coordinate(projected_v),
    )


def _round_pixel_coordinate(value: float) -> int:
    tolerance = 1e-9 * max(1.0, abs(value))
    return int(math.floor(value + 0.5 + tolerance))


def _continuous_coordinate_in_raster(value: float, length: int) -> bool:
    tolerance = 1e-9 * max(1.0, abs(value), float(length))
    return (
        value >= -0.5 - tolerance
        and value < float(length) - 0.5 - tolerance
    )


def _continuous_coordinate_in_region(
    value: float,
    start: int,
    length: int,
) -> bool:
    tolerance = 1e-9 * max(
        1.0,
        abs(value),
        float(start + length),
    )
    lower = float(start) - 0.5
    upper = float(start + length) - 0.5
    return value >= lower - tolerance and value < upper - tolerance


def _depth_valid_support_region(
    point_cloud: OrganizedDepthPointCloud,
) -> ImageRegion:
    row_counts = np.count_nonzero(point_cloud.valid_mask, axis=1)
    column_counts = np.count_nonzero(point_cloud.valid_mask, axis=0)
    minimum_column_count = max(
        1,
        int(math.ceil(point_cloud.height * 0.01)),
    )
    minimum_row_count = max(
        1,
        int(math.ceil(point_cloud.width * 0.01)),
    )
    supported_columns = np.flatnonzero(
        column_counts >= minimum_column_count
    )
    supported_rows = np.flatnonzero(
        row_counts >= minimum_row_count
    )
    if supported_columns.size == 0 or supported_rows.size == 0:
        raise RegisteredDepthSupportNotReady(
            "Registered depth image does not contain valid depth support"
        )
    region = ImageRegion(
        x=int(supported_columns[0]),
        y=int(supported_rows[0]),
        width=int(supported_columns[-1] - supported_columns[0] + 1),
        height=int(supported_rows[-1] - supported_rows[0] + 1),
    )
    region.validate()
    return region


def _resolve_rgb_size(
    rgb_size: Optional[Tuple[int, int]],
    depth_image: Image,
) -> Tuple[int, int]:
    if rgb_size is None:
        return depth_image.width, depth_image.height
    return _validate_image_size(rgb_size, "RGB image size")


def _validate_image_size(
    size: Tuple[int, int],
    label: str,
) -> Tuple[int, int]:
    if not isinstance(size, (tuple, list)) or len(size) != 2:
        raise ValueError(f"{label} must contain width and height")
    width, height = size
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
    ):
        raise ValueError(f"{label} dimensions must be positive integers")
    return width, height


def _validate_search_radius(search_radius_px: int) -> None:
    if (
        isinstance(search_radius_px, bool)
        or not isinstance(search_radius_px, int)
        or search_radius_px < 0
    ):
        raise ValueError("Depth search radius must be a non-negative integer")


def _validate_pixel_bounds(
    pixel: ImagePoint,
    size: Tuple[int, int],
    label: str,
) -> None:
    width, height = size
    if pixel.u >= width or pixel.v >= height:
        raise ValueError(f"Reference pixel is outside the {label}")


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
    point_cloud: OrganizedDepthPointCloud,
    search_radius_px: int,
) -> Tuple[ImagePoint, float]:
    for u, v in _candidate_pixels(
        requested_pixel,
        point_cloud.width,
        point_cloud.height,
        search_radius_px,
    ):
        if not bool(point_cloud.valid_mask[v, u]):
            continue
        return ImagePoint(u=u, v=v), float(point_cloud.depth_m[v, u])
    raise ValueError(
        f"No valid depth within {search_radius_px} px of depth pixel "
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
