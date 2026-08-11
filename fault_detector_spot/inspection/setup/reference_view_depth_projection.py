"""Project selected RGB reference pixels through registered depth."""

import math
import struct
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from sensor_msgs.msg import CameraInfo, Image

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
        """Validate a positive image region."""
        values = (self.x, self.y, self.width, self.height)
        if any(isinstance(value, bool) or not isinstance(value, int)
               for value in values):
            raise ValueError("Image region values must be integers")
        if self.x < 0 or self.y < 0:
            raise ValueError("Image region origin must not be negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Image region dimensions must be positive")

    def to_dict(self):
        """Serialize the image region."""
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
    """Map an RGB pixel into registered depth and back-project it."""
    if pixel is None:
        raise ValueError("No reference pixel is selected")
    pixel.validate()
    _validate_search_radius(search_radius_px)
    _validate_depth_image(depth_image)
    depth_size = (depth_image.width, depth_image.height)
    fx, fy, cx, cy = _camera_intrinsics(
        depth_camera_info,
        depth_size,
        "Depth CameraInfo",
    )
    resolved_rgb_size = _resolve_rgb_size(rgb_size, depth_image)
    _validate_pixel_bounds(pixel, resolved_rgb_size, "RGB reference image")
    mapped_pixel = map_rgb_pixel_to_depth(
        pixel,
        resolved_rgb_size,
        depth_size,
        rgb_camera_info=rgb_camera_info,
        depth_camera_info=(
            depth_camera_info
            if rgb_camera_info is not None
            else None
        ),
    )
    frame_id = _resolve_frame_id(depth_image, depth_camera_info)

    sampled_pixel, depth_m = _nearest_valid_depth(
        mapped_pixel,
        depth_image,
        search_radius_px,
    )
    x_m = (sampled_pixel.u - cx) * depth_m / fx
    y_m = (sampled_pixel.v - cy) * depth_m / fy
    return ProjectedReferencePoint(
        requested_pixel=ImagePoint(u=pixel.u, v=pixel.v),
        mapped_pixel=mapped_pixel,
        sampled_pixel=sampled_pixel,
        point_camera=Vector3Data(x=x_m, y=y_m, z=depth_m),
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
    """Map an RGB ray into the registered-depth raster."""
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
        raise ValueError(
            "RGB and depth CameraInfo must be provided together"
        )
    if rgb_camera_info is None:
        mapped_u = _map_pixel_coordinate(
            pixel.u,
            rgb_width,
            depth_width,
        )
        mapped_v = _map_pixel_coordinate(
            pixel.v,
            rgb_height,
            depth_height,
        )
        return ImagePoint(u=mapped_u, v=mapped_v)

    rgb_intrinsics = _camera_intrinsics(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_intrinsics = _camera_intrinsics(
        depth_camera_info,
        (depth_width, depth_height),
        "Depth CameraInfo",
    )
    mapped_u = _map_calibrated_coordinate(
        pixel.u,
        rgb_intrinsics[0],
        rgb_intrinsics[2],
        depth_intrinsics[0],
        depth_intrinsics[2],
    )
    mapped_v = _map_calibrated_coordinate(
        pixel.v,
        rgb_intrinsics[1],
        rgb_intrinsics[3],
        depth_intrinsics[1],
        depth_intrinsics[3],
    )
    if not (
        0 <= mapped_u < depth_width
        and 0 <= mapped_v < depth_height
    ):
        raise ValueError(
            "Reference pixel is outside the registered-depth field of view"
        )
    return ImagePoint(u=mapped_u, v=mapped_v)


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
    rgb_intrinsics = _camera_intrinsics(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_intrinsics = _camera_intrinsics(
        depth_camera_info,
        (depth_width, depth_height),
        "Depth CameraInfo",
    )
    valid_u = [
        u for u in range(rgb_width)
        if 0 <= _map_calibrated_coordinate(
            u,
            rgb_intrinsics[0],
            rgb_intrinsics[2],
            depth_intrinsics[0],
            depth_intrinsics[2],
        ) < depth_width
    ]
    valid_v = [
        v for v in range(rgb_height)
        if 0 <= _map_calibrated_coordinate(
            v,
            rgb_intrinsics[1],
            rgb_intrinsics[3],
            depth_intrinsics[1],
            depth_intrinsics[3],
        ) < depth_height
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
    _validate_depth_image(depth_image)
    depth_size = (depth_image.width, depth_image.height)
    rgb_intrinsics = _camera_intrinsics(
        rgb_camera_info,
        (rgb_width, rgb_height),
        "RGB CameraInfo",
    )
    depth_intrinsics = _camera_intrinsics(
        depth_camera_info,
        depth_size,
        "Depth CameraInfo",
    )
    depth_support = _depth_valid_support_region(depth_image)
    valid_u = [
        u for u in range(rgb_width)
        if depth_support.x <= _map_calibrated_coordinate(
            u,
            rgb_intrinsics[0],
            rgb_intrinsics[2],
            depth_intrinsics[0],
            depth_intrinsics[2],
        ) < depth_support.x + depth_support.width
    ]
    valid_v = [
        v for v in range(rgb_height)
        if depth_support.y <= _map_calibrated_coordinate(
            v,
            rgb_intrinsics[1],
            rgb_intrinsics[3],
            depth_intrinsics[1],
            depth_intrinsics[3],
        ) < depth_support.y + depth_support.height
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


def _depth_valid_support_region(depth_image: Image) -> ImageRegion:
    row_counts = [0] * depth_image.height
    column_counts = [0] * depth_image.width
    data = memoryview(depth_image.data)
    for v in range(depth_image.height):
        for u in range(depth_image.width):
            if _read_depth_m(depth_image, data, u, v) is None:
                continue
            row_counts[v] += 1
            column_counts[u] += 1

    minimum_column_count = max(
        1,
        int(math.ceil(depth_image.height * 0.01)),
    )
    minimum_row_count = max(
        1,
        int(math.ceil(depth_image.width * 0.01)),
    )
    supported_columns = [
        index
        for index, count in enumerate(column_counts)
        if count >= minimum_column_count
    ]
    supported_rows = [
        index
        for index, count in enumerate(row_counts)
        if count >= minimum_row_count
    ]
    if not supported_columns or not supported_rows:
        raise RegisteredDepthSupportNotReady(
            "Registered depth image does not contain valid depth support"
        )
    region = ImageRegion(
        x=supported_columns[0],
        y=supported_rows[0],
        width=supported_columns[-1] - supported_columns[0] + 1,
        height=supported_rows[-1] - supported_rows[0] + 1,
    )
    region.validate()
    return region


def _map_pixel_coordinate(
    coordinate: int,
    source_length: int,
    target_length: int,
) -> int:
    mapped = (
        (float(coordinate) + 0.5)
        * float(target_length)
        / float(source_length)
        - 0.5
    )
    rounded = int(math.floor(mapped + 0.5))
    return min(max(rounded, 0), target_length - 1)


def _map_calibrated_coordinate(
    coordinate: int,
    source_focal_length: float,
    source_principal_point: float,
    target_focal_length: float,
    target_principal_point: float,
) -> int:
    normalized = (
        float(coordinate) - source_principal_point
    ) / source_focal_length
    mapped = (
        normalized * target_focal_length
        + target_principal_point
    )
    return int(math.floor(mapped + 0.5))


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
    expected_size: Tuple[int, int],
    label: str,
) -> Tuple[float, float, float, float]:
    if len(camera_info.k) != 9:
        raise ValueError(f"{label} matrix must contain nine values")
    expected_width, expected_height = expected_size
    if camera_info.width not in (0, expected_width):
        raise ValueError(f"{label} width does not match its image")
    if camera_info.height not in (0, expected_height):
        raise ValueError(f"{label} height does not match its image")
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError(f"{label} contains a non-finite intrinsic")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"{label} focal lengths must be positive")
    return fx, fy, cx, cy


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
