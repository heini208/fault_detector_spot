"""Convert ROS registered-depth images into organized Open3D point clouds."""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


_OPEN3D_INSTALL_MESSAGE = (
    "Open3D is required for probe depth geometry. On Ubuntu 22.04 install "
    "it with: sudo apt install python3-open3d"
)


@dataclass(frozen=True)
class OrganizedDepthPointCloud:
    """One point per depth pixel, preserving the source raster layout."""

    points_camera: np.ndarray
    depth_m: np.ndarray
    valid_mask: np.ndarray

    @property
    def width(self) -> int:
        return int(self.points_camera.shape[1])

    @property
    def height(self) -> int:
        return int(self.points_camera.shape[0])

    def point_camera(self, u: int, v: int) -> np.ndarray:
        if not (0 <= u < self.width and 0 <= v < self.height):
            raise ValueError("Depth point pixel is outside the point cloud")
        if not bool(self.valid_mask[v, u]):
            raise ValueError("Depth point pixel does not contain valid depth")
        point = np.asarray(self.points_camera[v, u], dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("Open3D returned an invalid depth point")
        return point


def create_organized_depth_point_cloud(
    depth_image: Image,
    camera_info: CameraInfo,
) -> OrganizedDepthPointCloud:
    """Create an organized camera-frame point cloud from registered depth."""
    depth_array, depth_m = _depth_arrays(depth_image)
    fx, fy, cx, cy = camera_intrinsics(
        camera_info,
        (depth_image.width, depth_image.height),
        "Depth CameraInfo",
    )
    open3d = require_open3d()
    intrinsic = open3d.camera.PinholeCameraIntrinsic(
        depth_image.width,
        depth_image.height,
        fx,
        fy,
        cx,
        cy,
    )
    point_cloud = open3d.geometry.PointCloud.create_from_depth_image(
        open3d.geometry.Image(depth_array),
        intrinsic,
        depth_scale=(1000.0 if depth_array.dtype == np.uint16 else 1.0),
        depth_trunc=1000.0,
        stride=1,
        project_valid_depth_only=False,
    )
    points = np.asarray(point_cloud.points, dtype=float)
    expected_count = depth_image.width * depth_image.height
    if points.shape != (expected_count, 3):
        raise ValueError(
            "Open3D did not preserve the registered-depth raster layout"
        )
    points = points.reshape(depth_image.height, depth_image.width, 3).copy()
    valid_mask = np.isfinite(depth_m) & (depth_m > 0.0)
    points[~valid_mask] = np.nan
    return OrganizedDepthPointCloud(
        points_camera=points,
        depth_m=depth_m,
        valid_mask=valid_mask,
    )


def camera_intrinsics(
    camera_info: CameraInfo,
    expected_size: Tuple[int, int],
    label: str,
) -> Tuple[float, float, float, float]:
    """Validate and return fx, fy, cx, cy from a ROS CameraInfo."""
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


def _depth_arrays(depth_image: Image) -> Tuple[np.ndarray, np.ndarray]:
    _validate_depth_image(depth_image)
    encoding = depth_image.encoding.strip().lower()
    if encoding == "16uc1":
        source = _depth_view(
            depth_image,
            np.dtype(">u2" if depth_image.is_bigendian else "<u2"),
        )
        native = np.asarray(source, dtype=np.uint16).copy(order="C")
        depth_m = native.astype(np.float64) * 0.001
        depth_m[native == 0] = np.nan
        return native, depth_m

    source = _depth_view(
        depth_image,
        np.dtype(">f4" if depth_image.is_bigendian else "<f4"),
    )
    native = np.asarray(source, dtype=np.float32).copy(order="C")
    invalid = ~np.isfinite(native) | (native <= 0.0)
    native[invalid] = 0.0
    depth_m = native.astype(np.float64)
    depth_m[invalid] = np.nan
    return native, depth_m


def _depth_view(depth_image: Image, dtype: np.dtype) -> np.ndarray:
    return np.ndarray(
        shape=(depth_image.height, depth_image.width),
        dtype=dtype,
        buffer=memoryview(depth_image.data),
        strides=(depth_image.step, dtype.itemsize),
    )


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


def require_open3d():
    try:
        import open3d
    except ImportError as exception:
        raise RuntimeError(_OPEN3D_INSTALL_MESSAGE) from exception
    return open3d
