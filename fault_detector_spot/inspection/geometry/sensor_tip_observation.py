"""Camera observation geometry for a fixed, probe-referenced wall target."""

from dataclasses import dataclass
import math
from numbers import Real

import numpy as np

from fault_detector_spot.inspection.geometry.pose import relative_pose
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import rotation_from_quaternion


@dataclass(frozen=True)
class SensorTipObservationView:
    """Visibility, lateral observation move, and measured target association."""

    visible: bool
    camera_pose_odom: PoseData
    hand_translation_odom: Vector3Data
    detail: str
    target_plane_probe: SurfacePlane | None = None


def target_camera_point(reference_probe_pose, camera_pose_odom, expected_distance_m):
    """Express the intended +X target in the optical camera frame."""
    if not math.isfinite(expected_distance_m) or expected_distance_m <= 0.0:
        raise ValueError("Expected sensor-tip distance must be positive and finite")
    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose_odom)
    return origin + expected_distance_m * direction


def probe_axis_in_camera(reference_probe_pose, camera_pose_odom):
    """Express the fixed probe origin and unit +X direction in the camera."""
    probe_in_camera = relative_pose(camera_pose_odom, reference_probe_pose)
    return (
        _array(probe_in_camera.position),
        rotation_from_quaternion(probe_in_camera.orientation).apply([1.0, 0.0, 0.0]),
    )


def probe_axis_observation_translation(
    reference_probe_pose, camera_pose_odom, intrinsics, image_size, patch_margin_px,
    *, reference_position=None,
):
    """Bring the saved probe ray into view without knowing target depth.

    Translate perpendicular to the original probe +X and preserve orientation.
    By default, place the optical center on that line so every forward target
    projects to the mount's calibrated axis pixel. A custom normalized image
    position instead places the original tip origin at the selected pixel,
    inset by the patch margin. Positive-range targets then project along the
    segment from that pixel toward the calibrated axis pixel; their exact
    pixel depends on distance. Neither mode estimates a surface before moving.
    """
    reference_probe_pose.validate()
    camera_pose_odom.validate()
    width, height = image_size
    if (not all(math.isfinite(value) for value in (width, height, patch_margin_px))
            or patch_margin_px < 0
            or min(width, height) <= 2 * patch_margin_px + 1):
        raise ValueError("Camera image is too small for the observation patch margin")
    fx, fy, _, _ = intrinsics
    if not all(math.isfinite(value) for value in intrinsics) or min(fx, fy) <= 0:
        raise ValueError("Observation requires valid camera intrinsics")

    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose_odom)
    if direction[2] <= 0.0:
        raise ValueError("The saved probe axis does not face forward in the camera")
    u, v = project_camera_point(direction, intrinsics)
    if not (patch_margin_px <= u <= width - 1 - patch_margin_px
            and patch_margin_px <= v <= height - 1 - patch_margin_px):
        raise ValueError("The saved probe axis does not preserve the full observation patch margin")

    axial_offset = float(np.dot(origin, direction))
    # Ignore only floating-point transform noise at coincident axial positions.
    if axial_offset < -1e-9:
        raise ValueError("Camera is ahead of the original sensor tip along the saved probe axis")
    if reference_position is None:
        translation_camera = origin - axial_offset * direction
    else:
        try:
            horizontal, vertical = reference_position
        except (TypeError, ValueError) as error:
            raise ValueError("Observation reference position must contain two normalized coordinates") from error
        if (isinstance(reference_position, (str, bytes, dict))
                or any(isinstance(value, bool) or not isinstance(value, Real)
                       or not math.isfinite(value) or not 0.0 <= value <= 1.0
                       for value in (horizontal, vertical))):
            raise ValueError("Observation reference coordinates must be finite numbers from 0 to 1")
        if axial_offset <= 0.0:
            raise ValueError("Custom observation reference requires the camera behind the original sensor tip")
        selected_u = patch_margin_px + float(horizontal) * (width - 1 - 2 * patch_margin_px)
        selected_v = patch_margin_px + float(vertical) * (height - 1 - 2 * patch_margin_px)
        _, _, cx, cy = intrinsics
        selected_ray = np.array([(selected_u - cx) / fx, (selected_v - cy) / fy, 1.0])
        denominator = float(np.dot(selected_ray, direction))
        if not math.isfinite(denominator) or denominator <= 0.0:
            raise ValueError("Observation reference ray does not face along the saved probe axis")
        translation_camera = origin - axial_offset / denominator * selected_ray
    translation_odom = rotation_from_quaternion(camera_pose_odom.orientation).apply(
        translation_camera,
    )
    return Vector3Data(*(float(value) for value in translation_odom))


def project_camera_point(point_camera, intrinsics):
    """Project an optical-frame point using calibrated rectified intrinsics."""
    point = np.asarray(point_camera, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)) or point[2] <= 0.0:
        raise ValueError("Intended sensor-tip target is behind the camera or invalid")
    fx, fy, cx, cy = intrinsics
    return float(fx * point[0] / point[2] + cx), float(fy * point[1] / point[2] + cy)


def transform_surface_plane(plane, parent_to_plane_frame: PoseData, frame_id: str):
    """Express a plane in its parent frame without changing its normal sign."""
    plane.validate()
    parent_to_plane_frame.validate()
    rotation = rotation_from_quaternion(parent_to_plane_frame.orientation)
    point = rotation.apply(plane.point_array()) + _array(parent_to_plane_frame.position)
    normal = rotation.apply(plane.normal_array())
    result = SurfacePlane(
        point=Vector3Data(*(float(value) for value in point)),
        normal=Vector3Data(*(float(value) for value in normal)),
        frame_id=frame_id,
        inlier_count=plane.inlier_count,
        sample_count=plane.sample_count,
        inlier_ratio=plane.inlier_ratio,
        rmse_m=plane.rmse_m,
    )
    result.validate()
    return result


def _array(vector):
    return np.array([vector.x, vector.y, vector.z], dtype=float)
