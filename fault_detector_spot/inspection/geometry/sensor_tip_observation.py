"""Camera observation geometry for a fixed, probe-referenced wall target."""

from dataclasses import dataclass
import math

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
):
    """Place the camera on the saved probe axis without knowing target depth.

    Translate perpendicular to the original probe +X and preserve orientation.
    Once the optical center lies on that line, every forward target projects to
    the same calibrated pixel. The pixel follows the mount direction, rather
    than a surface estimate or an arbitrary image destination.
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
    translation_camera = origin - axial_offset * direction
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
