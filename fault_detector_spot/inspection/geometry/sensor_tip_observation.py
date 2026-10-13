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
    """Visibility and a lateral translation that brings the target patch into view."""

    visible: bool
    camera_pose_odom: PoseData
    hand_translation_odom: Vector3Data
    detail: str


def target_camera_point(reference_probe_pose, camera_pose_odom, expected_distance_m):
    """Express the intended +X target in the optical camera frame."""
    if not math.isfinite(expected_distance_m) or expected_distance_m <= 0.0:
        raise ValueError("Expected sensor-tip distance must be positive and finite")
    probe_in_camera = relative_pose(camera_pose_odom, reference_probe_pose)
    return (
        rotation_from_quaternion(probe_in_camera.orientation).apply(
            [expected_distance_m, 0.0, 0.0],
        ) + _array(probe_in_camera.position)
    )


def observation_translation(
    reference_probe_pose, camera_pose_odom, expected_distance_m,
    intrinsics, image_size, patch_margin_px,
):
    """Bring the target to the nearest usable image pixel, without advancing.

    A visible patch need not be centered. Clip only out-of-bounds coordinates
    to an inset that leaves room for the full plane fit. Solve translation
    along the plane perpendicular to the saved probe +X; hand orientation
    stays unchanged and no mount-specific offsets are introduced.
    """
    reference_probe_pose.validate()
    camera_pose_odom.validate()
    width, height = image_size
    if (not math.isfinite(patch_margin_px) or patch_margin_px < 0
            or min(width, height) <= 2 * patch_margin_px + 1):
        raise ValueError("Camera image is too small for the observation patch margin")
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(value) for value in intrinsics) or min(fx, fy) <= 0:
        raise ValueError("Observation requires valid camera intrinsics")
    point_camera = target_camera_point(
        reference_probe_pose, camera_pose_odom, expected_distance_m,
    )
    u, v = project_camera_point(point_camera, intrinsics)
    desired_u = max(patch_margin_px, min(width - 1 - patch_margin_px, u))
    desired_v = max(patch_margin_px, min(height - 1 - patch_margin_px, v))
    if (desired_u, desired_v) == (u, v):
        return Vector3Data.zero()

    probe_axis = rotation_from_quaternion(reference_probe_pose.orientation).apply(
        [1.0, 0.0, 0.0],
    )
    camera_rotation = rotation_from_quaternion(camera_pose_odom.orientation)
    desired_ray = camera_rotation.apply([(desired_u-cx)/fx, (desired_v-cy)/fy, 1.0])
    displacement = camera_rotation.apply(point_camera)
    denominator = float(np.dot(desired_ray, probe_axis))
    if denominator <= 0.1:
        raise ValueError("Camera viewing direction cannot observe the forward probe target safely")
    optical_depth = float(np.dot(displacement, probe_axis)) / denominator
    if optical_depth <= 0.0:
        raise ValueError("Intended sensor-tip target is behind the camera")
    translation = displacement - optical_depth * desired_ray
    return Vector3Data(*(float(value) for value in translation))


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
