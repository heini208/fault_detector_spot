"""Resolve a captured camera pose in the reference-tag frame."""

import math
from copy import deepcopy

import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from fault_detector_spot.inspection.model.models import ReferenceView
from fault_detector_spot.shared.geometry.transforms import (
    pose_to_pose_data,
    relative_pose,
)


def resolve_reference_view_pose(
    tf_buffer,
    rgb_image,
    reference_tag_observation,
    fixed_frame: str = "odom",
    transform_timeout_sec: float = 0.05,
) -> ReferenceView:
    """Resolve the RGB optical frame relative to the reference tag."""
    if not fixed_frame.strip():
        raise ValueError("Fixed frame must not be empty")
    if (
        not math.isfinite(transform_timeout_sec)
        or transform_timeout_sec < 0.0
    ):
        raise ValueError(
            "Transform timeout must be finite and non-negative"
        )

    _validate_stamped_frame(
        rgb_image.header,
        "RGB image",
    )
    _validate_stamped_frame(
        reference_tag_observation.pose.header,
        "Base-camera tag observation",
    )

    camera_origin = PoseStamped()
    camera_origin.header = deepcopy(rgb_image.header)
    camera_origin.pose.orientation.w = 1.0
    timeout = Duration(seconds=transform_timeout_sec)

    tag_in_fixed = tf_buffer.transform(
        deepcopy(reference_tag_observation.pose),
        fixed_frame,
        timeout=timeout,
    )
    camera_in_fixed = tf_buffer.transform(
        camera_origin,
        fixed_frame,
        timeout=timeout,
    )

    _require_fixed_frame(
        tag_in_fixed,
        fixed_frame,
        "transformed tag pose",
    )
    _require_fixed_frame(
        camera_in_fixed,
        fixed_frame,
        "transformed camera pose",
    )

    camera_pose_object = relative_pose(
        tag_in_fixed,
        camera_in_fixed,
    )
    reference_view = ReferenceView(
        controlled_frame_pose_object=pose_to_pose_data(
            camera_pose_object
        ),
        controlled_frame=rgb_image.header.frame_id,
    )
    reference_view.validate()
    return reference_view


def _validate_stamped_frame(header, name: str) -> None:
    if not header.frame_id.strip():
        raise ValueError(f"{name} frame must not be empty")

    stamp = header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError(f"{name} timestamp is invalid")
    if stamp.sec == 0 and stamp.nanosec == 0:
        raise ValueError(f"{name} timestamp must not be zero")


def _require_fixed_frame(
    pose: PoseStamped,
    fixed_frame: str,
    name: str,
) -> None:
    if pose.header.frame_id != fixed_frame:
        raise ValueError(
            f"TF returned {name} in frame "
            f"'{pose.header.frame_id}', expected '{fixed_frame}'"
        )
