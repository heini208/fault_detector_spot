"""Shared helpers for resolving named ROS frame transforms."""

import math

from rclpy.duration import Duration
from rclpy.time import Time

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


def transform_to_pose_data(transform) -> PoseData:
    """Convert a geometry_msgs Transform or TransformStamped to PoseData."""
    value = getattr(transform, "transform", transform)
    if not hasattr(value, "translation") or not hasattr(value, "rotation"):
        raise TypeError("Expected Transform or TransformStamped")
    pose = PoseData(
        position=Vector3Data(
            x=float(value.translation.x),
            y=float(value.translation.y),
            z=float(value.translation.z),
        ),
        orientation=QuaternionData(
            x=float(value.rotation.x),
            y=float(value.rotation.y),
            z=float(value.rotation.z),
            w=float(value.rotation.w),
        ),
    )
    pose.validate()
    return pose


def lookup_pose_data(
    buffer,
    target_frame: str,
    source_frame: str,
    lookup_time=None,
    timeout_sec: float = 0.5,
) -> PoseData:
    """Resolve target<-source through tf2 and return the rigid transform."""
    target = str(target_frame).strip()
    source = str(source_frame).strip()
    if not target:
        raise ValueError("TF target frame must not be empty")
    if not source:
        raise ValueError("TF source frame must not be empty")
    timeout = float(timeout_sec)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("TF lookup timeout must be positive and finite")
    transform = buffer.lookup_transform(
        target,
        source,
        lookup_time if lookup_time is not None else Time(),
        timeout=Duration(seconds=timeout),
    )
    return transform_to_pose_data(transform)


__all__ = [
    "lookup_pose_data",
    "transform_to_pose_data",
]
