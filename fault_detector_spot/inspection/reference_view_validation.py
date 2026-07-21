"""Validation for synchronized reference-view inputs."""

import math
from typing import TYPE_CHECKING

from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    ReferenceView,
    Vector3Data,
)

if TYPE_CHECKING:
    from fault_detector_msgs.msg import TagElement
    from sensor_msgs.msg import CameraInfo, Image


_RGB_BYTES_PER_PIXEL = {
    "bgr8": 3,
    "bgra8": 4,
    "rgb8": 3,
    "rgba8": 4,
}
_DEPTH_BYTES_PER_PIXEL = {
    "16UC1": 2,
    "32FC1": 4,
}


def validate_reference_view_inputs(
    rgb_image: "Image",
    depth_image: "Image",
    camera_info: "CameraInfo",
    reference_tag_observation: "TagElement",
    expected_reference_tag_id: int,
    controlled_frame_pose_object: PoseData,
    controlled_frame: str,
    maximum_timestamp_skew_sec: float = 0.05,
    maximum_tag_timestamp_skew_sec: float = 0.25,
) -> None:
    """Reject inputs that cannot produce a valid reference view."""
    if (
        not math.isfinite(maximum_timestamp_skew_sec)
        or maximum_timestamp_skew_sec < 0.0
    ):
        raise ValueError(
            "Maximum timestamp skew must be finite and non-negative"
        )
    if (
        not math.isfinite(maximum_tag_timestamp_skew_sec)
        or maximum_tag_timestamp_skew_sec < 0.0
    ):
        raise ValueError(
            "Maximum tag timestamp skew must be finite and non-negative"
        )

    _validate_image(
        rgb_image,
        "RGB image",
        _RGB_BYTES_PER_PIXEL,
    )
    _validate_image(
        depth_image,
        "Depth image",
        _DEPTH_BYTES_PER_PIXEL,
    )
    _validate_camera_info(camera_info)
    _validate_matching_geometry(
        rgb_image,
        depth_image,
        camera_info,
    )
    _validate_matching_frames(
        rgb_image,
        depth_image,
        camera_info,
    )
    _validate_timestamps(
        rgb_image,
        depth_image,
        maximum_timestamp_skew_sec,
    )
    _validate_reference_tag(
        reference_tag_observation,
        expected_reference_tag_id,
        rgb_image,
        maximum_tag_timestamp_skew_sec,
    )
    if controlled_frame != rgb_image.header.frame_id:
        raise ValueError(
            "Controlled frame must match the RGB optical frame"
        )

    ReferenceView(
        controlled_frame_pose_object=controlled_frame_pose_object,
        controlled_frame=controlled_frame,
    ).validate()


def _validate_image(image, name, bytes_per_pixel) -> None:
    if image.width <= 0 or image.height <= 0:
        raise ValueError(f"{name} dimensions must be positive")

    pixel_size = bytes_per_pixel.get(image.encoding)
    if pixel_size is None:
        supported = ", ".join(sorted(bytes_per_pixel))
        raise ValueError(
            f"{name} encoding must be one of: {supported}"
        )

    minimum_step = image.width * pixel_size
    if image.step < minimum_step:
        raise ValueError(f"{name} step is smaller than one row")
    if len(image.data) < image.step * image.height:
        raise ValueError(f"{name} data is incomplete")


def _validate_camera_info(camera_info) -> None:
    if camera_info.width <= 0 or camera_info.height <= 0:
        raise ValueError("CameraInfo dimensions must be positive")
    if len(camera_info.k) != 9:
        raise ValueError("CameraInfo K must contain nine values")
    if not all(math.isfinite(value) for value in camera_info.k):
        raise ValueError("CameraInfo K contains a non-finite value")
    if camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
        raise ValueError("CameraInfo focal lengths must be positive")


def _validate_matching_geometry(
    rgb_image,
    depth_image,
    camera_info,
) -> None:
    dimensions = {
        (rgb_image.width, rgb_image.height),
        (depth_image.width, depth_image.height),
        (camera_info.width, camera_info.height),
    }
    if len(dimensions) != 1:
        raise ValueError(
            "RGB, depth, and CameraInfo dimensions must match"
        )


def _validate_matching_frames(
    rgb_image,
    depth_image,
    camera_info,
) -> None:
    frames = {
        rgb_image.header.frame_id,
        depth_image.header.frame_id,
        camera_info.header.frame_id,
    }
    if "" in frames:
        raise ValueError("Reference-view frame IDs must not be empty")
    if len(frames) != 1:
        raise ValueError(
            "RGB, registered depth, and CameraInfo frames must match"
        )


def _validate_timestamps(
    rgb_image,
    depth_image,
    maximum_timestamp_skew_sec,
) -> None:
    stamps = [
        _stamp_nanoseconds(rgb_image.header.stamp, "RGB image"),
        _stamp_nanoseconds(depth_image.header.stamp, "Depth image"),
    ]
    skew_sec = (max(stamps) - min(stamps)) / 1_000_000_000
    if skew_sec > maximum_timestamp_skew_sec:
        raise ValueError(
            "RGB and depth timestamps exceed the allowed skew"
        )


def _validate_reference_tag(
    observation,
    expected_tag_id,
    rgb_image,
    maximum_skew_sec,
) -> None:
    if expected_tag_id < 0:
        raise ValueError("Expected reference tag ID must not be negative")
    if int(observation.id) != expected_tag_id:
        raise ValueError(
            "Base-camera tag does not match the reference tag"
        )
    if not observation.pose.header.frame_id:
        raise ValueError(
            "Base-camera tag observation frame must not be empty"
        )

    pose = observation.pose.pose
    PoseData(
        position=Vector3Data(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
        ),
        orientation=QuaternionData(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w,
        ),
    ).validate()

    rgb_stamp = _stamp_nanoseconds(
        rgb_image.header.stamp,
        "RGB image",
    )
    tag_stamp = _stamp_nanoseconds(
        observation.pose.header.stamp,
        "Base-camera tag observation",
    )
    skew_sec = abs(tag_stamp - rgb_stamp) / 1_000_000_000
    if skew_sec > maximum_skew_sec:
        raise ValueError(
            "Base-camera tag and RGB timestamps exceed the allowed skew"
        )


def _stamp_nanoseconds(stamp, name) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError(f"{name} timestamp is invalid")
    nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
    if nanoseconds == 0:
        raise ValueError(f"{name} timestamp must not be zero")
    return nanoseconds
