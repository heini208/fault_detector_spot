"""Compose one validated and persisted reference-view capture."""

import math

from fault_detector_spot.inspection.models import InspectionObject
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)
from fault_detector_spot.inspection.reference_view_pose_resolver import (
    resolve_reference_view_pose,
)
from fault_detector_spot.inspection.reference_view_validation import (
    validate_reference_view_inputs,
)


_FUTURE_TOLERANCE_SEC = 0.1


class ReferenceViewCaptureNotReady(RuntimeError):
    """Indicate that the completed collection has no usable capture."""


def validate_reference_view_capture_target(
    object_repository: ObjectRepository,
    object_id: str,
    routine_id: str,
    replace_existing: bool,
) -> int:
    """Validate the persistent target before collecting sensor data."""
    definition = object_repository.load(object_id)
    routine = definition.get_routine(routine_id)
    if routine is None:
        raise KeyError(f"Routine does not exist: {routine_id}")
    if routine.reference_view is not None and not replace_existing:
        raise FileExistsError(
            "Routine already has a reference view: "
            f"{object_id}/{routine_id}"
        )
    return definition.reference_tag.tag_id


def capture_reference_view(
    object_repository: ObjectRepository,
    input_synchronizer: ReferenceViewInputSynchronizer,
    tf_buffer,
    object_id: str,
    routine_id: str,
    current_time,
    maximum_input_age_sec: float = 2.0,
    maximum_timestamp_skew_sec: float = 0.05,
    maximum_tag_timestamp_skew_sec: float = 0.25,
    fixed_frame: str = "odom",
    transform_timeout_sec: float = 0.05,
    replace_existing: bool = False,
    minimum_image_sequence: int = 0,
) -> InspectionObject:
    """Persist the best set from one completed collection window."""
    if (
        not math.isfinite(maximum_input_age_sec)
        or maximum_input_age_sec < 0.0
    ):
        raise ValueError(
            "Maximum input age must be finite and non-negative"
        )

    reference_tag_id = validate_reference_view_capture_target(
        object_repository,
        object_id,
        routine_id,
        replace_existing,
    )
    inputs = input_synchronizer.best_snapshot(
        reference_tag_id,
        minimum_image_sequence,
    )
    if inputs is None:
        raise ReferenceViewCaptureNotReady(
            "No valid synchronized RGB, depth, and base-tag set was "
            f"collected for tag {reference_tag_id}"
        )

    rgb_image, depth_image, camera_info, reference_tag = inputs
    _require_fresh(
        current_time,
        rgb_image.header.stamp,
        "RGB image",
        maximum_input_age_sec,
    )
    _require_fresh(
        current_time,
        depth_image.header.stamp,
        "Depth image",
        maximum_input_age_sec,
    )
    _require_fresh(
        current_time,
        reference_tag.pose.header.stamp,
        "Base-camera tag observation",
        maximum_input_age_sec,
    )

    reference_view = resolve_reference_view_pose(
        tf_buffer,
        rgb_image,
        reference_tag,
        fixed_frame=fixed_frame,
        transform_timeout_sec=transform_timeout_sec,
    )
    validate_reference_view_inputs(
        rgb_image,
        depth_image,
        camera_info,
        reference_tag,
        reference_tag_id,
        reference_view.controlled_frame_pose_object,
        reference_view.controlled_frame,
        maximum_timestamp_skew_sec=maximum_timestamp_skew_sec,
        maximum_tag_timestamp_skew_sec=(
            maximum_tag_timestamp_skew_sec
        ),
    )

    return object_repository.save_reference_dataset(
        object_id,
        routine_id,
        reference_view,
        rgb_image,
        depth_image,
        camera_info,
    )


def _require_fresh(
    current_time,
    stamp,
    name: str,
    maximum_age_sec: float,
) -> None:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError(f"{name} timestamp is invalid")

    stamp_nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
    if stamp_nanoseconds == 0:
        raise ValueError(f"{name} timestamp must not be zero")

    age_sec = (
        current_time.nanoseconds - stamp_nanoseconds
    ) / 1_000_000_000
    if age_sec < -_FUTURE_TOLERANCE_SEC:
        raise ValueError(
            f"{name} timestamp is in the future: {age_sec:.3f} s"
        )
    if age_sec > maximum_age_sec:
        raise ReferenceViewCaptureNotReady(
            f"{name} is stale: {age_sec:.3f} s"
        )
