"""Resolve and persist synchronized reference views for selected cameras."""

import math
from dataclasses import dataclass
from typing import Sequence

from fault_detector_spot.inspection.data.multi_reference_view_repository import (
    CapturedReferenceView,
    MultiReferenceViewRepository,
)
from fault_detector_spot.inspection.setup.reference_view_capture import (
    ReferenceViewCaptureNotReady,
)
from fault_detector_spot.inspection.setup.reference_view_pose_resolver import (
    resolve_reference_view_pose,
)
from fault_detector_spot.inspection.setup.reference_view_validation import (
    validate_reference_view_inputs,
)


_FUTURE_TOLERANCE_SEC = 0.1


@dataclass(frozen=True)
class CameraCaptureRequest:
    """One selected camera and its active synchronization transaction."""

    slot_index: int
    camera_id: str
    input_synchronizer: object
    minimum_input_sequence: int


def validate_multi_reference_view_capture_target(
    repository: MultiReferenceViewRepository,
    object_id: str,
    routine_id: str,
    replace_existing: bool,
) -> int:
    """Validate the target routine before creating camera subscriptions."""
    definition = repository.object_repository.load(object_id)
    routine = definition.get_routine(routine_id)
    if routine is None:
        raise KeyError(f"Routine does not exist: {routine_id}")
    if routine.reference_views and not replace_existing:
        raise FileExistsError(
            "Routine already has captured reference views: "
            f"{object_id}/{routine_id}"
        )
    return definition.reference_tag.tag_id


def capture_reference_views(
    repository: MultiReferenceViewRepository,
    requests: Sequence[CameraCaptureRequest],
    tf_buffer,
    object_id: str,
    routine_id: str,
    current_time,
    reference_tag_id: int,
    reference_tags,
    maximum_input_age_sec: float = 2.0,
    maximum_timestamp_skew_sec: float = 0.05,
    fixed_frame: str = "odom",
    transform_timeout_sec: float = 0.05,
):
    """Resolve every selected camera and save the set only when all pass."""
    if (
        not math.isfinite(maximum_input_age_sec)
        or maximum_input_age_sec < 0.0
    ):
        raise ValueError(
            "Maximum input age must be finite and non-negative"
        )
    snapshots = []
    for request in requests:
        inputs = request.input_synchronizer.best_snapshot(
            request.minimum_input_sequence,
        )
        if inputs is None:
            diagnostics = request.input_synchronizer.collection_diagnostics(
                request.minimum_input_sequence,
            )
            raise ReferenceViewCaptureNotReady(
                "No valid synchronized RGB and registered-depth pair was "
                f"collected for camera {request.camera_id}: {diagnostics}"
            )
        (
            rgb_image,
            depth_image,
            rgb_camera_info,
            depth_camera_info,
        ) = inputs
        _require_fresh(
            current_time,
            rgb_image.header.stamp,
            f"{request.camera_id} RGB image",
            maximum_input_age_sec,
        )
        _require_fresh(
            current_time,
            depth_image.header.stamp,
            f"{request.camera_id} depth image",
            maximum_input_age_sec,
        )
        snapshots.append((
            request,
            rgb_image,
            depth_image,
            rgb_camera_info,
            depth_camera_info,
        ))

    reference_tag = _select_reference_tag(
        reference_tags,
        reference_tag_id,
    )

    captures = []
    for (
        request,
        rgb_image,
        depth_image,
        rgb_camera_info,
        depth_camera_info,
    ) in snapshots:
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
            depth_camera_info,
            reference_tag,
            reference_tag_id,
            reference_view.controlled_frame_pose_object,
            reference_view.controlled_frame,
            maximum_timestamp_skew_sec=maximum_timestamp_skew_sec,
            rgb_camera_info=rgb_camera_info,
        )
        captures.append(CapturedReferenceView(
            slot_index=request.slot_index,
            camera_id=request.camera_id,
            reference_view=reference_view,
            rgb_image=rgb_image,
            depth_image=depth_image,
            rgb_camera_info=rgb_camera_info,
            depth_camera_info=depth_camera_info,
            reference_tag=reference_tag,
            fixed_frame=fixed_frame,
        ))

    return repository.save_reference_views(
        object_id,
        routine_id,
        captures,
        maximum_timestamp_skew_sec=maximum_timestamp_skew_sec,
    )


def _select_reference_tag(
    reference_tags,
    reference_tag_id: int,
):
    candidates = tuple(
        reference_tag
        for reference_tag in reference_tags
        if int(reference_tag.id) == reference_tag_id
    )
    if not candidates:
        raise ReferenceViewCaptureNotReady(
            "No capture anchor is available for "
            f"reference tag {reference_tag_id}"
        )
    return max(
        candidates,
        key=lambda reference_tag: _stamp_nanoseconds(
            reference_tag.pose.header.stamp
        ),
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


def _stamp_nanoseconds(stamp) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Sensor input timestamp is invalid")
    nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
    if nanoseconds == 0:
        raise ValueError("Sensor input timestamp must not be zero")
    return nanoseconds
