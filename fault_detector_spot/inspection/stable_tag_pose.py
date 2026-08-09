"""Stabilize fresh base-camera tag poses before supervised motion."""

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .models import PoseData, QuaternionData, Vector3Data


@dataclass(frozen=True)
class TagPoseSample:
    """One timestamped base-camera tag pose."""

    stamp_seconds: float
    frame_id: str
    pose: PoseData


@dataclass(frozen=True)
class StableTagPose:
    """Quality-checked pose formed from distinct fresh observations."""

    pose: PoseData
    frame_id: str
    newest_stamp_seconds: float
    sample_count: int
    sample_span_sec: float
    maximum_position_deviation_m: float
    maximum_orientation_deviation_rad: float


def stabilize_tag_pose(
    samples: Iterable[TagPoseSample],
    now_seconds: float,
    maximum_age_sec: float = 0.25,
    minimum_samples: int = 3,
    minimum_span_sec: float = 0.10,
    maximum_position_deviation_m: float = 0.015,
    maximum_orientation_deviation_rad: float = math.radians(5.0),
) -> StableTagPose:
    """Return a stable pose from distinct fresh base observations."""
    _validate_parameters(
        now_seconds,
        maximum_age_sec,
        minimum_samples,
        minimum_span_sec,
        maximum_position_deviation_m,
        maximum_orientation_deviation_rad,
    )
    ordered = _distinct_sorted_samples(samples)
    fresh = [
        sample
        for sample in ordered
        if -0.05
        <= now_seconds - sample.stamp_seconds
        <= maximum_age_sec
    ]
    if len(fresh) < minimum_samples:
        raise ValueError(
            "Need at least three distinct fresh base-tag observations"
        )
    selected = _recent_sample_window(
        fresh,
        minimum_samples,
        minimum_span_sec,
    )
    frames = {sample.frame_id for sample in selected}
    if "" in frames or len(frames) != 1:
        raise ValueError("Stable base-tag samples must share one frame")
    span = selected[-1].stamp_seconds - selected[0].stamp_seconds
    if span + 1e-9 < minimum_span_sec:
        raise ValueError(
            "Base-tag observations do not span the stabilization window"
        )

    positions = np.array(
        [
            [
                sample.pose.position.x,
                sample.pose.position.y,
                sample.pose.position.z,
            ]
            for sample in selected
        ],
        dtype=float,
    )
    position = np.median(positions, axis=0)
    position_deviations = np.linalg.norm(
        positions - position,
        axis=1,
    )
    maximum_position_deviation = float(np.max(position_deviations))
    if maximum_position_deviation > maximum_position_deviation_m:
        raise ValueError("Base-tag position is not stable")

    quaternions = np.array(
        [
            [
                sample.pose.orientation.x,
                sample.pose.orientation.y,
                sample.pose.orientation.z,
                sample.pose.orientation.w,
            ]
            for sample in selected
        ],
        dtype=float,
    )
    reference = quaternions[-1]
    signs = np.where(quaternions @ reference < 0.0, -1.0, 1.0)
    aligned = quaternions * signs[:, None]
    quaternion = np.mean(aligned, axis=0)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Stable base-tag orientation cannot be normalized")
    quaternion /= norm
    dots = np.clip(np.abs(aligned @ quaternion), 0.0, 1.0)
    orientation_deviations = 2.0 * np.arccos(dots)
    maximum_orientation_deviation = float(
        np.max(orientation_deviations)
    )
    if maximum_orientation_deviation > maximum_orientation_deviation_rad:
        raise ValueError("Base-tag orientation is not stable")

    pose = PoseData(
        position=Vector3Data(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        ),
        orientation=QuaternionData(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        ),
    )
    pose.validate()
    return StableTagPose(
        pose=pose,
        frame_id=selected[-1].frame_id,
        newest_stamp_seconds=selected[-1].stamp_seconds,
        sample_count=len(selected),
        sample_span_sec=span,
        maximum_position_deviation_m=maximum_position_deviation,
        maximum_orientation_deviation_rad=(
            maximum_orientation_deviation
        ),
    )


def _distinct_sorted_samples(
    samples: Iterable[TagPoseSample],
) -> Tuple[TagPoseSample, ...]:
    by_stamp = {}
    for sample in samples:
        if not math.isfinite(sample.stamp_seconds):
            raise ValueError("Base-tag timestamp must be finite")
        if not sample.frame_id or sample.frame_id != sample.frame_id.strip():
            raise ValueError("Base-tag frame must not be empty or padded")
        sample.pose.validate()
        by_stamp[sample.stamp_seconds] = sample
    return tuple(by_stamp[stamp] for stamp in sorted(by_stamp))


def _recent_sample_window(
    samples,
    minimum_samples,
    minimum_span_sec,
):
    """Return the smallest newest suffix spanning the required window."""
    newest_stamp = samples[-1].stamp_seconds
    for index in range(len(samples) - 2, -1, -1):
        if (
            len(samples) - index >= minimum_samples
            and newest_stamp - samples[index].stamp_seconds + 1e-9
            >= minimum_span_sec
        ):
            return samples[index:]
    return samples


def _validate_parameters(
    now_seconds,
    maximum_age_sec,
    minimum_samples,
    minimum_span_sec,
    maximum_position_deviation_m,
    maximum_orientation_deviation_rad,
):
    values = (
        now_seconds,
        maximum_age_sec,
        minimum_span_sec,
        maximum_position_deviation_m,
        maximum_orientation_deviation_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Tag stabilization parameters must be finite")
    if maximum_age_sec <= 0.0:
        raise ValueError("Maximum tag age must be positive")
    if minimum_span_sec < 0.0:
        raise ValueError("Tag stabilization span must not be negative")
    if maximum_position_deviation_m <= 0.0:
        raise ValueError("Maximum tag position deviation must be positive")
    if maximum_orientation_deviation_rad <= 0.0:
        raise ValueError(
            "Maximum tag orientation deviation must be positive"
        )
    if (
        isinstance(minimum_samples, bool)
        or not isinstance(minimum_samples, int)
        or minimum_samples < 2
    ):
        raise ValueError("Minimum tag sample count must be at least two")
