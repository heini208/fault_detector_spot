"""Record derived Spot and probe geometry without a synthetic ROS topic."""

import math
from collections.abc import Callable
from copy import deepcopy
from threading import RLock
from typing import Optional

from fault_detector_spot.inspection.measurement.measurement_models import (
    MeasurementRecording,
)
from fault_detector_spot.inspection.measurement.measurement_repository import (
    MeasurementRepository,
)
from fault_detector_spot.inspection.model.models import PoseData
from fault_detector_spot.inspection.model.sensor_models import (
    SENSOR_PARENT_FRAME,
    SensorChannel,
    SensorChannelSource,
)
from fault_detector_spot.inspection.geometry.pose import relative_pose
from fault_detector_spot.shared.ros.tf_transforms import lookup_pose_data


FirstSampleCallback = Callable[[str, int], None]
SourceErrorCallback = Callable[[str, Exception], None]


class SpotGeometryRecordingSource:
    """Sample body, hand, and probe geometry while a recording is active."""

    def __init__(
        self,
        *,
        node,
        tf_buffer,
        repository: MeasurementRepository,
        recording: MeasurementRecording,
        channel: SensorChannel,
        object_pose_execution: PoseData,
        probe_frame: str,
        execution_frame: str = "odom",
        body_frame: str = "body",
        hand_frame: str = SENSOR_PARENT_FRAME,
        sample_rate_hz: float = 10.0,
        transform_timeout_sec: float = 0.02,
        on_first_sample: Optional[FirstSampleCallback] = None,
        on_error: Optional[SourceErrorCallback] = None,
        pose_lookup=lookup_pose_data,
    ):
        """Configure an internal timer source without starting its timer."""
        if channel.source_kind != SensorChannelSource.SPOT_GEOMETRY:
            raise ValueError(
                "Spot geometry source requires a spot_geometry channel"
            )
        channel.validate()
        recording.validate()
        if channel not in recording.configured_channels:
            raise ValueError("Channel is not part of the recording snapshot")
        object_pose_execution.validate()
        frames = {
            "execution": str(execution_frame).strip(),
            "body": str(body_frame).strip(),
            "hand": str(hand_frame).strip(),
            "probe": str(probe_frame).strip(),
        }
        if not all(frames.values()):
            raise ValueError("Geometry frame names must not be empty")
        rate = float(sample_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(
                "Geometry sample rate must be positive and finite"
            )
        timeout = float(transform_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("TF lookup timeout must be positive and finite")

        self._node = node
        self._tf_buffer = tf_buffer
        self._repository = repository
        self._recording = recording
        self._channel = channel
        self._object_pose_execution = deepcopy(object_pose_execution)
        self._execution_frame = frames["execution"]
        self._body_frame = frames["body"]
        self._hand_frame = frames["hand"]
        self._probe_frame = frames["probe"]
        self._sample_period_sec = 1.0 / rate
        self._transform_timeout_sec = timeout
        self._on_first_sample = on_first_sample
        self._on_error = on_error
        self._pose_lookup = pose_lookup
        self._timer = None
        self._active = False
        self._first_sample_received = False
        self._last_error_text = None
        self._lock = RLock()

    @property
    def channel_id(self) -> str:
        """Return the stable configured channel ID."""
        return self._channel.channel_id

    @property
    def active(self) -> bool:
        """Return whether the geometry timer is active."""
        with self._lock:
            return self._active

    def start(self) -> None:
        """Create the recording-only sampling timer."""
        with self._lock:
            if self._active:
                raise RuntimeError("Spot geometry recording source is active")
            if not self._repository.is_open(self._recording):
                raise RuntimeError("Measurement recording is not open")
            self._first_sample_received = False
            self._last_error_text = None
            self._active = True
            try:
                self._timer = self._node.create_timer(
                    self._sample_period_sec,
                    self._sample_geometry,
                )
            except Exception:
                self._active = False
                self._timer = None
                raise

    def stop(self) -> None:
        """Stop sampling and destroy the recording-only timer."""
        with self._lock:
            self._active = False
            timer = self._timer
            self._timer = None
        if timer is not None:
            self._node.destroy_timer(timer)

    def _sample_geometry(self) -> None:
        first_sample_callback = None
        sample_time_ns = 0
        try:
            with self._lock:
                if not self._active:
                    return
                sample_time_ns = int(
                    self._node.get_clock().now().nanoseconds
                )
                body_pose = self._lookup(self._body_frame)
                hand_pose = self._lookup(self._hand_frame)
                probe_pose = self._lookup(self._probe_frame)
                probe_pose_object = relative_pose(
                    self._object_pose_execution,
                    probe_pose,
                )
                sample = {
                    "receive_time_ns": sample_time_ns,
                    "source_time_ns": sample_time_ns,
                    "data": {
                        "frames": {
                            "execution": self._execution_frame,
                            "body": self._body_frame,
                            "hand": self._hand_frame,
                            "probe": self._probe_frame,
                            "object": self._recording.object_id,
                        },
                        "object_pose_execution": (
                            self._object_pose_execution.to_dict()
                        ),
                        "body_pose_execution": body_pose.to_dict(),
                        "hand_pose_execution": hand_pose.to_dict(),
                        "probe_pose_execution": probe_pose.to_dict(),
                        "probe_pose_object": probe_pose_object.to_dict(),
                    },
                }
                self._repository.append_sample(
                    self._recording,
                    self._channel.channel_id,
                    sample,
                )
                self._last_error_text = None
                if not self._first_sample_received:
                    self._first_sample_received = True
                    first_sample_callback = self._on_first_sample
        except Exception as exception:  # transient TF failures are expected
            self._notify_error(exception)
            return
        if first_sample_callback is not None:
            first_sample_callback(self._channel.channel_id, sample_time_ns)

    def _lookup(self, source_frame: str) -> PoseData:
        return self._pose_lookup(
            self._tf_buffer,
            self._execution_frame,
            source_frame,
            timeout_sec=self._transform_timeout_sec,
        )

    def _notify_error(self, exception: Exception) -> None:
        error_callback = None
        with self._lock:
            error_text = f"{type(exception).__name__}: {exception}"
            if error_text != self._last_error_text:
                self._last_error_text = error_text
                error_callback = self._on_error
        if error_callback is not None:
            error_callback(self._channel.channel_id, exception)


__all__ = ["SpotGeometryRecordingSource"]
