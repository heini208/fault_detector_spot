"""Record an arbitrary configured ROS message stream as JSONL."""

from collections.abc import Callable
from threading import RLock
from typing import Any, Optional

from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from fault_detector_spot.inspection.measurement.measurement_models import (
    MeasurementRecording,
)
from fault_detector_spot.inspection.measurement.measurement_repository import (
    MeasurementRepository,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannel,
    SensorChannelSource,
)


FirstSampleCallback = Callable[[str, int], None]
SourceErrorCallback = Callable[[str, Exception], None]


def message_source_time_ns(message: Any) -> Optional[int]:
    """Return a valid ``header.stamp`` value when the message has one."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    second = getattr(stamp, "sec", None)
    nanosecond = getattr(stamp, "nanosec", None)
    if (
        isinstance(second, bool)
        or not isinstance(second, int)
        or isinstance(nanosecond, bool)
        or not isinstance(nanosecond, int)
        or second < 0
        or nanosecond < 0
        or nanosecond >= 1_000_000_000
    ):
        return None
    return second * 1_000_000_000 + nanosecond


class RosTopicRecordingSource:
    """Own one temporary ROS subscription for an open measurement."""

    def __init__(
        self,
        *,
        node,
        repository: MeasurementRepository,
        recording: MeasurementRecording,
        channel: SensorChannel,
        on_first_sample: Optional[FirstSampleCallback] = None,
        on_error: Optional[SourceErrorCallback] = None,
        qos_profile=qos_profile_sensor_data,
        message_type_resolver=get_message,
        message_converter=message_to_ordereddict,
        callback_group=None,
    ):
        """Configure a source without subscribing until :meth:`start`."""
        if channel.source_kind != SensorChannelSource.ROS_TOPIC:
            raise ValueError("ROS topic source requires a ros_topic channel")
        channel.validate()
        recording.validate()
        if channel not in recording.configured_channels:
            raise ValueError("Channel is not part of the recording snapshot")
        self._node = node
        self._repository = repository
        self._recording = recording
        self._channel = channel
        self._on_first_sample = on_first_sample
        self._on_error = on_error
        self._qos_profile = qos_profile
        self._message_type_resolver = message_type_resolver
        self._message_converter = message_converter
        self._callback_group = callback_group
        self._subscription = None
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
        """Return whether the source currently accepts messages."""
        with self._lock:
            return self._active

    def start(self) -> None:
        """Resolve the type and create the temporary subscription."""
        with self._lock:
            if self._active:
                raise RuntimeError("ROS topic recording source is active")
            if not self._repository.is_open(self._recording):
                raise RuntimeError("Measurement recording is not open")
            message_type = self._message_type_resolver(
                self._channel.message_type
            )
            self._first_sample_received = False
            self._last_error_text = None
            self._active = True
            try:
                subscription_options = {}
                if self._callback_group is not None:
                    subscription_options["callback_group"] = (
                        self._callback_group
                    )
                self._subscription = self._node.create_subscription(
                    message_type,
                    self._channel.topic,
                    self._receive_message,
                    self._qos_profile,
                    **subscription_options,
                )
            except Exception:
                self._active = False
                self._subscription = None
                raise

    def stop(self) -> None:
        """Stop accepting messages and destroy the temporary subscription."""
        with self._lock:
            self._active = False
            subscription = self._subscription
            self._subscription = None
        if subscription is not None:
            self._node.destroy_subscription(subscription)

    def _receive_message(self, message: Any) -> None:
        first_sample_callback = None
        receive_time_ns = 0
        try:
            with self._lock:
                if not self._active:
                    return
                receive_time_ns = int(
                    self._node.get_clock().now().nanoseconds
                )
                sample = {
                    "receive_time_ns": receive_time_ns,
                    "source_time_ns": message_source_time_ns(message),
                    "data": dict(self._message_converter(message)),
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
        except Exception as exception:  # keep the ROS executor alive
            self._notify_error(exception)
            return
        if first_sample_callback is not None:
            first_sample_callback(self._channel.channel_id, receive_time_ns)

    def _notify_error(self, exception: Exception) -> None:
        error_callback = None
        with self._lock:
            error_text = f"{type(exception).__name__}: {exception}"
            if error_text != self._last_error_text:
                self._last_error_text = error_text
                error_callback = self._on_error
        if error_callback is not None:
            error_callback(self._channel.channel_id, exception)


__all__ = [
    "RosTopicRecordingSource",
    "message_source_time_ns",
]
