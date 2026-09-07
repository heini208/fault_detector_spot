"""Coordinate one multi-channel sensor measurement."""

import time
from dataclasses import dataclass, field
from enum import Enum
from threading import RLock

import tf2_ros
from fault_detector_msgs.srv import SetSensorAcquisition
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.inspection.measurement import (
    MeasurementCompletionState,
    MeasurementRecording,
    RosTopicRecordingSource,
    SpotGeometryRecordingSource,
)
from fault_detector_spot.inspection.model.models import PoseData
from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannelSource,
)
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class SensorAcquisitionStatus(str, Enum):
    """Public sensor recording states."""

    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"
    FAILED = "failed"


@dataclass(frozen=True)
class SensorAcquisitionRequest:
    """Context required to start one measurement."""

    object_id: str
    routine_id: str
    probe_point_id: str
    object_pose_execution: PoseData
    execution_frame: str = "odom"

    def validate(self):
        for value, label in (
            (self.object_id, "object ID"),
            (self.routine_id, "routine ID"),
            (self.probe_point_id, "probe point ID"),
        ):
            validate_storage_name(value, label)
        if not self.execution_frame.strip():
            raise ValueError("Execution frame must not be empty")
        self.object_pose_execution.validate()


@dataclass(frozen=True)
class SensorAcquisitionState:
    """Authoritative state consumed by commands and the UI."""

    status: SensorAcquisitionStatus
    sensor_id: str = ""
    detail: str = "Idle"


@dataclass
class _Session:
    recording: MeasurementRecording
    sources: list = field(default_factory=list)
    ready_channels: set = field(default_factory=set)
    required_channels: frozenset = frozenset()
    physical_channels: frozenset = frozenset()
    client: object = None
    timer: object = None
    tf_listener: object = None
    head_started: bool = False
    deadline: float = 0.0
    final_state: MeasurementCompletionState = (
        MeasurementCompletionState.COMPLETE
    )
    final_detail: str = "Recording stopped"


class SensorAcquisitionCoordinator:
    """Own sensor-head control, recording sources, and finalization."""

    START_TIMEOUT_SEC = 5.0
    STOP_TIMEOUT_SEC = 3.0
    GEOMETRY_RATE_HZ = 10.0

    def __init__(
        self,
        *,
        node,
        measurement_repository,
        sensor_attachment_controller,
        sensor_repository,
        error_handler=None,
    ):
        self.node = node
        self.measurements = measurement_repository
        self.attachments = sensor_attachment_controller
        self.sensors = sensor_repository
        self.error_handler = error_handler
        self.callback_group = ReentrantCallbackGroup()
        self._lock = RLock()
        self._listeners = []
        self._session = None
        self._last_start_ns = -1
        self._state = SensorAcquisitionState(SensorAcquisitionStatus.IDLE)

    def snapshot(self):
        with self._lock:
            return self._state

    def add_state_listener(self, listener):
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_state_listener(self, listener):
        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def start(self, request):
        """Start acquisition, or return idle when it is safely skipped."""
        if not isinstance(request, SensorAcquisitionRequest):
            raise TypeError("Expected SensorAcquisitionRequest")
        request.validate()
        with self._lock:
            if self._session is not None:
                raise RuntimeError("Sensor acquisition is already active")
            client = None
            try:
                attachment = self.attachments.require_motion_attachment()
                if not attachment.has_sensor:
                    return self._set_state(
                        SensorAcquisitionStatus.IDLE,
                        detail="No sensor attached; acquisition skipped",
                    )
                definition = self.sensors.load(attachment.sensor_id)
                if not definition.channels:
                    return self._set_state(
                        SensorAcquisitionStatus.IDLE,
                        detail="Sensor has no channels; acquisition skipped",
                    )

                physical = self._physical_channels(
                    definition.channels,
                    attachment.sensor_id,
                )
                client = self._sensor_client(attachment.sensor_id, physical)
                if physical and client is None:
                    return self._set_state(
                        SensorAcquisitionStatus.IDLE,
                        detail="Sensor head is offline; acquisition skipped",
                    )

                recording = MeasurementRecording.start(
                    object_id=request.object_id,
                    routine_id=request.routine_id,
                    probe_point_id=request.probe_point_id,
                    sensor_id=attachment.sensor_id,
                    attachment_revision=attachment.attachment_revision,
                    started_at_ns=self._start_time_ns(),
                    configured_channels=tuple(definition.channels),
                )
                self.measurements.create(recording)
                required = physical or frozenset(
                    channel.channel_id for channel in definition.channels
                )
                session = _Session(
                    recording=recording,
                    required_channels=required,
                    physical_channels=physical,
                    client=client,
                )
                self._session = session
                self._set_state(
                    SensorAcquisitionStatus.STARTING,
                    attachment.sensor_id,
                    "Starting sensor acquisition",
                )
                self._create_sources(session, definition, request)
                for source in session.sources:
                    source.start()
                session.deadline = time.monotonic() + self.START_TIMEOUT_SEC
                session.timer = self.node.create_timer(
                    0.1,
                    self._check_timeout,
                    callback_group=self.callback_group,
                )
                if physical:
                    self._call_head(session, True, self._head_started)
                else:
                    session.head_started = True
            except Exception as exception:
                if self._session is not None:
                    self._finish(
                        self._session,
                        MeasurementCompletionState.FAILED,
                        str(exception),
                    )
                else:
                    if client is not None:
                        self.node.destroy_client(client)
                    self._set_state(
                        SensorAcquisitionStatus.FAILED,
                        detail=str(exception),
                    )
            return self._state

    def stop(self):
        """Stop acquisition and wait for the sensor-head acknowledgement."""
        with self._lock:
            if self._session is None:
                if self._state.status is SensorAcquisitionStatus.FAILED:
                    return self._set_state(SensorAcquisitionStatus.IDLE)
                return self._state
            final_state = (
                MeasurementCompletionState.COMPLETE
                if self._state.status is SensorAcquisitionStatus.RECORDING
                else MeasurementCompletionState.CANCELLED
            )
            self._begin_stop(self._session, final_state, "Recording stopped")
            return self._state

    def close(self):
        """Release active resources during application shutdown."""
        with self._lock:
            if self._session is not None:
                session = self._session
                if (
                    session.client is not None
                    and session.client.service_is_ready()
                ):
                    try:
                        session.client.call_async(self._request(False))
                    except Exception as exception:
                        self._report_error(exception)
                self._finish(
                    session,
                    MeasurementCompletionState.CANCELLED,
                    "Application shutdown cancelled acquisition",
                )
            self._listeners.clear()

    def _create_sources(self, session, definition, request):
        geometry = any(
            channel.source_kind is SensorChannelSource.SPOT_GEOMETRY
            for channel in definition.channels
        )
        tf_buffer = None
        if geometry:
            tf_buffer = tf2_ros.Buffer()
            session.tf_listener = tf2_ros.TransformListener(
                tf_buffer,
                self.node,
                spin_thread=False,
            )
        for channel in definition.channels:
            options = dict(
                node=self.node,
                repository=self.measurements,
                recording=session.recording,
                channel=channel,
                on_first_sample=self._sample_received,
                on_error=self._source_error,
                callback_group=self.callback_group,
            )
            if channel.source_kind is SensorChannelSource.ROS_TOPIC:
                source = RosTopicRecordingSource(**options)
            else:
                source = SpotGeometryRecordingSource(
                    **options,
                    tf_buffer=tf_buffer,
                    object_pose_execution=request.object_pose_execution,
                    probe_frame=definition.probe_frame,
                    execution_frame=request.execution_frame,
                    sample_rate_hz=self.GEOMETRY_RATE_HZ,
                )
            session.sources.append(source)

    def _sample_received(self, channel_id, _timestamp_ns):
        with self._lock:
            session = self._session
            if session is None:
                return
            session.ready_channels.add(channel_id)
            self._update_start(session)

    def _head_started(self, session, future):
        with self._lock:
            if self._session is not session:
                return
            success, detail = self._response(future)
            if not success:
                self._begin_stop(
                    session,
                    MeasurementCompletionState.FAILED,
                    detail,
                )
                return
            session.head_started = True
            self._update_start(session)

    def _update_start(self, session):
        if (
            self._state.status is SensorAcquisitionStatus.STARTING
            and session.head_started
            and session.ready_channels & session.required_channels
        ):
            self._set_state(
                SensorAcquisitionStatus.RECORDING,
                session.recording.sensor_id,
                "Measurement recording is active",
            )

    def _begin_stop(self, session, final_state, detail):
        if self._state.status is SensorAcquisitionStatus.STOPPING:
            return
        for source in session.sources:
            source.stop()
        session.final_state = final_state
        session.final_detail = detail
        self._set_state(
            SensorAcquisitionStatus.STOPPING,
            session.recording.sensor_id,
            detail,
        )
        if session.client is None:
            self._finish(session, final_state, detail)
            return
        session.deadline = time.monotonic() + self.STOP_TIMEOUT_SEC
        try:
            self._call_head(session, False, self._head_stopped)
        except Exception as exception:
            self._finish(
                session,
                MeasurementCompletionState.FAILED,
                str(exception),
            )

    def _head_stopped(self, session, future):
        with self._lock:
            if self._session is not session:
                return
            success, detail = self._response(future)
            final_state = session.final_state
            if not success:
                final_state = MeasurementCompletionState.FAILED
            self._finish(
                session,
                final_state,
                session.final_detail if success else detail,
            )

    def _check_timeout(self):
        with self._lock:
            session = self._session
            if session is None or time.monotonic() < session.deadline:
                return
            if self._state.status is SensorAcquisitionStatus.STARTING:
                self._begin_stop(
                    session,
                    MeasurementCompletionState.FAILED,
                    "Timed out waiting for the first sensor sample",
                )
            elif self._state.status is SensorAcquisitionStatus.STOPPING:
                self._finish(
                    session,
                    MeasurementCompletionState.FAILED,
                    "Timed out waiting for the sensor to stop",
                )

    def _finish(self, session, final_state, detail):
        if self._session is not session:
            return
        for source in session.sources:
            source.stop()
        try:
            self.measurements.finalize(
                session.recording,
                final_state,
                max(
                    session.recording.started_at_ns,
                    int(self.node.get_clock().now().nanoseconds),
                ),
            )
        except Exception as exception:
            final_state = MeasurementCompletionState.FAILED
            detail = f"Failed to finalize measurement: {exception}"
        if session.timer is not None:
            self.node.destroy_timer(session.timer)
        if session.tf_listener is not None:
            session.tf_listener.unregister()
        if session.client is not None:
            self.node.destroy_client(session.client)
        self._session = None
        status = (
            SensorAcquisitionStatus.FAILED
            if final_state is MeasurementCompletionState.FAILED
            else SensorAcquisitionStatus.IDLE
        )
        self._set_state(status, detail=detail)

    def _sensor_client(self, sensor_id, physical_channels):
        if not physical_channels:
            return None
        client = self.node.create_client(
            SetSensorAcquisition,
            f"/fault_detector/sensors/{sensor_id}/set_acquisition",
            callback_group=self.callback_group,
        )
        if client.service_is_ready():
            return client
        self.node.destroy_client(client)
        return None

    def _call_head(self, session, enabled, callback):
        future = session.client.call_async(self._request(enabled))
        future.add_done_callback(
            lambda completed: callback(session, completed)
        )

    @staticmethod
    def _request(enabled):
        request = SetSensorAcquisition.Request()
        request.enabled = enabled
        return request

    @staticmethod
    def _response(future):
        try:
            response = future.result()
            return bool(response.success), response.detail
        except Exception as exception:
            return False, str(exception)

    @staticmethod
    def _physical_channels(channels, sensor_id):
        prefix = f"/sensors/{sensor_id}/"
        return frozenset(
            channel.channel_id
            for channel in channels
            if channel.source_kind is SensorChannelSource.ROS_TOPIC
            and channel.topic.startswith(prefix)
        )

    def _start_time_ns(self):
        value = max(
            0,
            int(self.node.get_clock().now().nanoseconds),
            self._last_start_ns + 1,
        )
        self._last_start_ns = value
        return value

    def _set_state(self, status, sensor_id="", detail="Idle"):
        self._state = SensorAcquisitionState(status, sensor_id, detail)
        for listener in tuple(self._listeners):
            try:
                listener(self._state)
            except Exception as exception:
                self._report_error(exception)
        return self._state

    def _source_error(self, channel_id, exception):
        self._report_error(
            RuntimeError(f"Acquisition channel '{channel_id}': {exception}")
        )

    def _report_error(self, exception):
        if self.error_handler is not None:
            self.error_handler(exception)


__all__ = [
    "SensorAcquisitionCoordinator",
    "SensorAcquisitionRequest",
    "SensorAcquisitionState",
    "SensorAcquisitionStatus",
]
