"""Coordinate one multi-channel sensor measurement."""

import time
from dataclasses import dataclass, field
from enum import Enum
from threading import Event, RLock

import tf2_ros
from fault_detector_msgs.srv import SetSensorAcquisition
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from fault_detector_spot.inspection.measurement import (
    MeasurementCompletionState,
    MeasurementRecording,
    validate_measurement_context,
)
from fault_detector_spot.inspection.measurement.ros_topic_recording_source import (
    RosTopicRecordingSource,
)
from fault_detector_spot.inspection.measurement.spot_geometry_recording_source import (
    SpotGeometryRecordingSource,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannelSource,
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
    object_pose_execution: PoseData | None
    execution_frame: str = "odom"

    def validate(self):
        validate_measurement_context(
            self.object_id,
            self.routine_id,
            self.probe_point_id,
        )
        if not self.execution_frame.strip():
            raise ValueError("Execution frame must not be empty")
        if self.object_pose_execution is not None:
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
    stopped_sources: set = field(default_factory=set)
    ready_channels: set = field(default_factory=set)
    required_channels: frozenset = frozenset()
    physical_channels: frozenset = frozenset()
    client: object = None
    head_started: bool = False
    start_pending: bool = False
    stop_generation: int = 0
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
        self._software_stop = Event()
        self._listeners = []
        self._session = None
        # ROS entities survive cancellation until stop confirmation or timeout.
        self._subscriptions = {}
        self._clients = {}
        self._tf_buffer = None
        self._tf_listener = None
        self._last_start_ns = -1
        self._stop_confirmed = True
        self._state = SensorAcquisitionState(SensorAcquisitionStatus.IDLE)
        # Keep the handle alive while the executor can have a timer take queued.
        self._timer = node.create_timer(
            1.0 / self.GEOMETRY_RATE_HZ,
            self._tick,
            callback_group=self.callback_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._timer.cancel()

    @property
    def execution_lock(self):
        """Serialize compound acquisition operations with sensor callbacks."""
        return self._lock

    @property
    def software_stop_latched(self):
        return self._software_stop.is_set()

    def latch_software_stop(self):
        """Fence every acquisition entry immediately, without workflow locks."""
        self._software_stop.set()

    def reset_software_stop(self):
        """Clear admission after the controller's cached shutdown proof.

        While latched no new session can start. Checking the stopped fields
        directly keeps this callback independent of acquisition workflow locks.
        """
        if self._session is not None or not self._stop_confirmed:
            raise RuntimeError("Sensor acquisition stop is unconfirmed")
        self._software_stop.clear()

    @property
    def recording_stopped(self):
        """Whether acquisition ended with confirmed sensor-head shutdown."""
        with self._lock:
            return self._session is None and self._stop_confirmed

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
            if self.software_stop_latched:
                raise RuntimeError("Software stop is latched; sensor acquisition admission is blocked")
            if self._session is not None:
                raise RuntimeError("Sensor acquisition is already active")
            if not self._stop_confirmed:
                raise RuntimeError("Previous sensor acquisition stop is unconfirmed")
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
                if self.software_stop_latched:
                    self._finish(session, MeasurementCompletionState.CANCELLED,
                                 "Software stop cancelled acquisition before sensor startup")
                    return self._state
                for source in session.sources:
                    source.start()
                session.deadline = time.monotonic() + self.START_TIMEOUT_SEC
                self._timer.reset()
                if self.software_stop_latched:
                    self._finish(session, MeasurementCompletionState.CANCELLED,
                                 "Software stop cancelled acquisition before sensor startup")
                    return self._state
                if physical:
                    self._stop_confirmed = False
                    session.start_pending = True
                    self._call_head(session, True, self._head_started)
                else:
                    session.head_started = True
            except Exception as exception:
                if self._session is not None:
                    finish = self._finish if self._stop_confirmed else self._begin_stop
                    finish(self._session, MeasurementCompletionState.FAILED, str(exception))
                else:
                    self._set_state(
                        SensorAcquisitionStatus.FAILED,
                        detail=str(exception),
                    )
            return self._state

    def stop(self, *, cancelled=False):
        """Stop samples and finalize only after sensor-head acknowledgement.

        A software stop aborts the measurement, retaining its samples and
        cancelled metadata. A previously failed attempt stays failed.
        """
        with self._lock:
            cancelled = cancelled or self.software_stop_latched
            if self._session is None:
                if self._state.status is SensorAcquisitionStatus.FAILED and self._stop_confirmed:
                    return self._set_state(SensorAcquisitionStatus.IDLE)
                return self._state
            final_state = (
                MeasurementCompletionState.COMPLETE
                if self._state.status is SensorAcquisitionStatus.RECORDING and not cancelled
                else MeasurementCompletionState.CANCELLED
            )
            self._begin_stop(
                self._session, final_state,
                "Software stop cancelled acquisition" if cancelled else "Recording stopped",
            )
            return self._state

    def abort_recording(self):
        """Finalize a failed attempt only after stopping its acquisition."""
        with self._lock:
            if self._session is None:
                return self._state
            session = self._session
            if self._state.status is SensorAcquisitionStatus.STOPPING:
                session.final_state = MeasurementCompletionState.FAILED
                session.final_detail = "Recording attempt failed"
            else:
                self._begin_stop(session, MeasurementCompletionState.FAILED,
                                 "Recording attempt failed")
            return self._state

    def request_shutdown(self):
        """Close admission and request stopping while replies remain usable."""
        self.latch_software_stop()
        self.stop(cancelled=True)

    def shutdown_status(self):
        with self._lock:
            return self.recording_stopped, self._state.detail

    def close(self, *, stop_unconfirmed=False):
        """Finalize after confirmation or an explicitly reported deadline."""
        self.latch_software_stop()
        with self._lock:
            if self._session is not None and not stop_unconfirmed:
                raise RuntimeError("Sensor acquisition stop confirmation is still pending")
            errors = []
            try:
                if self._session is not None:
                    self._finish(
                        self._session,
                        MeasurementCompletionState.FAILED,
                        "Application shutdown stop-unconfirmed; sensor reconciliation required",
                    )
            except Exception as error:
                errors.append(error)
            self._listeners.clear()
            try:
                self._timer.cancel()
            except Exception as error:
                errors.append(error)
            if errors:
                raise errors[0]

    def _create_sources(self, session, definition, request):
        geometry = any(
            channel.source_kind is SensorChannelSource.SPOT_GEOMETRY
            for channel in definition.channels
        )
        tf_buffer = None
        if geometry:
            if request.object_pose_execution is None:
                raise RuntimeError(
                    "Live inspection object pose is unavailable"
                )
            if self._tf_buffer is None:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(
                    self._tf_buffer, self.node, spin_thread=False,
                )
            tf_buffer = self._tf_buffer
        for channel in definition.channels:
            options = dict(
                node=self.node,
                repository=self.measurements,
                recording=session.recording,
                channel=channel,
                on_first_sample=lambda channel_id, stamp: self._sample_received(session, channel_id, stamp),
                on_error=self._source_error,
                callback_group=self.callback_group,
            )
            if channel.source_kind is SensorChannelSource.ROS_TOPIC:
                self._ensure_subscription(channel)
                source = RosTopicRecordingSource(**options, externally_subscribed=True)
            else:
                source = SpotGeometryRecordingSource(
                    **options,
                    tf_buffer=tf_buffer,
                    object_pose_execution=request.object_pose_execution,
                    probe_frame=definition.probe_frame,
                    execution_frame=request.execution_frame,
                    sample_rate_hz=self.GEOMETRY_RATE_HZ,
                    externally_sampled=True,
                )
            session.sources.append(source)

    def _sample_received(self, session, channel_id, _timestamp_ns):
        with self._lock:
            if self._session is not session:
                return
            session.ready_channels.add(channel_id)
            self._update_start(session)

    def _head_started(self, session, future):
        with self._lock:
            if self._session is not session:
                return
            session.start_pending = False
            success, detail = self._response(future)
            if self._state.status is SensorAcquisitionStatus.STOPPING:
                # An earlier stop reply cannot cover a delayed start reply.
                # Confirm a fresh stop after this start request has resolved.
                try:
                    self._request_head_stop(session)
                except Exception as exception:
                    self._finish(session, MeasurementCompletionState.FAILED, str(exception))
                return
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
            # A later software stop cannot turn an interrupted measurement into
            # a successful one, or hide a failure already being finalized.
            if (final_state is MeasurementCompletionState.FAILED
                    or (final_state is MeasurementCompletionState.CANCELLED
                        and session.final_state is MeasurementCompletionState.COMPLETE)):
                session.final_state = final_state
                session.final_detail = detail
            return
        errors = self._stop_sources(session)
        if errors:
            final_state = MeasurementCompletionState.FAILED
            detail = f"Recording sources could not stop: {'; '.join(errors)}"
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
            self._request_head_stop(session)
        except Exception as exception:
            self._finish(
                session,
                MeasurementCompletionState.FAILED,
                str(exception),
            )

    def _request_head_stop(self, session):
        session.stop_generation += 1
        generation = session.stop_generation
        self._call_head(session, False, lambda current, future: self._head_stopped(
            current, generation, future,
        ))

    def _head_stopped(self, session, generation, future):
        with self._lock:
            if self._session is not session or generation != session.stop_generation:
                return
            success, detail = self._response(future)
            if success and session.start_pending:
                return
            self._stop_confirmed = success
            final_state = session.final_state
            if not success:
                final_state = MeasurementCompletionState.FAILED
            self._finish(
                session,
                final_state,
                session.final_detail if success else detail,
            )

    def _tick(self):
        with self._lock:
            session = self._session
            if session is None:
                return
            for source in session.sources:
                if isinstance(source, SpotGeometryRecordingSource):
                    source.sample()
            self._check_timeout()

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
        if self.software_stop_latched and final_state is MeasurementCompletionState.COMPLETE:
            final_state = MeasurementCompletionState.CANCELLED
            detail = "Software stop cancelled acquisition"
        errors = self._stop_sources(session)
        if errors:
            self._timer.cancel()
            self._set_state(SensorAcquisitionStatus.FAILED,
                            detail=f"Recording sources could not stop: {'; '.join(errors)}")
            raise RuntimeError(self._state.detail)
        try:
            finalized = self.measurements.finalize(
                session.recording,
                final_state,
                max(
                    session.recording.started_at_ns,
                    int(self.node.get_clock().now().nanoseconds),
                ),
            )
            if (
                finalized.completion_state is MeasurementCompletionState.FAILED
                and final_state is not MeasurementCompletionState.FAILED
            ):
                detail = (
                    "Measurement failed: one or more samples could not be saved"
                )
            final_state = finalized.completion_state
        except Exception as exception:
            # Retain the session so stop/shutdown can retry persistence cleanup.
            self._set_state(SensorAcquisitionStatus.FAILED,
                            detail=f"Failed to finalize measurement: {exception}")
            raise
        self._timer.cancel()
        self._session = None
        status = (
            SensorAcquisitionStatus.FAILED
            if final_state is MeasurementCompletionState.FAILED
            else SensorAcquisitionStatus.IDLE
        )
        self._set_state(status, detail=detail)

    @staticmethod
    def _stop_sources(session):
        errors = []
        for index, source in enumerate(session.sources):
            if index in session.stopped_sources:
                continue
            try:
                source.stop()
            except Exception as error:
                errors.append(f"source {index}: {error}")
            else:
                session.stopped_sources.add(index)
        return errors

    def _sensor_client(self, sensor_id, physical_channels):
        if not physical_channels:
            return None
        client = self._clients.get(sensor_id)
        if client is None:
            client = self.node.create_client(
                SetSensorAcquisition,
                f"/fault_detector/sensors/{sensor_id}/set_acquisition",
                callback_group=self.callback_group,
            )
            self._clients[sensor_id] = client
        return client if client.service_is_ready() else None

    def _ensure_subscription(self, channel):
        key = (channel.topic, channel.message_type)
        if key not in self._subscriptions:
            self._subscriptions[key] = self.node.create_subscription(
                get_message(channel.message_type), channel.topic,
                lambda message: self._receive_topic(key, message),
                qos_profile_sensor_data, callback_group=self.callback_group,
            )

    def _receive_topic(self, key, message):
        # A callback already waiting on finalization must not enter a new session.
        session = self._session
        with self._lock:
            if session is None or self._session is not session:
                return
            for channel, source in zip(session.recording.configured_channels, session.sources):
                if (channel.topic, channel.message_type) == key:
                    source.receive_message(message)

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
