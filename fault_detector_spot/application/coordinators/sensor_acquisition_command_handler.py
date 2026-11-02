"""Execute sensor recording commands through the acquisition coordinator."""

from threading import RLock

from fault_detector_msgs.msg import LiveInspectionObjectState

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
    CommandExecutionStatus,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionRequest,
    SensorAcquisitionStatus,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from fault_detector_spot.shared.ros.qos_profiles import LIVE_OBJECT_QOS
from fault_detector_spot.shared.geometry.models import (
    PoseData,
)


class SensorAcquisitionCommandHandler:
    """Bridge semantic start/stop commands to one coordinator."""

    COMMAND_IDS = frozenset({
        CommandID.START_SENSOR_RECORDING,
        CommandID.STOP_SENSOR_RECORDING,
    })
    LIVE_OBJECT_TOPIC = "fault_detector/state/live_inspection_object"

    def __init__(self, node, controller, coordinator):
        self.node = node
        self.controller = controller
        self.coordinator = coordinator
        self._lock = RLock()
        self._live_object = None
        self._pending_request = None
        self._execution_started = False
        self._stop_requested = False
        self._execution_failure = ""
        self._closed = False
        self._timers = set()
        self._subscription = node.create_subscription(
            LiveInspectionObjectState,
            self.LIVE_OBJECT_TOPIC,
            self._store_live_object,
            LIVE_OBJECT_QOS,
        )
        coordinator.add_state_listener(self._handle_acquisition_state)
        controller.add_status_listener(self._handle_command_status)

    def dispatch(self, request) -> bool:
        """Schedule a supported command and report whether it was handled."""
        if request.command.command_id not in self.COMMAND_IDS:
            return False
        with self._lock:
            self._pending_request = request
            self._execution_started = False
            self._stop_requested = False
            self._execution_failure = ""
        self._schedule_once(lambda: self._execute(request))
        return True

    def _schedule_once(self, callback):
        """Keep coordinator callbacks outside the controller's status lock."""
        timer = None

        def execute_once():
            with self._lock:
                timer.cancel()
                self.node.destroy_timer(timer)
                self._timers.discard(timer)
                if self._closed:
                    return
            callback()

        with self._lock:
            if self._closed:
                return
            timer = self.node.create_timer(
                0.001,
                execute_once,
                callback_group=self.coordinator.callback_group,
            )
            self._timers.add(timer)

    def close(self) -> None:
        with self._lock:
            self._closed = True
        self.coordinator.remove_state_listener(
            self._handle_acquisition_state
        )
        self.controller.remove_status_listener(self._handle_command_status)
        for timer in tuple(self._timers):
            timer.cancel()
            self.node.destroy_timer(timer)
        self._timers.clear()
        self.node.destroy_subscription(self._subscription)

    def _execute(self, request) -> None:
        if self.controller.active_request_id != request.request_id:
            return
        with self._lock:
            if self._pending_request is not request or self._stop_requested:
                return
        self._report(
            request.request_id,
            CommandControllerState.RUNNING,
            "Sensor acquisition command accepted",
        )
        # Coordinator callbacks take its execution lock before our lock. Keep
        # that order while making startup admission atomic with stop intent.
        with self.coordinator.execution_lock:
            with self._lock:
                if (self._pending_request is not request or self._stop_requested
                        or self._closed):
                    return
                self._execution_started = True
                try:
                    if request.command.command_id is CommandID.START_SENSOR_RECORDING:
                        self.coordinator.start(self._start_request(request.command))
                    else:
                        self.coordinator.stop()
                except Exception as exception:
                    self._execution_failure = str(exception)
        self._process_acquisition_state()

    def _start_request(self, command) -> SensorAcquisitionRequest:
        selection = command.inspection
        live_object = self._current_live_object(selection.object_id)
        object_pose = (
            pose_to_pose_data(live_object.object_pose)
            if live_object is not None
            else None
        )
        if object_pose is None and not selection.object_id:
            object_pose = PoseData.identity()
        return SensorAcquisitionRequest(
            object_id=selection.object_id,
            routine_id=selection.routine_id,
            probe_point_id=selection.probe_point_id,
            object_pose_execution=object_pose,
            execution_frame=(
                live_object.execution_frame
                if live_object is not None
                else "odom"
            ),
        )

    def _current_live_object(self, object_id):
        with self._lock:
            state = self._live_object
        if state is None or not state.has_object_pose:
            return None
        if state.state != LiveInspectionObjectState.LIVE:
            return None
        if object_id and state.object_id != object_id:
            return None
        return state

    def _store_live_object(self, state) -> None:
        with self._lock:
            self._live_object = state

    def _handle_acquisition_state(self, state) -> None:
        # This listener runs under the coordinator lock. Reporting directly to
        # the controller would invert the lock order used to signal a stop.
        with self._lock:
            if (self._pending_request is not None and self._execution_started
                    and state.status is SensorAcquisitionStatus.FAILED):
                self._execution_failure = (self._execution_failure or state.detail
                                           or "Sensor acquisition failed")
        self._schedule_once(self._process_acquisition_state)

    def _process_acquisition_state(self):
        with self.coordinator.execution_lock:
            state = self.coordinator.snapshot()
            stopped = self.coordinator.recording_stopped
            with self._lock:
                request = self._pending_request
                stopping = self._stop_requested
                started = self._execution_started
                failure = self._execution_failure
        if request is None or (not started and not stopping):
            return
        if stopping:
            if stopped:
                outcome = CommandControllerState.STOP_CONFIRMED
            elif failure or state.status is SensorAcquisitionStatus.FAILED:
                outcome = CommandControllerState.STOP_UNCONFIRMED
            else:
                outcome = CommandControllerState.STOP_REQUESTED
            self._finish(request.request_id, outcome, failure or state.detail,
                         execution_idle=stopped)
        elif failure or state.status is SensorAcquisitionStatus.FAILED:
            outcome = (CommandControllerState.FAILED if stopped
                       else CommandControllerState.UNKNOWN)
            self._finish(request.request_id, outcome, failure or state.detail,
                         execution_idle=stopped)
        elif state.status is SensorAcquisitionStatus.IDLE:
            outcome = (CommandControllerState.SUCCEEDED if stopped
                       else CommandControllerState.UNKNOWN)
            self._finish(request.request_id, outcome, state.detail,
                         execution_idle=stopped)
        elif (request.command.command_id is CommandID.START_SENSOR_RECORDING
              and state.status is SensorAcquisitionStatus.RECORDING):
            # Starting finished; the measurement intentionally spans subsequent
            # commands until a separate STOP_SENSOR_RECORDING request.
            self._finish(request.request_id, CommandControllerState.SUCCEEDED,
                         state.detail, execution_idle=True)

    def _handle_command_status(self, status) -> None:
        if status.state is not CommandControllerState.STOP_REQUESTED:
            return
        with self._lock:
            request = self._pending_request
            if (request is None or status.request_id != request.request_id
                    or self._stop_requested):
                return
            self._stop_requested = True
        self._schedule_once(lambda: self._stop_pending_request(request))

    def _stop_pending_request(self, request):
        with self.coordinator.execution_lock:
            with self._lock:
                if self._pending_request is not request or self._closed:
                    return
                try:
                    self.coordinator.stop(cancelled=True)
                except Exception as exception:
                    self._execution_failure = str(exception)
        self._process_acquisition_state()

    def _finish(self, request_id, state, detail, *, execution_idle) -> None:
        with self._lock:
            request = self._pending_request
            if request is None or request.request_id != request_id:
                return
            stopping = self._stop_requested
            if stopping and state not in {
                CommandControllerState.STOP_CONFIRMED,
                CommandControllerState.STOP_UNCONFIRMED,
                CommandControllerState.STOP_REQUESTED,
            }:
                return
        self._report(request_id, state, detail, execution_idle=execution_idle)
        if execution_idle:
            with self._lock:
                if self._pending_request is request and self._stop_requested == stopping:
                    self._pending_request = None

    def _report(self, request_id, state, detail, *, execution_idle=False) -> None:
        self.controller.handle_execution_status(
            CommandExecutionStatus(
                request_id=request_id,
                state=state,
                detail=detail,
                execution_idle=execution_idle,
            )
        )


__all__ = ["SensorAcquisitionCommandHandler"]
