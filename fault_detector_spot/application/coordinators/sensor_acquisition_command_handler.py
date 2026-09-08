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
from fault_detector_spot.inspection.model.models import PoseData


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

        timer = None

        def execute_once():
            timer.cancel()
            self.node.destroy_timer(timer)
            self._timers.discard(timer)
            self._execute(request)

        timer = self.node.create_timer(
            0.001,
            execute_once,
            callback_group=self.coordinator.callback_group,
        )
        self._timers.add(timer)
        return True

    def close(self) -> None:
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
            self._pending_request = request
        self._report(
            request.request_id,
            CommandControllerState.RUNNING,
            "Sensor acquisition command accepted",
        )
        try:
            if (
                request.command.command_id
                is CommandID.START_SENSOR_RECORDING
            ):
                state = self.coordinator.start(
                    self._start_request(request.command)
                )
            else:
                state = self.coordinator.stop()
        except Exception as exception:
            self._finish(
                request.request_id,
                CommandControllerState.FAILED,
                str(exception),
            )
            return
        # A start can finish immediately when acquisition is safely skipped.
        if state.status is SensorAcquisitionStatus.IDLE:
            self._finish(
                request.request_id,
                CommandControllerState.SUCCEEDED,
                state.detail,
            )
            return
        self._handle_acquisition_state(state)

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
        with self._lock:
            request = self._pending_request
        if request is None:
            return
        if state.status is SensorAcquisitionStatus.FAILED:
            self._finish(
                request.request_id,
                CommandControllerState.FAILED,
                state.detail,
            )
            return
        target = (
            SensorAcquisitionStatus.RECORDING
            if request.command.command_id
            is CommandID.START_SENSOR_RECORDING
            else SensorAcquisitionStatus.IDLE
        )
        if state.status is target:
            self._finish(
                request.request_id,
                CommandControllerState.SUCCEEDED,
                state.detail,
            )

    def _handle_command_status(self, status) -> None:
        with self._lock:
            request = self._pending_request
            cancelled = (
                request is not None
                and status.request_id == request.request_id
                and status.state is CommandControllerState.CANCELLED
            )
            if cancelled:
                self._pending_request = None
        if cancelled:
            self.coordinator.stop()

    def _finish(self, request_id, state, detail) -> None:
        with self._lock:
            request = self._pending_request
            if request is None or request.request_id != request_id:
                return
            self._pending_request = None
        self._report(request_id, state, detail)

    def _report(self, request_id, state, detail) -> None:
        self.controller.handle_execution_status(
            CommandExecutionStatus(
                request_id=request_id,
                state=state,
                detail=detail,
            )
        )


__all__ = ["SensorAcquisitionCommandHandler"]
