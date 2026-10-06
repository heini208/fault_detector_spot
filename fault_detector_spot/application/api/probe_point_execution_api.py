"""Correlated data and acquisition boundary for the probe-point BT action."""

import time

from fault_detector_spot.inspection.setup.stable_tag_pose import TagObservationUnavailable

from fault_detector_msgs.srv import ProbePointExecutionStep
from rclpy.clock import Clock, ClockType

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import CommandControllerState
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import SensorAcquisitionStatus, SensorAcquisitionRequest
from fault_detector_spot.application.ros.semantic_command_adapter import semantic_command_to_message
from fault_detector_spot.inspection.execution.saved_probe_motion import probe_point_plan


class ProbePointExecutionApi:
    """Keep acquisition in its existing owner; never issue physical motion."""

    SERVICE = "fault_detector/_internal/probe_point_execution"

    def __init__(self, node, controller, setup, acquisition, clock=time.monotonic):
        self.node, self.controller, self.setup = node, controller, setup
        self.acquisition, self.clock = acquisition, clock
        self._lock = acquisition.execution_lock
        self._request = None
        self._plan = None
        self._recording_state = "idle"
        self._deadline = None
        self._detail = ""
        self._service = node.create_service(
            ProbePointExecutionStep, self.SERVICE, self._handle,
            callback_group=acquisition.callback_group,
        )
        self._timer = node.create_timer(
            0.05, self._tick, callback_group=acquisition.callback_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._timer.cancel()
        controller.add_status_listener(self._command_status)
        acquisition.add_state_listener(self._acquisition_state)

    def _command_status(self, status):
        if status.request.command.command_id is not CommandID.EXECUTE_PROBE_POINT:
            return
        with self._lock:
            if status.state is CommandControllerState.DISPATCHED:
                self._request = status.request
                self._plan = None
                self._recording_state = "idle"
                self._deadline = None
            elif (self._request is not None and status.request_id == self._request.request_id
                  and status.state in {CommandControllerState.SUCCEEDED,
                                       CommandControllerState.FAILED, CommandControllerState.CANCELLED}):
                self._timer.cancel()
                if self._recording_state in {"starting", "recording", "stopping", "aborting"}:
                    self.acquisition.stop(cancelled=True)
                self._request = None
                self._plan = None

    def _handle(self, request, response):
        # Check controller correlation before taking the acquisition lock.
        active = self.controller.active_request_id
        with self._lock:
            try:
                if self._request is None or request.request_id != self._request.request_id or active != request.request_id:
                    raise ValueError("Probe execution request is no longer active")
                command = self._request.command
                if request.operation == request.PLAN:
                    if self._plan is None:
                        if self.acquisition.snapshot().status not in {SensorAcquisitionStatus.IDLE}:
                            raise RuntimeError("Stop the current sensor recording before executing a probe point")
                        refinement = self.setup.refinement_controller
                        self._plan = probe_point_plan(
                            command, self.setup.object_repository, self.setup.motion_state_source,
                            refinement.sensor_attachment_controller, refinement.motion_command_factory,
                        )
                    response.plan = [semantic_command_to_message(step) for step in self._plan]
                elif request.operation == request.RECORD:
                    if self._plan is None or self._recording_state not in {"idle", "failed"}:
                        raise RuntimeError("Recording cannot start in the current probe execution state")
                    if not self.acquisition.recording_stopped:
                        raise RuntimeError("Previous acquisition has not confirmed stopping")
                    recording_request = self.recording_request(command)
                    self._deadline = None
                    self._recording_state = "starting"
                    self._detail = "Starting probe-point recording"
                    self.acquisition.start(recording_request)
                    # Skipped acquisition is a failed measurement, not a successful run.
                    if self.acquisition.snapshot().status is SensorAcquisitionStatus.IDLE:
                        self._recording_state = "failed"
                        self._detail = "Sensor acquisition was skipped; no measurement recorded"
                    if self._recording_state in {"starting", "recording", "stopping", "aborting"}:
                        self._timer.reset()
                elif request.operation == request.ABORT_RECORDING:
                    self._timer.cancel()
                    self._recording_state = "aborting"
                    self.acquisition.abort_recording()
                    if self.acquisition.recording_stopped:
                        self._recording_state = "failed"
                    elif self.acquisition.snapshot().status is SensorAcquisitionStatus.FAILED:
                        self._recording_state = "failed"
                elif request.operation != request.POLL:
                    raise ValueError("Unknown probe execution operation")
                response.success = True
            except TagObservationUnavailable as exception:
                response.success = True
                response.recording_state = "waiting_tag"
                response.recording_stopped = self.acquisition.recording_stopped
                response.detail = str(exception)
                return response
            except Exception as exception:
                response.success = False
                self._detail = str(exception)
            response.recording_stopped = self.acquisition.recording_stopped
            response.recording_state = self._recording_state
            response.detail = self._detail
        return response

    def recording_request(self, command):
        selection = command.inspection
        return SensorAcquisitionRequest(
            object_id=selection.object_id,
            routine_id=selection.routine_id,
            probe_point_id=selection.probe_point_id,
            object_pose_execution=self.setup.motion_state_source.object_pose_execution(
                self._plan[0].tag.id,
            ),
            execution_frame="odom",
        )

    def _acquisition_state(self, state):
        with self._lock:
            if self._request is None or self._recording_state not in {"starting", "recording", "stopping", "aborting"}:
                return
            self._detail = state.detail
            if state.status is SensorAcquisitionStatus.RECORDING:
                if self._deadline is None:
                    self._deadline = self.clock() + self._request.command.wait_time
                self._recording_state = "recording"
            elif state.status is SensorAcquisitionStatus.FAILED:
                self._recording_state = "failed"
                self._timer.cancel()
            elif state.status is SensorAcquisitionStatus.IDLE:
                self._recording_state = "complete" if self._recording_state == "stopping" else "failed"
                self._timer.cancel()

    def _tick(self):
        with self._lock:
            if self._recording_state == "recording" and self.clock() >= self._deadline:
                self._recording_state = "stopping"
                self.acquisition.stop()

    def close(self):
        self.controller.remove_status_listener(self._command_status)
        self.acquisition.remove_state_listener(self._acquisition_state)
        self.node.destroy_timer(self._timer)
        self.node.destroy_service(self._service)
