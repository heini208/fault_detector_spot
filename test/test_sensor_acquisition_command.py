"""Focused checks for recordable sensor acquisition commands."""

from threading import RLock

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_command_handler import (
    SensorAcquisitionCommandHandler,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionState,
    SensorAcquisitionStatus,
)


class FakeController:
    def __init__(self, request_id):
        self.active_request_id = request_id
        self.statuses = []

    def handle_execution_status(self, status):
        self.statuses.append(status)
        return True


class FakeCoordinator:
    def __init__(self):
        self.request = None

    def start(self, request):
        self.request = request
        return SensorAcquisitionState(SensorAcquisitionStatus.STARTING)


def test_start_command_finishes_only_after_recording_state():
    command = SemanticCommand(
        CommandID.START_SENSOR_RECORDING,
        inspection=InspectionSelection(),
    )
    request = CommandRequest.create(
        command=command,
        client_id="test",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    handler = SensorAcquisitionCommandHandler.__new__(
        SensorAcquisitionCommandHandler
    )
    handler.controller = FakeController(request.request_id)
    handler.coordinator = FakeCoordinator()
    handler._lock = RLock()
    handler._live_object = None
    handler._pending_request = None

    handler._execute(request)

    assert [status.state for status in handler.controller.statuses] == [
        CommandControllerState.RUNNING
    ]
    assert handler.coordinator.request.object_id == ""
    assert handler.coordinator.request.object_pose_execution is not None

    handler._handle_acquisition_state(SensorAcquisitionState(
        SensorAcquisitionStatus.RECORDING,
        sensor_id="bmm150_probe",
        detail="Measurement recording is active",
    ))

    assert handler.controller.statuses[-1].state is (
        CommandControllerState.SUCCEEDED
    )
