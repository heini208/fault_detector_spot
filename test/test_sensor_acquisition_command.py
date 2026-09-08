"""Focused checks for recordable sensor acquisition commands."""

from threading import RLock

import pytest

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
    def __init__(self, start_state):
        self.request = None
        self.start_state = start_state

    def start(self, request):
        self.request = request
        return self.start_state


@pytest.mark.parametrize("offline", [False, True])
def test_start_command_waits_for_recording_or_completes_offline_skip(offline):
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
    start_state = SensorAcquisitionState(
        SensorAcquisitionStatus.IDLE if offline else SensorAcquisitionStatus.STARTING,
        detail="Sensor head is offline; acquisition skipped" if offline else "Starting",
    )
    handler.coordinator = FakeCoordinator(start_state)
    handler._lock = RLock()
    handler._live_object = None
    handler._pending_request = None

    handler._execute(request)

    assert handler.coordinator.request.object_id == ""
    assert handler.coordinator.request.object_pose_execution is not None
    if offline:
        assert [status.state for status in handler.controller.statuses] == [
            CommandControllerState.RUNNING,
            CommandControllerState.SUCCEEDED,
        ]
        assert handler.controller.statuses[-1].detail == start_state.detail
        assert handler._pending_request is None
        return

    assert [status.state for status in handler.controller.statuses] == [
        CommandControllerState.RUNNING
    ]

    handler._handle_acquisition_state(SensorAcquisitionState(
        SensorAcquisitionStatus.RECORDING,
        sensor_id="bmm150_probe",
        detail="Measurement recording is active",
    ))

    assert handler.controller.statuses[-1].state is (
        CommandControllerState.SUCCEEDED
    )
