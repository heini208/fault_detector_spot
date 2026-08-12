"""Tests for transport-independent command dispatch readiness."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
)


def request(command_id):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="navigation-ui",
        origin=CommandOrigin.NAVIGATION_SETUP,
        recording_policy=RecordingPolicy.EXCLUDE,
    )


def test_request_waits_until_dispatch_transport_is_ready():
    dispatched = []
    ready = {"value": False}
    controller = CommandController(
        dispatch_request=dispatched.append,
        dispatch_ready=lambda: ready["value"],
    )
    pending = request(CommandID.SWAP_MAP)

    controller.submit(pending)

    assert dispatched == []
    assert controller.active_request_id == ""
    assert controller.queued_request_ids == (pending.request_id,)

    ready["value"] = True
    controller.poll()

    assert dispatched == [pending]
    assert controller.active_request_id == pending.request_id


def test_emergency_is_not_lost_when_dispatch_transport_is_unready():
    dispatched = []
    ready = {"value": False}
    controller = CommandController(
        dispatch_request=dispatched.append,
        dispatch_ready=lambda: ready["value"],
    )
    controller.submit(request(CommandID.SWAP_MAP))

    emergency_id = controller.cancel_all("operator-ui")

    assert dispatched == []
    assert controller.active_request_id == ""
    assert controller.queued_request_ids == (emergency_id,)

    ready["value"] = True
    controller.poll()

    assert [item.request_id for item in dispatched] == [emergency_id]
    assert controller.active_request_id == emergency_id
