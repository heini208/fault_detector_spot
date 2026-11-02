"""Wire lifecycle states preserve correlated owner evidence."""

import pytest
from fault_detector_msgs.msg import CommandStatus

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import CommandController

from fault_detector_spot.application.ros.command_transport import (
    RosCommandTransport,
)
from test_command_ros_transport import FakeNode, OWNER_INSTANCE, make_request, owner_status


@pytest.mark.parametrize("state", [
    CommandStatus.STATE_RUNNING, CommandStatus.STATE_UNKNOWN,
    CommandStatus.STATE_STOP_REQUESTED, CommandStatus.STATE_STOP_UNCONFIRMED,
    CommandStatus.STATE_SUCCEEDED, CommandStatus.STATE_FAILED,
    CommandStatus.STATE_CANCELLED, CommandStatus.STATE_STOP_CONFIRMED,
])
def test_ros_transport_preserves_lifecycle_state_and_owner(state):
    node, controller = FakeNode(), CommandController()
    transport = RosCommandTransport(node, controller)
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)
    idle = state in {CommandStatus.STATE_SUCCEEDED, CommandStatus.STATE_FAILED,
                     CommandStatus.STATE_CANCELLED, CommandStatus.STATE_STOP_CONFIRMED}
    feedback = owner_status(request_id=request.request_id, state=state,
                            execution_idle=idle, detail="Owner observation")
    assert transport.handle_command_status(feedback)
    public = node.publishers["fault_detector/_internal/commands/status"].messages[-1]
    assert public.request_id == request.request_id
    assert public.execution_instance_id == OWNER_INSTANCE
    assert public.status_sequence > 0
    assert public.state == state
    assert public.execution_idle is idle
    assert public.detail == feedback.detail
