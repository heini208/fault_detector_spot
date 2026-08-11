from fault_detector_msgs.msg import ApplicationCommandState

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)


def test_controller_states_have_distinct_public_states():
    expected = {
        CommandControllerState.QUEUED: (
            ApplicationCommandState.STATE_QUEUED
        ),
        CommandControllerState.DISPATCHED: (
            ApplicationCommandState.STATE_DISPATCHED
        ),
        CommandControllerState.RUNNING: (
            ApplicationCommandState.STATE_RUNNING
        ),
        CommandControllerState.SUCCEEDED: (
            ApplicationCommandState.STATE_SUCCEEDED
        ),
        CommandControllerState.FAILED: (
            ApplicationCommandState.STATE_FAILED
        ),
        CommandControllerState.CANCELLED: (
            ApplicationCommandState.STATE_CANCELLED
        ),
    }

    actual = {
        state: ApplicationApiNode._public_state(state)
        for state in CommandControllerState
    }

    assert actual == expected
