from fault_detector_msgs.msg import (
    ApplicationCommandState,
    NavigationSetupState,
)

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.api.navigation_setup_api import (
    NavigationSetupApi,
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
        CommandControllerState.FAILED: ApplicationCommandState.STATE_FAILED,
        CommandControllerState.CANCELLED: (
            ApplicationCommandState.STATE_CANCELLED
        ),
    }

    actual = {
        state: ApplicationApiNode._public_state(state)
        for state in CommandControllerState
    }

    assert actual == expected


def test_navigation_setup_states_preserve_terminal_distinctions():
    expected = {
        CommandControllerState.QUEUED: NavigationSetupState.STATE_QUEUED,
        CommandControllerState.DISPATCHED: (
            NavigationSetupState.STATE_RUNNING
        ),
        CommandControllerState.RUNNING: NavigationSetupState.STATE_RUNNING,
        CommandControllerState.SUCCEEDED: (
            NavigationSetupState.STATE_SUCCEEDED
        ),
        CommandControllerState.FAILED: NavigationSetupState.STATE_FAILED,
        CommandControllerState.CANCELLED: (
            NavigationSetupState.STATE_CANCELLED
        ),
    }

    actual = {
        state: NavigationSetupApi._public_state(state)
        for state in CommandControllerState
    }

    assert actual == expected
