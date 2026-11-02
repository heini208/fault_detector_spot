from fault_detector_msgs.msg import (
    ApplicationCommandState,
    NavigationSetupState,
    ProbeSetupState,
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
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi


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
        CommandControllerState.UNKNOWN: ApplicationCommandState.STATE_UNKNOWN,
        CommandControllerState.STOP_REQUESTED: ApplicationCommandState.STATE_STOP_REQUESTED,
        CommandControllerState.STOP_CONFIRMED: ApplicationCommandState.STATE_STOP_CONFIRMED,
        CommandControllerState.STOP_UNCONFIRMED: ApplicationCommandState.STATE_STOP_UNCONFIRMED,
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
        CommandControllerState.UNKNOWN: NavigationSetupState.STATE_UNKNOWN,
        CommandControllerState.STOP_REQUESTED: NavigationSetupState.STATE_STOP_REQUESTED,
        CommandControllerState.STOP_CONFIRMED: NavigationSetupState.STATE_STOP_CONFIRMED,
        CommandControllerState.STOP_UNCONFIRMED: NavigationSetupState.STATE_STOP_UNCONFIRMED,
    }

    actual = {
        state: NavigationSetupApi._public_state(state)
        for state in CommandControllerState
    }

    assert actual == expected


def test_probe_motion_preserves_uncertainty_and_stop_confirmation():
    for name in ("UNKNOWN", "STOP_REQUESTED", "STOP_CONFIRMED", "STOP_UNCONFIRMED"):
        assert ProbeSetupMotionApi._public_state(getattr(CommandControllerState, name)) == getattr(
            ProbeSetupState, "STATE_" + name,
        )
