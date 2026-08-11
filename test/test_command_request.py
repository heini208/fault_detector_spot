"""Tests for application command request contracts."""

from dataclasses import FrozenInstanceError

import pytest
from fault_detector_msgs.msg import CommandRequest as CommandRequestMessage

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)


def make_request(**changes):
    values = {
        "command": {"command_id": "stand_up"},
        "client_id": "operator_ui",
        "context_id": "",
        "origin": CommandOrigin.OPERATIONAL,
        "recording_policy": RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    }
    values.update(changes)
    return CommandRequest.create(**values)


def test_create_assigns_request_identity_and_preserves_contract():
    command = {"command_id": "move_to_waypoint"}

    request = make_request(
        command=command,
        client_id=" remote_ui ",
        context_id=" navigation_7 ",
    )

    assert validate_request_id(request.request_id) == request.request_id
    assert request.client_id == "remote_ui"
    assert request.context_id == "navigation_7"
    assert request.origin is CommandOrigin.OPERATIONAL
    assert (
        request.recording_policy
        is RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
    )
    assert request.command is command


def test_numeric_ros_values_are_normalized_to_enums():
    request = make_request(
        origin=CommandOrigin.SYSTEM.value,
        recording_policy=RecordingPolicy.EXCLUDE.value,
    )

    assert request.origin is CommandOrigin.SYSTEM
    assert request.recording_policy is RecordingPolicy.EXCLUDE


@pytest.mark.parametrize("origin", list(CommandOrigin))
def test_command_origin_values_match_ros_contract(origin):
    constant = getattr(CommandRequestMessage, f"ORIGIN_{origin.name}")

    assert origin.value == constant


@pytest.mark.parametrize("policy", list(RecordingPolicy))
def test_recording_policy_values_match_ros_contract(policy):
    constant = getattr(
        CommandRequestMessage,
        f"RECORDING_POLICY_{policy.name}",
    )

    assert policy.value == constant


@pytest.mark.parametrize(
    "origin",
    [
        CommandOrigin.PROBE_SETUP,
        CommandOrigin.NAVIGATION_SETUP,
        CommandOrigin.INTERNAL,
    ],
)
def test_setup_and_internal_commands_cannot_be_recordable(origin):
    with pytest.raises(ValueError, match="must be excluded"):
        make_request(origin=origin)


def test_playback_commands_can_extend_active_recording():
    request = make_request(origin=CommandOrigin.PLAYBACK)

    assert (
        request.recording_policy
        is RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
    )


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"client_id": "  "}, "Client ID must not be empty"),
        ({"origin": CommandOrigin.UNSPECIFIED}, "origin must be specified"),
        (
            {"recording_policy": RecordingPolicy.UNSPECIFIED},
            "recording policy must be specified",
        ),
        ({"command": None}, "Command must not be None"),
    ],
)
def test_invalid_contract_fields_are_rejected(changes, message):
    with pytest.raises(ValueError, match=message):
        make_request(**changes)


def test_request_is_immutable():
    request = make_request()

    with pytest.raises(FrozenInstanceError):
        request.client_id = "another_client"
