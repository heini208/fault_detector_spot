"""Preserve execution-owned recovery evidence through BT and public feedback."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.application.controllers.application_controller import ApplicationOperationStatus
from fault_detector_spot.application.controllers.command_controller import CommandControllerState
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from fault_detector_spot.manipulation.behaviours.measure_sensor_tip_distance_behaviour import MeasureSensorTipDistanceBehaviour
from fault_detector_spot.manipulation.behaviours.orient_sensor_tip_to_surface_behaviour import OrientSensorTipToSurfaceBehaviour
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import MeasureSensorTipDistanceCommand
from fault_detector_spot.manipulation.commands.orient_sensor_tip_to_surface_command import OrientSensorTipToSurfaceCommand
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceOutcome,
)


@pytest.mark.parametrize("orientation", [False, True])
@pytest.mark.parametrize("continuation_safe", [False, True])
@pytest.mark.parametrize("still_active", [False, True])
def test_behaviour_publishes_only_stopped_execution_evidence(
    monkeypatch, orientation, continuation_safe, still_active,
):
    command = (OrientSensorTipToSurfaceCommand("orient", Time(), "tip") if orientation
               else MeasureSensorTipDistanceCommand("measure", Time()))
    outcome = (ArmMovementUpdate(ArmMovementOutcome.EXECUTION_ERROR, "No depth") if orientation
               else MeasureSensorTipDistanceOutcome.FAILURE)
    execution = SimpleNamespace(
        active=still_active, continuation_safe=continuation_safe, feedback_message="No depth",
        start=lambda _: outcome, cancel=lambda: None,
    )
    behaviour_type = (OrientSensorTipToSurfaceBehaviour if orientation
                      else MeasureSensorTipDistanceBehaviour)
    behaviour = behaviour_type(surface_source=object(), config=MeasureSensorTipDistanceConfig())
    behaviour.executor = object()
    behaviour._execution = execution
    monkeypatch.setattr(Blackboard, "storage", {"/last_command": command})
    behaviour.initialise()

    assert behaviour.update() is Status.FAILURE
    assert command.continuation_safe is (continuation_safe and not still_active)


@pytest.mark.parametrize("continuation_safe", [False, True])
def test_public_application_feedback_carries_execution_continuation_evidence(continuation_safe):
    operation = SimpleNamespace(
        request_id="request", intent=0,
        request=SimpleNamespace(client_id="ui", context_id="setup"),
    )
    status = ApplicationOperationStatus(
        operation, CommandControllerState.FAILED, "No depth", 0,
        continuation_safe=continuation_safe,
    )
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        _public_state=ApplicationApiNode._public_state,
    )
    message = ApplicationApiNode._state_message(node, status)
    assert message.continuation_safe is continuation_safe
