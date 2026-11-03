"""Regression tests for the executor-backed close-surface command."""

import inspect
from types import SimpleNamespace

from py_trees.blackboard import Blackboard
from py_trees.common import Status

import pytest
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.coordinators.probe_finalization_controller import (
    ProbeFinalizationController,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
    semantic_command_to_message,
)
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MoveCloseToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MoveCloseToSurfaceConfig,
    MoveCloseToSurfaceExecution,
    MoveCloseToSurfaceOutcome,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)


def close_surface_intent(target=0.05, tolerance=0.005):
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.target_surface_distance_m = target
    intent.surface_tolerance_m = tolerance
    return intent


def test_operational_intent_carries_surface_distance_and_tolerance():
    command = operational_intent_to_command(close_surface_intent())

    assert command.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert command.target_surface_distance_m == pytest.approx(0.05)
    assert command.surface_tolerance_m == pytest.approx(0.005)
    assert command.inspection.object_id == ""
    assert command.inspection.routine_id == ""
    assert command.inspection.probe_point_id == ""
    assert command.motion_sensor_id == ""


def test_operational_intent_accepts_zero_as_contact_mode():
    command = operational_intent_to_command(
        close_surface_intent(target=0.0)
    )

    assert command.target_surface_distance_m == pytest.approx(0.0)


def test_operational_intent_accepts_zero_tolerance_for_config_fallback():
    command = operational_intent_to_command(
        close_surface_intent(tolerance=0.0)
    )

    assert command.surface_tolerance_m == pytest.approx(0.0)


def test_operational_intent_rejects_negative_surface_distance():
    with pytest.raises(ValueError, match="non-negative"):
        operational_intent_to_command(
            close_surface_intent(target=-0.001)
        )


def test_operational_intent_rejects_negative_surface_tolerance():
    with pytest.raises(ValueError, match="non-negative"):
        operational_intent_to_command(
            close_surface_intent(tolerance=-0.001)
        )


def test_surface_distance_and_tolerance_survive_payload_round_trip():
    command = operational_intent_to_command(
        close_surface_intent(target=0.037, tolerance=0.006)
    )

    restored = semantic_command_from_message(
        semantic_command_to_message(command)
    )

    assert restored.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert restored.target_surface_distance_m == pytest.approx(0.037)
    assert restored.surface_tolerance_m == pytest.approx(0.006)


def execution_for_tolerance(tolerance):
    execution = object.__new__(MoveCloseToSurfaceExecution)
    execution.config = MoveCloseToSurfaceConfig(tolerance_m=0.005)
    execution._command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.03,
        surface_tolerance_m=tolerance,
    )
    return execution


@pytest.mark.parametrize("requested, expected", [
    (0.0, 0.005), (0.002, 0.005), (0.005, 0.005), (0.012, 0.012),
])
def test_positive_target_clamps_tolerance_to_configured_minimum(requested, expected):
    execution = execution_for_tolerance(requested)

    assert execution._surface_tolerance() == pytest.approx(expected)


@pytest.mark.parametrize("requested", [0.0, 0.002, 0.005, 0.012])
def test_contact_target_preserves_requested_backoff(requested):
    execution = execution_for_tolerance(requested)
    execution._command.target_surface_distance_m = 0.0

    assert execution._surface_tolerance() == pytest.approx(requested)


def test_behaviour_tree_registers_close_surface_behaviour():
    source = inspect.getsource(runner.build_command_tree)

    assert "CommandID.MOVE_CLOSE_TO_SURFACE" in source
    assert "MoveCloseToSurfaceBehaviour" in source
    assert "robot_command_resources=robot_command_resources" in source
    assert "close_surface.action_name" not in source


@pytest.mark.parametrize("cancel_running", [False, True])
def test_close_surface_behaviour_is_thin_execution_adapter(monkeypatch, cancel_running):
    command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, object(), 0.03,
    )
    events = []
    execution = SimpleNamespace(
        active=True,
        feedback_message="Approaching surface",
        start=lambda actual: events.append(actual) or MoveCloseToSurfaceOutcome.RUNNING,
        cancel=lambda: events.append("cancel"),
    )
    def complete():
        execution.active = False
        return MoveCloseToSurfaceOutcome.SUCCESS

    execution.poll = complete
    behaviour = MoveCloseToSurfaceBehaviour(
        surface_source=object(), config=MoveCloseToSurfaceConfig(),
    )
    behaviour.executor = object()
    behaviour._execution = execution
    monkeypatch.setattr(Blackboard, "storage", {"/last_command": command})
    behaviour.initialise()
    assert behaviour.update() is Status.RUNNING
    assert events == [command]
    assert behaviour.feedback_message == "Approaching surface"
    if cancel_running:
        behaviour.terminate(Status.INVALID)
        assert events == [command, "cancel"]
    else:
        execution.feedback_message = "Target gap reached"
        assert behaviour.update() is Status.SUCCESS
        assert behaviour.feedback_message == "Target gap reached"
        behaviour.terminate(Status.INVALID)
        assert events == [command]


def test_finalization_keeps_aligned_pose_as_execution_authority():
    source = inspect.getsource(
        ProbeFinalizationController.approve_probe_geometry
    )

    assert "current_probe_pose" not in source
    assert "approve_surface_alignment_pose" in source
    assert "aligned_preapproach_pose_object" in source
    assert "refinement.approve" in source
