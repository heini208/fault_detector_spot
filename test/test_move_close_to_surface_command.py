"""Regression tests for the executor-backed close-surface command."""

import inspect

import pytest
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
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
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MoveCloseToSurfaceBehaviour,
    MoveCloseToSurfaceConfig,
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


def test_bt_command_accepts_distance_and_tolerance():
    command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.03,
        surface_tolerance_m=0.004,
    )

    assert command.target_surface_distance_m == pytest.approx(0.03)
    assert command.surface_tolerance_m == pytest.approx(0.004)


def test_behaviour_uses_command_tolerance_when_supplied():
    behaviour = MoveCloseToSurfaceBehaviour(
        surface_source=object(),
        config=MoveCloseToSurfaceConfig(tolerance_m=0.005),
    )
    behaviour._command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.03,
        surface_tolerance_m=0.002,
    )

    assert behaviour._surface_tolerance() == pytest.approx(0.002)


def test_behaviour_falls_back_to_configured_tolerance():
    behaviour = MoveCloseToSurfaceBehaviour(
        surface_source=object(),
        config=MoveCloseToSurfaceConfig(tolerance_m=0.005),
    )
    behaviour._command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.03,
        surface_tolerance_m=0.0,
    )

    assert behaviour._surface_tolerance() == pytest.approx(0.005)


def test_command_subscriber_builds_close_surface_command():
    source = inspect.getsource(CommandSubscriber._move_close_to_surface)

    assert "MoveCloseToSurfaceCommand" in source
    assert "target_surface_distance_m" in source
    assert "surface_tolerance_m" in source
    assert "aligned_preapproach_distance_m" not in source


def test_behaviour_tree_registers_close_surface_behaviour():
    source = inspect.getsource(runner.build_command_tree)

    assert "CommandID.MOVE_CLOSE_TO_SURFACE" in source
    assert "MoveCloseToSurfaceBehaviour" in source
    assert "robot_command_resources=robot_command_resources" in source
    assert "close_surface.action_name" not in source


def test_close_surface_is_direct_arm_workflow_without_operation_class():
    source = inspect.getsource(MoveCloseToSurfaceBehaviour)

    assert issubclass(MoveCloseToSurfaceBehaviour, ArmMovementBehaviour)
    assert "get_probe_surface_source(" in source
    assert "guarded_probe(" in source
    assert ".probe(" in source
    assert "ArmMovementOutcome.CONTACT" in source
    assert "MoveCloseToSurfaceOperation" not in source
    assert "MoveCloseToSurface.Goal" not in source
    assert "WorkflowActionBehaviour" not in source


def test_finalization_keeps_aligned_pose_as_execution_authority():
    source = inspect.getsource(
        ProbeFinalizationController.approve_probe_geometry
    )

    assert "current_probe_pose" not in source
    assert "approve_surface_alignment_pose" in source
    assert "aligned_preapproach_pose_object" in source
    assert "refinement.approve" in source
