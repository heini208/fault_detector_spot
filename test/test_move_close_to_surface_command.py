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
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)


def close_surface_intent(target=0.05, aligned=0.20):
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.target_surface_distance_m = target
    intent.aligned_preapproach_distance_m = aligned
    return intent


def test_operational_intent_requires_surface_and_aligned_distances():
    command = operational_intent_to_command(close_surface_intent())

    assert command.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert command.target_surface_distance_m == pytest.approx(0.05)
    assert command.aligned_preapproach_distance_m == pytest.approx(0.20)
    assert command.inspection.object_id == ""
    assert command.inspection.routine_id == ""
    assert command.inspection.probe_point_id == ""
    assert command.motion_sensor_id == ""


def test_operational_intent_accepts_zero_as_contact_mode():
    command = operational_intent_to_command(
        close_surface_intent(target=0.0, aligned=0.20)
    )

    assert command.target_surface_distance_m == pytest.approx(0.0)


def test_operational_intent_rejects_negative_surface_distance():
    with pytest.raises(ValueError, match="non-negative"):
        operational_intent_to_command(
            close_surface_intent(target=-0.001)
        )


def test_operational_intent_rejects_aligned_distance_at_target():
    intent = close_surface_intent(target=0.05, aligned=0.05)

    with pytest.raises(ValueError, match="must exceed target"):
        operational_intent_to_command(intent)


def test_surface_distances_survive_command_payload_round_trip():
    command = operational_intent_to_command(
        close_surface_intent(target=0.037, aligned=0.23)
    )

    restored = semantic_command_from_message(
        semantic_command_to_message(command)
    )

    assert restored.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert restored.target_surface_distance_m == pytest.approx(0.037)
    assert restored.aligned_preapproach_distance_m == pytest.approx(0.23)


def test_bt_command_accepts_zero_distance_as_contact_mode():
    command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.0,
        aligned_preapproach_distance_m=0.20,
    )

    assert command.target_surface_distance_m == 0.0


def test_command_subscriber_builds_close_surface_command():
    source = inspect.getsource(CommandSubscriber)

    assert "CommandID.MOVE_CLOSE_TO_SURFACE" in source
    assert "MoveCloseToSurfaceCommand" in source
    assert "target_surface_distance_m" in source
    assert "aligned_preapproach_distance_m" in source


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
