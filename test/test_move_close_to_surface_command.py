"""Regression tests for the standalone close-to-surface command boundary."""

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
from fault_detector_spot.manipulation.behaviours.manipulator_move_close_to_surface_action import (
    ManipulatorMoveCloseToSurfaceAction,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)


def test_operational_intent_requires_only_positive_surface_distance():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.target_surface_distance_m = 0.05

    command = operational_intent_to_command(intent)

    assert command.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert command.target_surface_distance_m == pytest.approx(0.05)
    assert command.inspection.object_id == ""
    assert command.inspection.routine_id == ""
    assert command.inspection.probe_point_id == ""
    assert command.motion_sensor_id == ""


def test_operational_intent_rejects_invalid_surface_distance():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE

    with pytest.raises(ValueError, match="Target surface distance"):
        operational_intent_to_command(intent)


def test_surface_distance_survives_command_payload_round_trip():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.target_surface_distance_m = 0.037
    command = operational_intent_to_command(intent)

    restored = semantic_command_from_message(
        semantic_command_to_message(command)
    )

    assert restored.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert restored.target_surface_distance_m == pytest.approx(0.037)


def test_bt_command_rejects_non_positive_distance():
    with pytest.raises(ValueError, match="Target surface distance"):
        MoveCloseToSurfaceCommand(
            CommandID.MOVE_CLOSE_TO_SURFACE,
            stamp=object(),
            target_surface_distance_m=0.0,
        )


def test_command_subscriber_builds_close_surface_command():
    source = inspect.getsource(CommandSubscriber)

    assert "CommandID.MOVE_CLOSE_TO_SURFACE" in source
    assert "MoveCloseToSurfaceCommand" in source
    assert "target_surface_distance_m" in source


def test_behaviour_tree_registers_close_surface_action():
    source = inspect.getsource(runner.build_command_tree)

    assert "CommandID.MOVE_CLOSE_TO_SURFACE" in source
    assert "ManipulatorMoveCloseToSurfaceAction" in source


def test_close_surface_behavior_has_no_probe_setup_context_dependency():
    source = inspect.getsource(ManipulatorMoveCloseToSurfaceAction)

    for forbidden in (
        "context_id",
        "ProbeSetup",
        "object_id",
        "routine_id",
        "reference_view",
    ):
        assert forbidden not in source
    assert "target_surface_distance_m" in source
    assert "active_attachment" in source
    assert "surface_distance_samples" in source
    assert "end_effector_force" in source


def test_finalization_keeps_aligned_pose_as_execution_authority():
    source = inspect.getsource(
        ProbeFinalizationController.approve_verified_probe
    )

    assert "current_probe_pose" not in source
    assert "approve_surface_alignment_pose" in source
    assert "aligned_preapproach_pose_object" in source
    assert "refinement.approve" in source
