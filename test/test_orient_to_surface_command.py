"""Focused regression tests for the orient-to-surface command."""

import inspect
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.behaviours.orient_to_surface_behaviour import (
    OrientToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.commands.orient_to_surface_command import (
    OrientToSurfaceCommand,
)
from fault_detector_spot.ui.manipulation.controls import ManipulationControls


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=34))


def subscriber():
    result = CommandSubscriber()
    result.node = SimpleNamespace(get_clock=lambda: FakeClock())
    return result


def test_public_intent_maps_to_orient_to_surface_command():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_ORIENT_TO_SURFACE

    command = operational_intent_to_command(intent)

    assert command.command_id is CommandID.ORIENT_TO_SURFACE


def test_semantic_command_preserves_bound_motion_sensor():
    command = SemanticCommand(
        command_id=CommandID.ORIENT_TO_SURFACE,
        motion_sensor_id="hall_probe",
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], OrientToSurfaceCommand)
    assert translated[0].motion_sensor_id == "hall_probe"


def test_orient_to_surface_requires_bound_motion_sensor():
    command = SemanticCommand(command_id=CommandID.ORIENT_TO_SURFACE)

    with pytest.raises(ValueError, match="active sensor geometry"):
        subscriber().fire_command_sequence(command)


def test_behaviour_only_dispatches_to_executor():
    behaviour = OrientToSurfaceBehaviour(name="OrientToSurfaceBehaviour")
    command = OrientToSurfaceCommand(
        CommandID.ORIENT_TO_SURFACE,
        Time(),
        "hall_probe",
    )
    marker = object()
    behaviour._last_command = lambda: command
    behaviour.executor = SimpleNamespace(
        orient_to_surface=lambda sensor_id: (
            marker if sensor_id == "hall_probe" else None
        )
    )

    assert behaviour._start_operation() is marker


def test_executor_owns_surface_orientation_calculation_and_guarded_move():
    start = inspect.getsource(ArmMovementExecutor.orient_to_surface)
    resolve = inspect.getsource(
        ArmMovementExecutor._resolve_surface_orientation_target
    )

    assert "self.guarded_probe(" in start
    assert "_resolve_surface_orientation_target" in start
    assert "latest_hand_depth()" in resolve
    assert "estimate_reference_surface_normal(" in resolve
    assert "surface_aligned_probe_orientation(" in resolve
    assert "sensor_probe_frame(sensor_id)" in resolve


def test_ui_button_dispatches_orient_to_surface_intent():
    source = inspect.getsource(
        ManipulationControls.handle_orient_to_surface
    )

    assert "INTENT_ORIENT_TO_SURFACE" in source
    assert "execute_operation(intent)" in source
    assert "show_setup_unavailable" not in source
