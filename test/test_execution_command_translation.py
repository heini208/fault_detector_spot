from types import SimpleNamespace

from builtin_interfaces.msg import Time

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.application.behaviour_tree.commands.wait_command import (
    WaitCommand,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.mapping.commands.map_command import MapCommand
from fault_detector_spot.navigation.commands.waypoint_command import WaypointCommand


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=34))


def subscriber():
    result = CommandSubscriber()
    result.node = SimpleNamespace(get_clock=lambda: FakeClock())
    return result


def test_simple_semantic_command_becomes_simple_execution_command():
    command = SemanticCommand(command_id=CommandID.STAND_UP)

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert type(translated[0]) is ExecutionCommand
    assert translated[0].command_id is CommandID.STAND_UP


def test_wait_semantic_command_becomes_timer_command():
    command = SemanticCommand(
        command_id=CommandID.WAIT_TIME,
        wait_time=2.5,
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], WaitCommand)
    assert translated[0].duration == 2.5


def test_mapping_semantic_command_preserves_map_name():
    command = SemanticCommand(
        command_id=CommandID.START_LOCALIZATION,
        map_name="lab",
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], MapCommand)
    assert translated[0].map_name == "lab"


def test_waypoint_semantic_command_becomes_waypoint_command():
    command = SemanticCommand(
        command_id=CommandID.MOVE_TO_WAYPOINT,
        map_name="lab",
        waypoint_name="inspection_station",
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], WaypointCommand)
    assert translated[0].map_name == "lab"
    assert translated[0].waypoint_name == "inspection_station"
    assert translated[0].goal_pose is None


def test_generic_execution_container_is_removed():
    import fault_detector_spot.application.commanding as commanding

    assert not hasattr(commanding, "GenericCommand")
    assert not hasattr(commanding, "SimpleCommand")


def test_application_commanding_contains_no_bt_execution_modules():
    from pathlib import Path

    commanding_dir = Path(__file__).parents[1] / "fault_detector_spot/application/commanding"
    assert not (commanding_dir / "simple_command.py").exists()
    assert not (commanding_dir / "timer_command.py").exists()
    assert not (commanding_dir / "move_command.py").exists()
    assert not (commanding_dir / "move_relative_command.py").exists()
    assert not (commanding_dir / "move_to_tag_command.py").exists()
