from types import SimpleNamespace

import pytest
import py_trees
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
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    stamped_pose_to_message,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
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


@pytest.mark.parametrize("bypass", [False, True])
def test_arm_execution_command_preserves_collision_policy(bypass):
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_RELATIVE,
        ignore_environment_collisions=bypass,
    )
    translated = subscriber().fire_command_sequence(command)
    assert translated[0].ignore_environment_collisions is bypass


def test_wait_semantic_command_becomes_timer_command():
    command = SemanticCommand(
        command_id=CommandID.WAIT_TIME,
        wait_time=2.5,
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], WaitCommand)
    assert translated[0].duration == 2.5


@pytest.mark.parametrize("wait_time", [0.0, 2.5])
@pytest.mark.parametrize("bypass", [False, True])
def test_move_and_wait_preserves_motion_and_request_contract(wait_time, bypass):
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG_AND_WAIT,
        tag=SemanticTag(7, StampedPose(
            frame_id="body", position=CommandVector3(0.7, 0.2, 0.4),
            orientation=CommandQuaternion(w=1.0),
        )),
        offset=StampedPose(
            frame_id="Tag_7", position=CommandVector3(z=0.15),
            orientation=CommandQuaternion(w=1.0),
        ),
        orientation_mode="custom_orientation",
        motion_sensor_id="hall_probe",
        arm_speed_scale=0.3,
        tag_position_tolerance_m=0.025,
        precision_mode=True,
        precision_orientation_tolerance_rad=0.04,
        ignore_environment_collisions=bypass,
        wait_time=wait_time,
    )
    request = CommandRequest.create(
        command=command,
        client_id="record_manager",
        context_id="inspection_session",
        origin=CommandOrigin.PLAYBACK,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    owner = subscriber()
    owner.blackboard = SimpleNamespace(command_buffer=[])

    count = owner.fire_request(request)

    steps = owner.blackboard.command_buffer
    expected_ids = [CommandID.MOVE_ARM_TO_TAG]
    if wait_time > 0:
        expected_ids.append(CommandID.WAIT_TIME)
        assert isinstance(steps[-1], WaitCommand)
        assert steps[-1].duration == wait_time
    assert count == len(expected_ids)
    assert [step.command_id for step in steps] == expected_ids
    movement = steps[0]
    assert isinstance(movement, ManipulatorToTagCommand)
    assert movement.tag_id == command.tag.id
    assert movement.tag_pose == stamped_pose_to_message(command.tag.pose)
    assert movement.offset == stamped_pose_to_message(command.offset)
    assert movement.orientation_mode == command.orientation_mode
    assert movement.motion_sensor_id == command.motion_sensor_id
    assert movement.arm_speed_scale == command.arm_speed_scale
    assert movement.tag_position_tolerance_m == command.tag_position_tolerance_m
    assert movement.precision_mode is command.precision_mode
    assert movement.precision_orientation_tolerance_rad == command.precision_orientation_tolerance_rad
    assert command.command_id is CommandID.MOVE_ARM_TO_TAG_AND_WAIT
    for step in steps:
        assert step.semantic_command is command
        assert step.ignore_environment_collisions is bypass
        assert step.request_id == request.request_id
        assert step.client_id == request.client_id
        assert step.context_id == request.context_id
        assert step.origin is request.origin
        assert step.recording_policy is request.recording_policy


def test_move_and_wait_expands_only_to_registered_tree_commands(monkeypatch):
    from fault_detector_spot.application.behaviour_tree import runner

    helper = SimpleNamespace(
        rtabmap_runtime=object(), robot_command_resources=object(), tag_state_source=object(),
    )
    monkeypatch.setattr(runner, "get_helper_container", lambda _node: helper)
    registered_ids = []

    def register_command(_node, command_id, _constructor):
        registered_ids.append(command_id)
        return py_trees.behaviours.Success(name=command_id.value)

    monkeypatch.setattr(runner, "make_simple_command_sequence", register_command)
    runner.build_command_tree(object())
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG_AND_WAIT,
        tag=SemanticTag(7, StampedPose()),
        motion_sensor_id="hall_probe",
        wait_time=2.5,
    )

    steps = subscriber().fire_command_sequence(command)

    assert [step.command_id for step in steps] == [CommandID.MOVE_ARM_TO_TAG, CommandID.WAIT_TIME]
    assert all(registered_ids.count(step.command_id) == 1 for step in steps)


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
