"""Saved paths survive transport/recording and expand into ordered tag moves."""

from types import SimpleNamespace
from fault_detector_spot.application.commanding.command_ids import CommandID
import pytest
from unittest.mock import Mock

from py_trees.common import Status
from fault_detector_msgs.msg import TagElement
from fault_detector_spot.application.commanding.command_request import CommandRequest, CommandOrigin, RecordingPolicy
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import CommandManager
from fault_detector_spot.application.recording.semantic_command_codec import (
    serialize_recorded_command, deserialize_recorded_command,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.inspection.execution.saved_probe_motion import saved_probe_command
from fault_detector_spot.inspection.model.models import PreApproachPathPoint
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
from fault_detector_spot.shared.geometry.models import PoseData, QuaternionData
from test_probe_execution_target import inspection_object, sensor
from test_saved_probe_controls import saved_intent
from test_command_request_correlation import FakeClock
from test_command_manager_static_clock import FakeNode, FakeManagerBlackboard


def path_command():
    definition = inspection_object()
    first = PoseData.identity()
    first.position.x = .8
    first.position.y = .15
    first.orientation = QuaternionData(0., 0., .6, .8)
    second = PoseData.identity()
    second.position.x = .5
    definition.routines[0].probe_points[0].pre_approach_path = [
        PreApproachPathPoint("Around housing", first), PreApproachPathPoint("Above bearing", second),
    ]
    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.pose.position.y = 2.
    tag.pose.pose.orientation.w = 1.
    return saved_probe_command(
        saved_intent(25), Mock(load=Mock(return_value=definition)),
        Mock(reference_tag=Mock(return_value=tag)),
        Mock(require_motion_attachment=Mock(return_value=sensor())),
        ProbeSetupMotionCommandFactory(),
    )


def executable_path(command):
    subscriber = CommandSubscriber()
    subscriber.blackboard = SimpleNamespace(command_buffer=[])
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())
    request = CommandRequest.create(
        command=command, client_id="operator", origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    subscriber.fire_request(request)
    return subscriber.blackboard.command_buffer


def test_path_roundtrip_and_bt_order_preserve_full_poses():
    command = path_command()
    assert semantic_command_from_message(semantic_command_to_message(command)) == command
    assert deserialize_recorded_command(serialize_recorded_command(command)).pre_approach_offsets == command.pre_approach_offsets
    steps = executable_path(command)
    assert len(steps) == 3
    assert [step.offset.pose.position.x for step in steps] == [.8, .5, command.offset.position.x]
    assert steps[0].offset.pose.position.y == pytest.approx(.15)
    assert steps[0].offset.pose.orientation.z == .6
    assert steps[0].offset.pose.orientation.w == .8
    assert len({step.request_id for step in steps}) == 1
    assert command.command_id == CommandID.FOLLOW_MOVE_TO_TAG_PATH
    assert all(step.command_id == CommandID.MOVE_ARM_TO_TAG for step in steps)
    assert all(step.motion_sensor_id == command.motion_sensor_id for step in steps)


def test_path_failure_discards_final_and_remaining_points():
    steps = executable_path(path_command())
    manager = CommandManager()
    manager.node = FakeNode(42_000_000_000)
    manager.blackboard = FakeManagerBlackboard(steps[1:])
    manager.blackboard.last_command = steps[0]
    manager.blackboard.command_tree_status = Status.FAILURE
    manager.update()
    assert manager.blackboard.command_buffer == []
    assert manager.blackboard.last_command == steps[0]


def test_tag_tolerance_survives_recording_transport_and_each_path_leg():
    from dataclasses import replace
    command = replace(path_command(), tag_position_tolerance_m=.025, pre_approach_tolerances_m=(.03, .04))
    message = semantic_command_to_message(command)
    assert message.tag_position_tolerance_m == .025
    restored = semantic_command_from_message(message)
    recorded = serialize_recorded_command(restored)
    assert deserialize_recorded_command(recorded).tag_position_tolerance_m == .025
    assert [step.tag_position_tolerance_m for step in executable_path(restored)] == [.03, .04, .025]
    del recorded["tag_position_tolerance_m"]
    assert deserialize_recorded_command(recorded).tag_position_tolerance_m == .01


@pytest.mark.parametrize("tolerance", [0., -.1, float("nan"), float("inf")])
def test_invalid_tag_tolerance_is_rejected(tolerance):
    from dataclasses import replace
    with pytest.raises(ValueError, match="Tag position tolerance"):
        replace(path_command(), tag_position_tolerance_m=tolerance)


def test_operational_move_to_tag_accepts_position_tolerance():
    from fault_detector_msgs.msg import OperationalIntent
    from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_ARM_TO_TAG
    intent.tag.id = 7
    intent.tag.pose.header.frame_id = "body"
    from fault_detector_spot.shared.geometry.movement_frames import OrientationModes
    intent.orientation_mode = next(iter(OrientationModes)).value
    intent.offset.header.frame_id = "body"
    intent.tag_position_tolerance_m = .025
    assert operational_intent_to_command(intent).tag_position_tolerance_m == .025


@pytest.mark.parametrize("tolerances", [(.01,), (.01, 0.), (.01, float("nan"))])
def test_path_tolerance_count_and_values_are_validated(tolerances):
    from dataclasses import replace
    with pytest.raises(ValueError):
        replace(path_command(), pre_approach_tolerances_m=tolerances)


def test_saved_speeds_survive_transport_recording_and_bt_expansion():
    from dataclasses import replace
    command = replace(path_command(), arm_speed_scale=.4, pre_approach_speed_scales=(.7, .2))
    restored = semantic_command_from_message(semantic_command_to_message(command))
    restored = deserialize_recorded_command(serialize_recorded_command(restored))
    restored = replace(restored, motion_sensor_id=command.motion_sensor_id)
    assert [step.arm_speed_scale for step in executable_path(restored)] == [.7, .2, .4]


def test_single_pose_path_uses_existing_tag_execution():
    from dataclasses import replace

    command = replace(path_command(), pre_approach_offsets=(),
                      pre_approach_tolerances_m=(), pre_approach_speed_scales=())
    steps = executable_path(command)
    assert len(steps) == 1
    assert steps[0].command_id == CommandID.MOVE_ARM_TO_TAG
    assert steps[0].offset.pose.position.x == command.offset.position.x


def test_single_tag_move_cannot_silently_include_a_path():
    from dataclasses import replace

    with pytest.raises(ValueError, match="follow-move-to-tag-path"):
        replace(path_command(), command_id=CommandID.MOVE_ARM_TO_TAG)


def test_existing_recorded_tag_paths_load_as_explicit_path_commands():
    command = path_command()
    recorded = serialize_recorded_command(command)
    recorded["command_id"] = CommandID.MOVE_ARM_TO_TAG.value
    restored = deserialize_recorded_command(recorded)
    assert restored.command_id == CommandID.FOLLOW_MOVE_TO_TAG_PATH
    assert restored.pre_approach_offsets == command.pre_approach_offsets
    assert recorded["command_id"] == CommandID.MOVE_ARM_TO_TAG.value
