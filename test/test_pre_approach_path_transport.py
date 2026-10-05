"""Saved paths survive transport/recording and expand into ordered tag moves."""

from types import SimpleNamespace
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
    assert all(step.command_id == command.command_id for step in steps)
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
