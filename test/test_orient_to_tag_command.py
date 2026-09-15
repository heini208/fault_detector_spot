"""Focused regression tests for the orient-to-tag command."""

from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest
import py_trees
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
    SemanticTag,
    StampedPose,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.behaviours.orient_to_tag_behaviour import (
    OrientToTagBehaviour,
)
from fault_detector_spot.manipulation.commands.orient_to_tag_command import (
    OrientToTagCommand,
)
from fault_detector_spot.ui.manipulation.controls import ManipulationControls


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=34))


def subscriber():
    result = CommandSubscriber()
    result.node = SimpleNamespace(get_clock=lambda: FakeClock())
    return result


def test_public_intent_maps_to_orient_to_tag_command():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_ORIENT_TO_TAG
    intent.tag.id = 7
    intent.tag.pose.header.frame_id = "body"

    command = operational_intent_to_command(intent)

    assert command.command_id is CommandID.ORIENT_TO_TAG
    assert command.tag.id == 7


def test_semantic_command_preserves_tag_and_bound_motion_sensor():
    command = SemanticCommand(
        command_id=CommandID.ORIENT_TO_TAG,
        tag=SemanticTag(
            id=7,
            pose=StampedPose(frame_id="body"),
        ),
        motion_sensor_id="hall_probe",
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], OrientToTagCommand)
    assert translated[0].tag_id == 7
    assert translated[0].motion_sensor_id == "hall_probe"


def test_behaviour_only_dispatches_to_executor():
    behaviour = OrientToTagBehaviour(name="OrientToTagBehaviour")
    command = OrientToTagCommand(
        CommandID.ORIENT_TO_TAG,
        Time(),
        tag_id=7,
        motion_sensor_id="hall_probe",
    )
    marker = object()
    behaviour._last_command = lambda: command
    behaviour.executor = SimpleNamespace(
        orient_to_tag=lambda tag_id, sensor_id: (
            marker
            if tag_id == 7 and sensor_id == "hall_probe"
            else None
        )
    )

    assert behaviour._start_operation() is marker


def pose(position=(0.4, 0.1, 0.3), angles=(0, 0, 0)):
    result = PoseStamped()
    result.header.frame_id = "body"
    (result.pose.position.x, result.pose.position.y,
     result.pose.position.z) = position
    q = Rotation.from_euler("xyz", angles, degrees=True).as_quat()
    (result.pose.orientation.x, result.pose.orientation.y,
     result.pose.orientation.z, result.pose.orientation.w) = q
    return result


def rotation(message):
    q = message.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def executor_with_geometry():
    # Exercise target resolution without creating ROS clients or moving hardware.
    executor = object.__new__(ArmMovementExecutor)
    tag = SimpleNamespace(pose=pose(angles=(15, 25, 35)))
    executor.tag_state_source = SimpleNamespace(
        reachable_tag=Mock(return_value=tag),
    )
    current = pose(angles=(-10, 5, 20))
    mounting = pose((0.12, -0.04, 0.08), (10, 20, 30)).pose
    executor.probe_motion_planner = SimpleNamespace(
        normalize_target=Mock(return_value=tag.pose),
        hand_to_probe_pose=Mock(return_value=mounting),
        current_pose=Mock(return_value=current),
    )
    return executor, tag, current, mounting


def test_executor_preserves_probe_origin_and_compensates_mount():
    from fault_detector_spot.manipulation.probe_motion_planner import (
        ProbeMotionPlanner,
    )
    from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME

    executor, tag, current, mounting = executor_with_geometry()
    original = deepcopy(current)
    target, sensor_id = executor._resolve_tag_orientation_target(7, "hall_probe")

    assert sensor_id == "hall_probe"
    assert target.pose.position == original.pose.position
    assert current == original
    executor.tag_state_source.reachable_tag.assert_called_once_with(7)
    executor.probe_motion_planner.normalize_target.assert_called_once_with(
        tag.pose, GRAV_ALIGNED_BODY_FRAME_NAME,
    )
    executor.probe_motion_planner.current_pose.assert_called_once_with(
        GRAV_ALIGNED_BODY_FRAME_NAME, "hall_probe_probe",
    )
    # Probe +X points along tag -Z; probe +Y follows tag +Y.
    expected = rotation(tag.pose.pose) @ np.array(
        [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    )
    np.testing.assert_allclose(rotation(target.pose), expected, atol=1e-9)
    hand = ProbeMotionPlanner.probe_pose_to_hand_pose(target, mounting).pose
    actual_probe = (
        np.array([hand.position.x, hand.position.y, hand.position.z])
        + rotation(hand) @ np.array([0.12, -0.04, 0.08])
    )
    np.testing.assert_allclose(actual_probe, [0.4, 0.1, 0.3], atol=1e-9)
    np.testing.assert_allclose(
        rotation(hand) @ rotation(mounting), expected, atol=1e-9,
    )


def test_executor_dispatches_target_builder_through_guard():
    executor, _, _, _ = executor_with_geometry()
    marker = object()
    executor.guarded_probe = Mock(return_value=marker)
    result = executor.orient_to_tag(
        7, "hall_probe", speed=0.1, force_threshold_n=5,
    )
    assert result is marker
    args, kwargs = executor.guarded_probe.call_args
    assert kwargs == {"speed": 0.1, "force_threshold_n": 5}
    target, sensor_id = args[0]()
    assert sensor_id == "hall_probe"
    assert target.pose.position.x == 0.4


@pytest.mark.parametrize("stale", [False, True])
def test_missing_or_stale_tag_rejects_target(stale):
    from fault_detector_msgs.msg import TagElement, TagElementArray
    from fault_detector_spot.sensing.tag_state_source import TagStateSource

    executor, _, _, _ = executor_with_geometry()
    node = SimpleNamespace(create_subscription=lambda *args: object())
    source = TagStateSource(node, monotonic_clock=lambda: 10.0)
    if stale:
        tag = TagElement()
        tag.id = 7
        message = TagElementArray()
        message.elements = [tag]
        source._receive_reachable_tags(message)
        source._monotonic_clock = lambda: 12.0
    executor.tag_state_source = source
    with pytest.raises(RuntimeError, match="not currently reachable"):
        executor._resolve_tag_orientation_target(7, "hall_probe")
    executor.probe_motion_planner.current_pose.assert_not_called()


def test_runner_passes_tag_source_on_first_orientation(monkeypatch):
    from fault_detector_spot.application.behaviour_tree import runner

    source = object()
    executor = SimpleNamespace(orient_to_tag=Mock(return_value=object()))
    resources = SimpleNamespace(
        get_arm_movement_executor=Mock(return_value=executor),
    )
    helper = SimpleNamespace(
        slam_helper=object(),
        robot_command_resources=resources,
        tag_state_source=source,
    )
    monkeypatch.setattr(runner, "get_helper_container", lambda node: helper)

    def construct(node, command_id, constructor):
        if command_id == CommandID.ORIENT_TO_TAG:
            return constructor(node)
        return py_trees.behaviours.Success(name=command_id.value)

    monkeypatch.setattr(runner, "make_simple_command_sequence", construct)
    node = object()
    tree = runner.build_command_tree(node)
    behaviour = next(
        child for child in tree.children
        if isinstance(child, OrientToTagBehaviour)
    )
    behaviour.node = node
    behaviour._last_command = lambda: OrientToTagCommand(
        CommandID.ORIENT_TO_TAG, Time(), 7, "hall_probe",
    )
    behaviour._ensure_executor()
    behaviour._start_operation()
    resources.get_arm_movement_executor.assert_called_once_with(
        node, tag_state_source=source, robot_name="",
    )
    executor.orient_to_tag.assert_called_once_with(7, "hall_probe")


def test_ui_button_dispatches_selected_tag_intent():
    tag = SimpleNamespace(pose=pose())
    control = SimpleNamespace(
        tag_dropdown=SimpleNamespace(currentText=lambda: "7"),
        ui=SimpleNamespace(visible_tags={7: tag}, execute_operation=Mock()),
    )
    control.add_tag_element_to_intent = lambda intent: (
        ManipulationControls.add_tag_element_to_intent(control, intent)
    )
    ManipulationControls.handle_orient_to_tag(control)
    intent = control.ui.execute_operation.call_args.args[0]
    assert intent.intent == OperationalIntent.INTENT_ORIENT_TO_TAG
    assert intent.tag.id == 7
    assert intent.tag.pose == tag.pose
