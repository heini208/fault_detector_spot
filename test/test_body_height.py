"""Offline height selection and native command isolation regressions."""

import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication
from bosdyn.api import robot_command_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.recording.semantic_command_codec import (
    serialize_recorded_command, deserialize_recorded_command,
)
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.navigation.base_movement_executor import BaseMovementExecutor, BaseMovementOutcome
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.navigation.body_height_readiness import BodyHeightReadiness, BodyHeightSample
from fault_detector_spot.navigation.behaviours.change_body_height_behaviour import ChangeBodyHeightBehaviour
from fault_detector_spot.ui.navigation.base_movement_controls import BaseMovementControls


def native_goal(goal):
    native = robot_command_pb2.RobotCommand()
    convert(goal.command, native)
    return native


def height_of(command):
    params = spot_command_pb2.MobilityParams()
    assert command.synchronized_command.mobility_command.params.Unpack(params)
    return params.body_control.base_offset_rt_footprint.points[0].pose.position.z


def test_slider_only_submits_on_click_and_height_survives_pipeline():
    app = QApplication.instance() or QApplication([])
    controls = object.__new__(BaseMovementControls)
    submitted = []
    controls.ui = SimpleNamespace(execute_operation=submitted.append)
    row = controls._make_body_height_row()
    controls.body_height_slider.setValue(-13)
    assert submitted == []
    controls.change_height_button.click()
    assert len(submitted) == 1
    intent = submitted[0]
    semantic = operational_intent_to_command(intent)
    semantic = semantic_command_from_message(semantic_command_to_message(semantic))
    semantic = deserialize_recorded_command(serialize_recorded_command(semantic))
    assert semantic.command_id is CommandID.CHANGE_BODY_HEIGHT
    assert semantic.body_height_m == pytest.approx(-0.13)
    assert semantic.offset.position.x == semantic.offset.position.y == 0
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: intent.offset.header.stamp
    execution = subscriber.fire_command_sequence(semantic)[0]
    behaviour = ChangeBodyHeightBehaviour("height")
    behaviour._last_command = lambda: execution
    behaviour.executor = Mock()
    behaviour._start_operation()
    behaviour.executor.change_height.assert_called_once_with(-0.13)


@pytest.mark.parametrize("height", [-0.21, 0.21, float("nan"), float("inf")])
def test_invalid_height_rejected_before_submission(height):
    with pytest.raises(ValueError, match="Body height"):
        SemanticCommand(CommandID.CHANGE_BODY_HEIGHT, body_height_m=height)
    client = Mock()
    executor = BaseMovementExecutor(object(), action_client=client)
    with pytest.raises(ValueError, match="Body height"):
        executor.change_height(height)
    client.send_goal_async.assert_not_called()
    assert not executor.active


@pytest.mark.parametrize("height", [-0.2, 0.0, 0.2])
@pytest.mark.parametrize("tag_relative", [False, True])
def test_height_is_stand_only_and_next_walk_uses_nominal_height(height, tag_relative):
    executor = BaseMovementExecutor(object())
    stand = native_goal(executor._build_stand_goal(height))
    mobility = stand.synchronized_command.mobility_command
    assert mobility.HasField("stand_request")
    assert not mobility.HasField("se2_trajectory_request")
    assert not mobility.HasField("se2_velocity_request")
    assert not stand.synchronized_command.HasField("arm_command")
    assert height_of(stand) == pytest.approx(height)
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    source = SimpleNamespace(visible_snapshot=lambda: {7: SimpleNamespace(id=7, pose=target)})
    command = SimpleNamespace(tag_id=7, compute_goal_pose=lambda _: target, walking_profile="")
    planner = executor.motion_planner
    plan = planner.resolve_tag(command, source) if tag_relative else planner.resolve_relative(command)
    walk = native_goal(planner.build_goal(plan, ""))
    assert height_of(walk) == 0.0
    assert height_of(native_goal(executor._build_stand_goal())) == 0.0


def test_height_uses_existing_executor_busy_and_result_lifecycle():
    from test_base_movement_executor import ManualFuture, FakeActionClient, FakeGoalHandle

    send = ManualFuture()
    result = ManualFuture()
    executor = BaseMovementExecutor(
        object(), action_client=FakeActionClient(send),
        height_readiness=measured_height(0.0),
        ros_time_sec=lambda: 10.0,
    )
    assert executor.change_height(0.12).outcome is BaseMovementOutcome.RUNNING
    assert executor.change_height(-0.12).outcome is BaseMovementOutcome.BUSY
    send.set_result(FakeGoalHandle(result))
    executor.poll()
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=True, message="done")))
    assert executor.poll().outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


def measured_height(offset):
    readiness = BodyHeightReadiness(SimpleNamespace(
        sample=lambda: BodyHeightSample(0.5 + offset, 10.0),
    ))
    readiness.nominal_height_m = 0.5
    return readiness


@pytest.mark.parametrize("state", [
    ArmStowState.STOWED, ArmStowState.DEPLOYED, ArmStowState.UNKNOWN, None,
])
@pytest.mark.parametrize("start, height", [
    (0.2, -0.2), (-0.2, 0.2), (0.0, 0.1), (0.1, 0.0),
    (0.0, 0.013), (0.0, 0.0),
])
def test_height_transition_has_constant_rate_unless_arm_stowed(state, start, height):
    from test_base_movement_executor import ManualFuture, FakeActionClient

    client = FakeActionClient(ManualFuture())
    client.send_goal_async = Mock(wraps=client.send_goal_async)
    executor = BaseMovementExecutor(
        object(), action_client=client,
        arm_state_source=SimpleNamespace(stow_state=lambda: state),
        height_readiness=measured_height(start), ros_time_sec=lambda: 10.0,
    )
    executor._commanded_height_m = start
    assert executor.change_height(height).outcome is BaseMovementOutcome.RUNNING
    command = native_goal(client.send_goal_async.call_args.args[0])
    params = spot_command_pb2.MobilityParams()
    assert command.synchronized_command.mobility_command.params.Unpack(params)
    trajectory = params.body_control.base_offset_rt_footprint
    target = trajectory.points[-1]
    duration = target.time_since_reference
    seconds = duration.seconds + duration.nanos * 1e-9
    assert target.pose.position.z == pytest.approx(height)
    if state is ArmStowState.STOWED or start == height:
        assert seconds == 0
        assert len(trajectory.points) == 1
    else:
        assert len(trajectory.points) == 2
        assert trajectory.points[0].pose.position.z == pytest.approx(start)
        assert trajectory.points[0].time_since_reference.ToNanoseconds() == 0
        assert trajectory.pos_interpolation == trajectory_pb2.POS_INTERP_LINEAR
        assert abs(height - start) / seconds == pytest.approx(0.025)
    assert not command.synchronized_command.HasField("arm_command")


@pytest.mark.parametrize("failure", ["missing", "stale", "no_reference"])
@pytest.mark.parametrize("height", [-0.2, 0.0, 0.2])
def test_height_change_uses_commanded_zero_without_tf_or_nominal_reference(failure, height):
    from test_base_movement_executor import ManualFuture, FakeActionClient

    readiness = measured_height(0.1)
    if failure == "missing":
        readiness.source.sample = lambda: None
    elif failure == "no_reference":
        readiness.nominal_height_m = None
    client = FakeActionClient(ManualFuture())
    client.send_goal_async = Mock(wraps=client.send_goal_async)
    executor = BaseMovementExecutor(
        object(), action_client=client, height_readiness=readiness,
        ros_time_sec=lambda: 11.0 if failure == "stale" else 10.0,
    )
    update = executor.change_height(height)
    assert update.outcome is BaseMovementOutcome.RUNNING
    command = native_goal(client.send_goal_async.call_args.args[0])
    params = spot_command_pb2.MobilityParams()
    assert command.synchronized_command.mobility_command.params.Unpack(params)
    points = params.body_control.base_offset_rt_footprint.points
    assert points[-1].pose.position.z == pytest.approx(height)
    if height:
        assert len(points) == 2
        assert points[0].pose.position.z == 0.0
        assert points[-1].time_since_reference.ToNanoseconds() * 1e-9 == pytest.approx(abs(height) / 0.025)
    assert readiness.reset_required


def test_timed_height_goal_requires_explicit_start():
    executor = BaseMovementExecutor(object())
    with pytest.raises(ValueError, match="starting height"):
        executor._build_stand_goal(0.2, duration_sec=16.0)


def test_legacy_recording_defaults_height():
    data = serialize_recorded_command(SemanticCommand(CommandID.STAND_UP))
    data.pop("body_height_m")
    assert deserialize_recorded_command(data).body_height_m == 0.0


def test_nav2_velocity_uses_unchanged_wrapper_defaults_after_height_command():
    # Exercise the actual driver wrapper without a ROS node or robot connection.
    from spot_wrapper.wrapper import SpotWrapper

    wrapper = object.__new__(SpotWrapper)
    wrapper._state = SimpleNamespace()
    wrapper._command_data = SimpleNamespace()
    wrapper._mobility_params = RobotCommandBuilder.mobility_params()
    wrapper._robot = SimpleNamespace(time_sync=SimpleNamespace(endpoint=None))
    wrapper._robot_command = Mock(return_value=(True, "ok", 1))
    executor = BaseMovementExecutor(object())
    wrapper.robot_command(native_goal(executor._build_stand_goal(-0.15)))
    assert height_of(wrapper._robot_command.call_args.args[0]) == pytest.approx(-0.15)
    wrapper.velocity_cmd(v_x=0.1, v_y=0.0, v_rot=0.0)
    velocity = wrapper._robot_command.call_args.args[0]
    assert velocity.synchronized_command.mobility_command.HasField("se2_velocity_request")
    assert height_of(velocity) == 0.0


def command_rig():
    from test_base_movement_executor import ManualFuture, FakeActionClient
    client = FakeActionClient(ManualFuture())
    client.send_goal_async = Mock(wraps=client.send_goal_async)
    return BaseMovementExecutor(object(), action_client=client), client


def finish_command(executor, client, success=True, accepted=True):
    from test_base_movement_executor import ManualFuture, FakeGoalHandle
    result = ManualFuture()
    client.send_future.set_result(FakeGoalHandle(result, accepted=accepted))
    update = executor.poll()
    if accepted:
        result.set_result(SimpleNamespace(result=SimpleNamespace(success=success)))
        update = executor.poll()
    client.send_future = ManualFuture()
    return update


def submitted_height_points(client):
    params = spot_command_pb2.MobilityParams()
    command = native_goal(client.send_goal_async.call_args.args[0])
    assert command.synchronized_command.mobility_command.params.Unpack(params)
    return params.body_control.base_offset_rt_footprint.points


def test_sequential_heights_track_successful_targets_without_tf():
    executor, client = command_rig()
    previous = 0.0
    for height in (-0.2, 0.2, 0.1, -0.2):
        assert executor.change_height(height).outcome is BaseMovementOutcome.RUNNING
        assert executor._commanded_height_m == previous
        points = submitted_height_points(client)
        assert len(points) == 2
        assert points[0].pose.position.z == pytest.approx(previous)
        assert points[1].pose.position.z == pytest.approx(height)
        duration = points[1].time_since_reference.ToNanoseconds() * 1e-9
        assert duration == pytest.approx(abs(height - previous) / 0.025)
        assert finish_command(executor, client).outcome is BaseMovementOutcome.SUCCESS
        assert executor._commanded_height_m == height
        previous = height
    # Stand restores zero without requiring TF confirmation.
    executor.stand()
    assert finish_command(executor, client).outcome is BaseMovementOutcome.SUCCESS
    assert executor._commanded_height_m == 0.0
    executor.change_height(0.1)
    assert submitted_height_points(client)[0].pose.position.z == 0.0


def test_rejected_height_preserves_previous_commanded_offset():
    executor, client = command_rig()
    executor.change_height(-0.2)
    finish_command(executor, client)
    executor.change_height(0.2)
    assert finish_command(executor, client, accepted=False).outcome is BaseMovementOutcome.GOAL_REJECTED
    assert executor._commanded_height_m == -0.2
    executor.change_height(0.1)
    assert submitted_height_points(client)[0].pose.position.z == -0.2


@pytest.mark.parametrize("failure", ["failed", "cancelled"])
def test_interrupted_height_invalidates_estimate_and_stand_recovers(failure):
    from test_base_movement_executor import ManualFuture, FakeGoalHandle
    executor, client = command_rig()
    executor.change_height(0.2)
    if failure == "failed":
        finish_command(executor, client, success=False)
    else:
        result = ManualFuture()
        client.send_future.set_result(FakeGoalHandle(result))
        executor.poll()
        executor.cancel()
        result.set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
        client.send_future = ManualFuture()
    assert executor._commanded_height_m is None
    count = client.send_calls
    assert executor.change_height(-0.2).outcome is BaseMovementOutcome.EXECUTION_ERROR
    assert client.send_calls == count
    executor.stand()
    assert finish_command(executor, client).outcome is BaseMovementOutcome.SUCCESS
    assert executor._commanded_height_m == 0.0
