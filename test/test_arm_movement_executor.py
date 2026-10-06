"""Focused tests for centralized Cartesian arm movement."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
import fault_detector_spot.manipulation.arm_command_builder as command_builder_module
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    ArmMovementOutcome,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
    HandVelocitySample,
)


class ManualFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._exception = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def set_result(self, result):
        self._result = result
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)

    def set_exception(self, exception):
        self._exception = exception
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
            return
        self._callbacks.append(callback)


class FakeGoalHandle:

    def __init__(self, result_future=None, accepted=True):
        self.accepted = accepted
        self.result_future = result_future or ManualFuture()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return ManualFuture()


class FakeActionClient:

    def __init__(self, send_future=None, server_ready=True):
        self.send_future = send_future or ManualFuture()
        self.server_ready = server_ready
        self.sent_goals = []

    def wait_for_server(self, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return self.server_ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        return self.send_future


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeArmStopServiceClient:

    def __init__(self):
        self.requests = []
        self.future = ManualFuture()

    def wait_for_service(self, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return True

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class FakeArmStateSource:

    def __init__(
        self,
        state,
        last_received_at=0.0,
        stale=False,
    ):
        self.state = state
        self.last_received_at = last_received_at
        self.stale = stale
        self.velocity = None

    def stow_state(self):
        if self.stale:
            return None
        return self.state

    def is_stale(self):
        return self.stale

    def hand_velocity_sample(self):
        return self.velocity


class FakeTransformer:

    def __init__(self, transforms=None):
        self.transforms = transforms or {}
        self.calls = []

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        self.calls.append((target, source, timeout_sec))
        key = (target, source)
        if key not in self.transforms:
            raise AssertionError(
                f"Unexpected TF lookup: {target} <- {source}"
            )
        return self.transforms[key]


class FakeRelativeCommand:

    def __init__(self, target):
        self.target = target
        self.calls = []

    def compute_goal_pose(self, transformer):
        self.calls.append(transformer)
        return deepcopy(self.target)


class FakeTag:

    def __init__(self, tag_id, pose):
        self.id = tag_id
        self.pose = pose


class FakeTagStateSource:

    def __init__(self, tags):
        self.tags = tags
        self.requests = []

    def usable_tag(self, tag_id):
        self.requests.append(tag_id)
        tag = self.tags.get(tag_id)
        return None if tag is None else deepcopy(tag)


class FakeTagCommand:

    def __init__(self, tag_id, sensor_id, probe_target):
        self.tag_id = tag_id
        self.tag_position_tolerance_m = 0.01
        self.motion_sensor_id = sensor_id
        self.tag_pose = PoseStamped()
        self.probe_target = probe_target
        self.calls = []

    def compute_goal_pose(self, transformer):
        self.calls.append(transformer)
        return deepcopy(self.probe_target)


def transform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    result.transform.translation.x = x
    result.transform.translation.y = y
    result.transform.translation.z = z
    result.transform.rotation.z = math.sin(yaw * 0.5)
    result.transform.rotation.w = math.cos(yaw * 0.5)
    return result


def capture_builder(monkeypatch):
    captured = {}

    def build(*args):
        captured["args"] = args
        return object()

    monkeypatch.setattr(
        command_builder_module.RobotCommandBuilder,
        "arm_pose_command",
        build,
    )
    monkeypatch.setattr(
        command_builder_module,
        "convert",
        lambda source, target: None,
    )
    return captured


def hand_velocity_sample(received_at, linear=0.0, angular=0.0):
    return HandVelocitySample(
        received_at=received_at,
        linear_x_mps=linear,
        linear_y_mps=0.0,
        linear_z_mps=0.0,
        angular_x_rad_s=angular,
        angular_y_rad_s=0.0,
        angular_z_rad_s=0.0,
    )


def confirm_physical_stop(executor, clock, first_sample_at):
    executor.arm_state_source.velocity = hand_velocity_sample(first_sample_at)
    clock.now = first_sample_at
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    executor.arm_state_source.velocity = hand_velocity_sample(
        first_sample_at + 0.4
    )
    clock.now = first_sample_at + 0.4
    return executor.poll()


def executor_with_client(transformer, **kwargs):
    client = kwargs.pop("action_client", FakeActionClient())
    kwargs.setdefault("speed_policy", ArmMotionSpeedPolicy(
        default_speed=ArmMotionSpeed(0.10, 0.50),
        minimum_duration_sec=0.50,
    ))
    kwargs.setdefault("ready_forward_distance_m", 0.0)
    kwargs.setdefault("ready_lift_distance_m", 0.10)
    if "arm_state_source" not in kwargs:
        kwargs["arm_state_source"] = FakeArmStateSource(
            ArmStowState.DEPLOYED
        )
    executor = ArmMovementExecutor(
        transformer,
        action_client=client,
        **kwargs,
    )
    return executor, client


def test_relative_uses_current_and_absolute_goal_for_speed(monkeypatch):
    relative = PoseStamped()
    relative.header.frame_id = "hand"
    relative.pose.position.x = 0.10
    relative.pose.orientation.w = 1.0

    current_transform = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=1.0,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): current_transform,
    })
    command = FakeRelativeCommand(relative)
    captured = capture_builder(monkeypatch)
    executor, client = executor_with_client(transformer)

    update = executor.relative(command)

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert command.calls == [transformer]
    assert transformer.calls == [
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
            0.0,
        )
    ]
    assert len(client.sent_goals) == 1
    args = captured["args"]
    assert args[0] == pytest.approx(1.10)
    assert args[7] == executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    assert args[8] == pytest.approx(1.0)


def test_relative_speed_override_uses_same_current_to_goal_path(
    monkeypatch,
):
    relative = PoseStamped()
    relative.header.frame_id = "hand"
    relative.pose.position.x = 0.10
    relative.pose.orientation.w = 1.0

    current_transform = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=1.0,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): current_transform,
    })
    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(transformer)

    executor.relative(
        FakeRelativeCommand(relative),
        speed=ArmMotionSpeed(
            linear_speed_mps=0.05,
            angular_speed_rad_s=0.25,
        ),
    )

    assert captured["args"][8] == pytest.approx(2.0)


def test_absolute_hand_pose_uses_current_to_goal_for_speed(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.4
    target.pose.orientation.w = 1.0

    current = transform(
        "body",
        "hand",
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", "hand"): current,
    })
    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(transformer)

    executor.pose(target)

    assert captured["args"][0] == pytest.approx(0.4)
    assert captured["args"][8] == pytest.approx(2.0)


def test_probe_pose_uses_probe_current_to_goal_for_speed(monkeypatch):
    probe_frame = "hall_probe_probe"
    current_probe = transform(
        "body",
        probe_frame,
        x=0.5,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", probe_frame): current_probe,
        ("hand", probe_frame): hand_to_probe,
    })

    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.7
    target.pose.orientation.w = 1.0

    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(transformer)

    executor.probe_pose(target, "hall_probe")

    assert captured["args"][0] == pytest.approx(0.5)
    assert captured["args"][8] == pytest.approx(2.0)


@pytest.mark.parametrize("speed_scale", [1.0, .25])
def test_tag_probe_forwards_speed_to_probe_motion(monkeypatch, speed_scale):
    tag_pose = PoseStamped()
    tag_pose.header.frame_id = "body"
    tag_pose.pose.orientation.w = 1.0

    probe_target = PoseStamped()
    probe_target.header.frame_id = "body"
    probe_target.pose.position.x = 0.8
    probe_target.pose.orientation.w = 1.0

    probe_frame = "hall_probe_probe"
    current_probe = transform(
        "body",
        probe_frame,
        x=0.6,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", probe_frame): current_probe,
        ("hand", probe_frame): hand_to_probe,
    })
    source = FakeTagStateSource({
        7: FakeTag(7, tag_pose),
    })
    command = FakeTagCommand(
        7,
        "hall_probe",
        probe_target,
    )
    command.arm_speed_scale = speed_scale
    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(
        transformer,
        tag_state_source=source,
    )

    executor.tag_probe(
        command,
        speed=ArmMotionSpeed(
            linear_speed_mps=0.05,
            angular_speed_rad_s=0.25,
        ),
    )

    assert source.requests == [7]
    assert command.calls == [transformer]
    assert captured["args"][0] == pytest.approx(0.6)
    assert captured["args"][7] == "body"
    assert captured["args"][8] == pytest.approx(4.0 / speed_scale)
    verification_target = executor._tag_accuracy["target"].target
    assert verification_target.header.frame_id == "body"
    assert verification_target.pose.position.x == pytest.approx(0.8)
    assert verification_target.pose.position.y == pytest.approx(0.0)


    assert executor._tag_accuracy["speed"].linear_speed_mps == pytest.approx(.05 * speed_scale)

def test_probe_relative_reuses_current_probe_transform(monkeypatch):
    probe_frame = "hall_probe_probe"
    current_probe = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        probe_frame,
        x=1.0,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
        ): current_probe,
        ("hand", probe_frame): hand_to_probe,
    })

    offset = PoseStamped()
    offset.header.frame_id = probe_frame
    offset.pose.position.x = 0.10
    offset.pose.orientation.w = 1.0

    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(transformer)

    executor.probe_relative(
        offset,
        "hall_probe",
    )

    assert transformer.calls == [
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            0.0,
        ),
        ("hand", probe_frame, 0.0),
    ]
    assert captured["args"][0] == pytest.approx(0.9)
    assert captured["args"][8] == pytest.approx(1.0)


def test_bare_hand_probe_pose_uses_hand_speed_path(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.4
    target.pose.orientation.w = 1.0

    current = transform(
        "body",
        "hand",
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", "hand"): current,
    })
    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(transformer)

    executor.probe_pose(target, "hand")

    assert transformer.calls == [
        ("body", "hand", 0.0),
    ]
    assert captured["args"][8] == pytest.approx(2.0)


def test_tag_probe_rejects_unusable_tag_as_typed_failure():
    executor, client = executor_with_client(
        FakeTransformer(),
        tag_state_source=FakeTagStateSource({}),
    )
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    update = executor.tag_probe(
        FakeTagCommand(7, "hand", target)
    )

    assert update.outcome is ArmMovementOutcome.EXECUTION_ERROR
    assert "not currently usable" in update.detail
    assert client.sent_goals == []


def test_public_movement_methods_do_not_accept_duration():
    executor, _ = executor_with_client(FakeTransformer())

    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    with pytest.raises(TypeError):
        executor.pose(target, duration_sec=2.0)


def test_executor_owns_goal_acceptance_and_success_result(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future=result_future)
    client = FakeActionClient(send_future=send_future)
    stop_client = FakeArmStopServiceClient()
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        arm_stop_service_client=stop_client,
    )

    assert executor.pose(target).outcome is ArmMovementOutcome.RUNNING
    send_future.set_result(handle)

    accepted = executor.poll()
    assert accepted.outcome is ArmMovementOutcome.RUNNING
    assert accepted.detail == "Goal accepted"

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    completed = executor.poll()

    assert stop_client.requests == []
    assert handle.cancel_count == 0

    assert completed.outcome is ArmMovementOutcome.SUCCESS
    assert not executor.active


def test_executor_reports_goal_rejection(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    send_future = ManualFuture()
    client = FakeActionClient(send_future=send_future)
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
    )

    executor.pose(target)
    send_future.set_result(
        FakeGoalHandle(accepted=False)
    )
    update = executor.poll()

    assert update.outcome is ArmMovementOutcome.GOAL_REJECTED
    assert not executor.active


def test_goal_response_timeout_cancels_goal_if_accepted_late(
    monkeypatch,
):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    clock = ManualClock()
    send_future = ManualFuture()
    client = FakeActionClient(send_future=send_future)
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        monotonic_clock=clock,
        goal_response_timeout_sec=2.0,
    )

    executor.pose(target)
    clock.now = 2.0
    update = executor.poll()

    assert (
        update.outcome
        is ArmMovementOutcome.GOAL_RESPONSE_TIMEOUT
    )
    handle = FakeGoalHandle()
    send_future.set_result(handle)
    assert handle.cancel_count == 1


def test_result_timeout_requests_goal_cancellation(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future=result_future)
    client = FakeActionClient(send_future=send_future)
    stop_client = FakeArmStopServiceClient()
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        arm_stop_service_client=stop_client,
        monotonic_clock=clock,
        result_timeout_sec=3.0,
    )

    executor.pose(target)
    send_future.set_result(handle)
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING

    clock.now = 3.0
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert handle.cancel_count == 1
    assert len(stop_client.requests) == 1
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    stop_client.future.set_result(SimpleNamespace(success=True))
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    update = confirm_physical_stop(executor, clock, 3.1)

    assert update.outcome is ArmMovementOutcome.RESULT_TIMEOUT
    assert handle.cancel_count == 1
    assert not executor.active


@pytest.mark.parametrize("poll_acceptance", [False, True])
def test_explicit_cancel_requests_goal_cancellation(monkeypatch, poll_acceptance):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future=result_future)
    client = FakeActionClient(send_future=send_future)
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
    )

    executor.pose(target)
    send_future.set_result(handle)
    if poll_acceptance:
        executor.poll()

    executor.cancel()

    assert handle.cancel_count == 1
    assert executor.active
    assert executor.cancelling
    assert executor.pose(target).outcome is ArmMovementOutcome.BUSY


def test_executor_rejects_overlapping_arm_movement(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    transformer = FakeTransformer({
        ("body", "hand"): transform("body", "hand"),
    })
    capture_builder(monkeypatch)

    executor, client = executor_with_client(transformer)

    first = executor.pose(target)
    second = executor.pose(target)

    assert first.outcome is ArmMovementOutcome.RUNNING
    assert second.outcome is ArmMovementOutcome.BUSY
    assert len(client.sent_goals) == 1


def test_stowed_relative_prepares_before_resolving_requested_goal(
    monkeypatch,
):
    relative = PoseStamped()
    relative.header.frame_id = "hand"
    relative.pose.position.x = 0.10
    relative.pose.orientation.w = 1.0

    frame = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    ready_transform = transform(
        frame,
        "hand",
        x=0.2,
        z=0.4,
    )
    deployed_transform = transform(
        frame,
        "hand",
        x=1.0,
        z=0.5,
    )
    transformer = FakeTransformer({
        (frame, "hand"): ready_transform,
    })
    state = FakeArmStateSource(ArmStowState.STOWED)
    ready_send_future = ManualFuture()
    ready_result_future = ManualFuture()
    ready_handle = FakeGoalHandle(
        result_future=ready_result_future
    )
    client = FakeActionClient(
        send_future=ready_send_future
    )
    captured = capture_builder(monkeypatch)
    command = FakeRelativeCommand(relative)
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        arm_state_source=state,
    )

    started = executor.relative(command)

    assert started.outcome is ArmMovementOutcome.RUNNING
    assert command.calls == []
    assert len(client.sent_goals) == 1

    ready_send_future.set_result(ready_handle)
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING

    ready_result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    waiting = executor.poll()

    assert waiting.outcome is ArmMovementOutcome.RUNNING
    assert "deployed" in waiting.detail
    assert command.calls == []

    state.state = ArmStowState.DEPLOYED
    transformer.transforms[(frame, "hand")] = deployed_transform
    client.send_future = ManualFuture()

    resumed = executor.poll()

    assert resumed.outcome is ArmMovementOutcome.RUNNING
    assert command.calls == [transformer]
    assert len(client.sent_goals) == 2
    assert captured["args"][0] == pytest.approx(1.10)


def test_movement_reports_missing_arm_state_after_bounded_wait():
    clock = ManualClock()
    state = FakeArmStateSource(
        None,
        last_received_at=None,
        stale=True,
    )
    executor, client = executor_with_client(
        FakeTransformer(),
        arm_state_source=state,
        monotonic_clock=clock,
        ready_state_timeout_sec=2.0,
    )
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    first = executor.pose(target)

    assert first.outcome is ArmMovementOutcome.RUNNING
    assert client.sent_goals == []

    clock.now = 2.0
    failed = executor.poll()

    assert (
        failed.outcome
        is ArmMovementOutcome.ARM_STATE_UNAVAILABLE
    )
    assert "2.0 s" in failed.detail
    assert client.sent_goals == []
    assert not executor.active


def test_prepare_noops_when_arm_is_already_deployed():
    state = FakeArmStateSource(ArmStowState.DEPLOYED)
    executor, client = executor_with_client(
        FakeTransformer(),
        arm_state_source=state,
    )

    update = executor.prepare()

    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert update.detail == "Arm is already deployed"
    assert client.sent_goals == []
    assert not executor.active


def test_prepare_uses_shared_speed_and_verifies_deployed(monkeypatch):
    current = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=0.2,
        y=-0.1,
        z=0.4,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): current,
    })
    state = FakeArmStateSource(ArmStowState.STOWED)
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future=result_future)
    client = FakeActionClient(send_future=send_future)
    captured = capture_builder(monkeypatch)
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        arm_state_source=state,
    )

    started = executor.prepare()

    assert started.outcome is ArmMovementOutcome.RUNNING
    args = captured["args"]
    assert args[0] == pytest.approx(0.2)
    assert args[1] == pytest.approx(-0.1)
    assert args[2] == pytest.approx(0.5)
    assert args[8] == pytest.approx(1.0)

    send_future.set_result(handle)
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    waiting = executor.poll()
    assert waiting.outcome is ArmMovementOutcome.RUNNING
    assert "deployed" in waiting.detail

    state.state = ArmStowState.DEPLOYED
    completed = executor.poll()

    assert completed.outcome is ArmMovementOutcome.SUCCESS
    assert completed.detail == "Arm deployed"
    assert not executor.active


def test_prepare_reports_missing_arm_state_after_bounded_wait():
    clock = ManualClock()
    state = FakeArmStateSource(
        None,
        last_received_at=None,
        stale=True,
    )
    executor, client = executor_with_client(
        FakeTransformer(),
        arm_state_source=state,
        monotonic_clock=clock,
        ready_state_timeout_sec=2.0,
    )

    first = executor.prepare()
    assert first.outcome is ArmMovementOutcome.RUNNING
    assert client.sent_goals == []

    clock.now = 2.0
    failed = executor.poll()

    assert (
        failed.outcome
        is ArmMovementOutcome.ARM_STATE_UNAVAILABLE
    )
    assert "2.0 s" in failed.detail
    assert not executor.active


def test_stow_noops_when_arm_is_already_stowed():
    state = FakeArmStateSource(ArmStowState.STOWED)
    executor, client = executor_with_client(
        FakeTransformer(),
        arm_state_source=state,
    )

    update = executor.stow()

    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert update.detail == "Arm is already stowed"
    assert client.sent_goals == []
    assert not executor.active


def test_stow_uses_native_command_and_verifies_stowed(monkeypatch):
    state = FakeArmStateSource(ArmStowState.DEPLOYED)
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future=result_future)
    client = FakeActionClient(send_future=send_future)
    captured = {}

    def build_stow():
        captured["called"] = True
        return object()

    monkeypatch.setattr(
        command_builder_module.RobotCommandBuilder,
        "arm_stow_command",
        build_stow,
    )
    monkeypatch.setattr(
        command_builder_module,
        "convert",
        lambda source, target: None,
    )
    executor, _ = executor_with_client(
        FakeTransformer(),
        action_client=client,
        arm_state_source=state,
    )

    started = executor.stow()

    assert started.outcome is ArmMovementOutcome.RUNNING
    assert captured["called"]
    assert len(client.sent_goals) == 1

    send_future.set_result(handle)
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    waiting = executor.poll()
    assert waiting.outcome is ArmMovementOutcome.RUNNING
    assert "stowed" in waiting.detail

    state.state = ArmStowState.STOWED
    completed = executor.poll()

    assert completed.outcome is ArmMovementOutcome.SUCCESS
    assert completed.detail == "Arm stowed"
    assert not executor.active


def test_planning_defers_spot_command_construction_until_execution(monkeypatch):
    frame = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    executor, client = executor_with_client(FakeTransformer({
        (frame, "hand"): transform(frame, "hand"),
    }))
    captured = capture_builder(monkeypatch)
    target = PoseStamped()
    target.header.frame_id = frame
    target.pose.position.x = 0.2
    target.pose.orientation.w = 1.0
    planner = executor.probe_motion_planner

    plan = planner.build_plan(
        lambda: planner.resolved_target(target, "hand")
    )

    assert captured == {}
    assert not hasattr(plan, "goal")
    assert plan.duration_sec == pytest.approx(2.0)
    update = executor.probe(plan)

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert captured["args"][0] == pytest.approx(0.2)
    assert captured["args"][-1] == pytest.approx(plan.duration_sec)


def record_probe_calls(monkeypatch, executor):
    calls = []
    original = executor.probe

    def probe(*args, **kwargs):
        calls.append((deepcopy(args), executor._operation))
        return original(*args, **kwargs)

    monkeypatch.setattr(executor, "probe", probe)
    return calls


def test_contact_retreat_uses_probe_and_preserves_guard_lifecycle(monkeypatch):
    from fault_detector_spot.manipulation.arm_state_source import HandForceSample

    frame = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    tf = FakeTransformer({(frame, "hand"): transform(frame, "hand")})
    clock = ManualClock()
    stop_client = FakeArmStopServiceClient()
    executor, client = executor_with_client(
        tf,
        arm_stop_service_client=stop_client,
        monotonic_clock=clock,
    )
    capture_builder(monkeypatch)
    calls = record_probe_calls(monkeypatch, executor)
    target = PoseStamped()
    target.header.frame_id = frame
    target.pose.position.x = 0.1
    target.pose.orientation.w = 1.0

    assert executor.guarded_probe(
        target, "hand", force_threshold_n=1.0
    ).outcome is ArmMovementOutcome.RUNNING
    assert len(calls) == 1
    assert calls[0][1] == executor_module._ArmOperation.GUARDED_MOVEMENT
    assert executor.probe(target, "hand").outcome is ArmMovementOutcome.BUSY
    calls.pop()  # The rejected public request does not submit a command.
    assert len(client.sent_goals) == 1

    clock.now = 0.1
    executor.guarded_probe_execution.observe_force_sample(
        HandForceSample(clock.now, -10.0, 0.0, 0.0)
    )
    clock.now = 0.2
    executor.guarded_probe_execution.observe_force_sample(
        HandForceSample(clock.now, -10.0, 0.0, 0.0)
    )
    assert len(stop_client.requests) == 1
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    tf.transforms[(frame, "hand")] = transform(frame, "hand", x=0.008)
    stop_client.future.set_result(SimpleNamespace(success=True, message="stopped"))
    client.send_future = ManualFuture()

    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(calls) == 1
    assert (
        confirm_physical_stop(executor, clock, 0.3).outcome
        is ArmMovementOutcome.RUNNING
    )
    assert len(calls) == 2
    assert calls[1][1] == executor_module._ArmOperation.GUARDED_MOVEMENT
    retreat = calls[1][0][0]
    assert retreat.target_hand.pose.position.x == pytest.approx(0.0)
    assert len(client.sent_goals) == 2
    assert executor.active

    retreat_result = ManualFuture()
    client.send_future.set_result(FakeGoalHandle(result_future=retreat_result))
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    retreat_result.set_result(SimpleNamespace(result=SimpleNamespace(success=True)))
    assert executor.poll().outcome is ArmMovementOutcome.CONTACT
    assert len(stop_client.requests) == 1
    assert not executor.active


def test_public_probe_rejects_new_requests_during_preparation(monkeypatch):
    frame = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    executor, client = executor_with_client(
        FakeTransformer({(frame, "hand"): transform(frame, "hand")}),
        arm_state_source=FakeArmStateSource(ArmStowState.STOWED),
    )
    capture_builder(monkeypatch)
    calls = record_probe_calls(monkeypatch, executor)

    assert executor.prepare().outcome is ArmMovementOutcome.RUNNING
    assert len(calls) == 1
    assert calls[0][1] == executor_module._ArmOperation.PREPARE
    assert executor.probe(PoseStamped(), "hand").outcome is ArmMovementOutcome.BUSY
    assert len(client.sent_goals) == 1
    assert executor._operation == executor_module._ArmOperation.PREPARE
    executor.cancel()
    assert executor.active
    assert executor.cancelling


@pytest.mark.parametrize("outcome_name", ["FAILURE", "TIMEOUT", "ERROR"])
def test_guarded_moveit_failure_releases_without_arm_stop(outcome_name):
    from fault_detector_spot.manipulation.moveit_arm_planner import (
        MoveItPlanOutcome, MoveItPlanUpdate,
    )
    from fault_detector_spot.manipulation.guarded_probe_execution import _Phase

    executor, client = executor_with_client(FakeTransformer({}))
    executor.arm_stop_service_client = FakeArmStopServiceClient()
    executor.moveit_arm_planner = SimpleNamespace(
        poll=lambda: MoveItPlanUpdate(
            getattr(MoveItPlanOutcome, outcome_name), "planning failed"
        ),
        cancel=lambda: None,
    )
    executor._active = True
    executor._operation = executor_module._ArmOperation.GUARDED_MOVEMENT
    executor._moveit_cartesian_plan = object()
    guard = executor.guarded_probe_execution
    guard._phase = _Phase.MOVING
    guard._plan = SimpleNamespace(force_guard_enabled=False)

    result = executor.poll()

    assert result.outcome is ArmMovementOutcome.PLANNING_FAILED
    assert "planning failed" in result.detail
    assert not executor.active
    assert not guard.active
    assert executor.arm_stop_service_client.requests == []
    assert client.sent_goals == []
    assert executor.stow().outcome is ArmMovementOutcome.RUNNING


def tag_verification_executor(monkeypatch, error=.02):
    from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget
    frame = executor_module.sensor_probe_frame("sensor_a")
    feedback = transform("odom", frame, x=error)
    feedback.header.stamp.sec = 1
    clock = [0.0]
    executor, _ = executor_with_client(
        FakeTransformer({("odom", frame): feedback}),
        monotonic_clock=lambda: clock[0],
    )
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.
    executor._active = True
    executor._tag_accuracy = {
        "target": ResolvedProbeTarget(target=target, sensor_id="sensor_a"),
        "tolerance": .01, "corrections": 0,
        "speed": ArmMotionSpeed(.1, .5), "force_threshold": 8.,
    }
    corrections = []
    monkeypatch.setattr(executor.probe_motion_planner, "build_plan", lambda builder, speed: (builder(), speed))

    def correct():
        corrections.append(executor._guarded_plan_builder())
        return executor_module.ArmMovementUpdate(ArmMovementOutcome.RUNNING, "correcting")

    monkeypatch.setattr(executor, "_begin_guarded_probe", correct)
    executor._begin_tag_position_verification()
    return executor, feedback, clock, corrections


def test_tag_accuracy_waits_for_fresh_feedback_then_corrects_same_target(monkeypatch):
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch)
    target = executor._tag_accuracy["target"]
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert not corrections
    feedback.header.stamp.sec += 1
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert corrections == [(target, ArmMotionSpeed(.05, .25))]
    assert executor._guarded_force_threshold_n == 8.
    # Successful completion of the one correction resolves immediately, even
    # without another feedback sample or an additional endpoint check.
    result = executor._finish_guarded_update(
        executor_module.ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "done")
    )
    assert result.outcome is ArmMovementOutcome.SUCCESS
    assert len(corrections) == 1
    assert executor._tag_accuracy is None
    assert not executor.active


def test_tag_accuracy_within_tolerance_finishes_without_adjustment(monkeypatch):
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch, error=.005)
    feedback.header.stamp.sec += 1
    assert executor.poll().outcome is ArmMovementOutcome.SUCCESS
    assert not corrections


def test_tag_accuracy_stale_feedback_times_out(monkeypatch):
    executor, _, clock, corrections = tag_verification_executor(monkeypatch, error=0.)
    clock[0] = 2.1
    assert executor.poll().outcome is ArmMovementOutcome.EXECUTION_ERROR
    assert not corrections


def test_tag_accuracy_does_not_retry_guard_failure(monkeypatch):
    executor, _, _, corrections = tag_verification_executor(monkeypatch)
    result = executor._finish_guarded_update(
        executor_module.ArmMovementUpdate(ArmMovementOutcome.MOTION_FAILED, "guard stopped")
    )
    assert result.outcome is ArmMovementOutcome.MOTION_FAILED
    assert not corrections
    assert executor._tag_accuracy is None


def test_tag_accuracy_cancellation_does_not_start_correction(monkeypatch):
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch)
    stopped = []
    monkeypatch.setattr(executor, "_monitor_cancel_stop", lambda: stopped.append(True))
    executor.cancel()
    assert stopped
    assert executor.cancelling
    feedback.header.stamp.sec += 1
    executor.poll()
    assert not corrections


def test_explicit_stop_confirmation_requires_stationary_feedback():
    clock = ManualClock()
    stop_client = FakeArmStopServiceClient()
    executor, _ = executor_with_client(FakeTransformer({}), monotonic_clock=clock,
                                       arm_stop_service_client=stop_client)
    update = executor.confirm_stop()
    assert update.outcome is ArmMovementOutcome.RUNNING
    assert executor.active
    stop_client.future.set_result(SimpleNamespace(success=True))
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    update = confirm_physical_stop(executor, clock, 0.1)
    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert not executor.active


def test_capture_probe_checkpoint_freezes_probe_tip_in_odom():
    frame = executor_module.sensor_probe_frame("sensor_a")
    feedback = transform("odom", frame, x=.37)
    executor, _ = executor_with_client(FakeTransformer({("odom", frame): feedback}))
    checkpoint = executor.capture_probe_checkpoint("sensor_a")
    feedback.transform.translation.x = .99
    assert checkpoint.sensor_id == "sensor_a"
    assert checkpoint.target.header.frame_id == "odom"
    assert checkpoint.target.pose.position.x == .37
    executor._active = True
    with pytest.raises(RuntimeError, match="movement is active"):
        executor.capture_probe_checkpoint("sensor_a")


def test_restore_checkpoint_uses_guard_and_verifies_orientation(monkeypatch):
    from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch, error=0.)
    checkpoint = executor._tag_accuracy["target"]
    executor._active = False
    calls = []
    def guarded(builder, **kwargs):
        calls.append((builder(), kwargs))
        executor._active = True
        return executor_module.ArmMovementUpdate(ArmMovementOutcome.RUNNING, "guarded")
    monkeypatch.setattr(executor, "guarded_probe", guarded)
    assert executor.restore_probe_checkpoint(checkpoint).outcome is ArmMovementOutcome.RUNNING
    assert isinstance(calls[0][0], ResolvedProbeTarget)
    assert calls[0][0].target == checkpoint.target
    executor._begin_tag_position_verification()
    feedback.header.stamp.sec += 1
    feedback.transform.rotation.z = 1.
    feedback.transform.rotation.w = 0.
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(corrections) == 1  # Correct orientation even at the right position.
    assert executor._finish_guarded_update(
        executor_module.ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "corrected")
    ).outcome is ArmMovementOutcome.RUNNING
    feedback.header.stamp.sec += 1
    assert executor.poll().outcome is ArmMovementOutcome.CHECKPOINT_TOLERANCE_FAILED
    assert len(corrections) == 1  # No unbounded correction loop.


@pytest.mark.parametrize("error, expected", [(.0102, ArmMovementOutcome.SUCCESS),
                                            (.021, ArmMovementOutcome.RUNNING)])
def test_checkpoint_return_uses_twenty_mm_position_tolerance(monkeypatch, error, expected):
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch, error=error)
    checkpoint = executor._tag_accuracy["target"]
    executor._active = False
    monkeypatch.setattr(executor, "guarded_probe", lambda *args, **kwargs:
                        executor_module.ArmMovementUpdate(ArmMovementOutcome.RUNNING, "guarded"))
    executor.restore_probe_checkpoint(checkpoint)
    executor._active = True
    executor._begin_tag_position_verification()
    feedback.header.stamp.sec += 1
    assert executor.poll().outcome is expected
    assert len(corrections) == (0 if expected is ArmMovementOutcome.SUCCESS else 1)
