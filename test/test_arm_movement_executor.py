"""Focused tests for centralized Cartesian arm movement."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    ArmMovementOutcome,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
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

    def stow_state(self):
        if self.stale:
            return None
        return self.state

    def is_stale(self):
        return self.stale


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

    def reachable_tag(self, tag_id):
        self.requests.append(tag_id)
        tag = self.tags.get(tag_id)
        return None if tag is None else deepcopy(tag)


class FakeTagCommand:

    def __init__(self, tag_id, sensor_id, probe_target):
        self.tag_id = tag_id
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
        executor_module.RobotCommandBuilder,
        "arm_pose_command",
        build,
    )
    monkeypatch.setattr(
        executor_module,
        "convert",
        lambda source, target: None,
    )
    return captured


def executor_with_client(transformer, **kwargs):
    client = kwargs.pop("action_client", FakeActionClient())
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


def test_tag_probe_forwards_speed_to_probe_motion(monkeypatch):
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
    assert captured["args"][8] == pytest.approx(4.0)


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


def test_tag_probe_rejects_unreachable_tag_as_typed_failure():
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
    assert "not currently reachable" in update.detail
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
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
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
    executor, _ = executor_with_client(
        transformer,
        action_client=client,
        monotonic_clock=clock,
        result_timeout_sec=3.0,
    )

    executor.pose(target)
    send_future.set_result(handle)
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING

    clock.now = 3.0
    update = executor.poll()

    assert update.outcome is ArmMovementOutcome.RESULT_TIMEOUT
    assert handle.cancel_count == 1
    assert not executor.active


def test_explicit_cancel_requests_goal_cancellation(monkeypatch):
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
    executor.poll()

    executor.cancel()

    assert handle.cancel_count == 1
    assert not executor.active


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
        executor_module.RobotCommandBuilder,
        "arm_stow_command",
        build_stow,
    )
    monkeypatch.setattr(
        executor_module,
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
