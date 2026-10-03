"""Focused tests for BaseMovementExecutor lifecycle."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.base_motion_planner import (
    BaseMovementPlan,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor,
    BaseMovementOutcome,
)
from fault_detector_spot.navigation.posture_state_source import (
    PostureState,
)


class ReadyHeight:
    """Height already verified; isolate existing movement lifecycle tests."""
    def sample(self, _now):
        return object()

    def at_nominal_height(self, _sample):
        return True

    def require_reset(self):
        pass

    def begin_confirmation(self, _stamp):
        pass

    def confirm_reset(self, _now):
        return True


class ManualFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def set_result(self, result):
        self._result = result
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:

    def __init__(self, result_future, accepted=True):
        self.accepted = accepted
        self._result_future = result_future
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return ManualFuture()


class FakeActionClient:

    def __init__(self, send_future):
        self.send_future = send_future
        self.send_calls = 0

    def wait_for_server(self, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return True

    def send_goal_async(self, _goal):
        self.send_calls += 1
        return self.send_future


class FakePostureStateSource:

    def __init__(
        self,
        state,
        last_received_at=0.0,
        stale=False,
    ):
        self.state = state
        self.last_received_at = last_received_at
        self.stale = stale

    def posture(self):
        if self.stale:
            return None
        return self.state

    def is_stale(self):
        return self.stale


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def movement_plan(executor):
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.orientation.w = 1.0
    return BaseMovementPlan(
        target=target,
        linear_speed_mps=0.1,
        profile=executor.walking_profiles.for_move(),
    )


def test_stand_uses_executor_lifecycle_until_success():
    send_future = ManualFuture()
    result_future = ManualFuture()
    client = FakeActionClient(send_future)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
    )
    executor._build_stand_goal = lambda: object()

    update = executor.stand()
    assert update.outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 1

    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.RUNNING

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


def test_sit_uses_executor_lifecycle_until_success():
    send_future = ManualFuture()
    result_future = ManualFuture()
    client = FakeActionClient(send_future)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
    )
    executor._build_sit_goal = lambda: object()

    update = executor.sit()
    assert update.outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 1

    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.RUNNING

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


def test_standing_relative_starts_without_stand():
    send_future = ManualFuture()
    client = FakeActionClient(send_future)
    posture = FakePostureStateSource(PostureState.STANDING)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
        posture_state_source=posture,
    )
    command = object()
    built = []
    plan = movement_plan(executor)
    executor.motion_planner.resolve_relative = (
        lambda value: built.append(value) or plan
    )
    executor.motion_planner.build_goal = lambda value, robot_name="": object()

    update = executor.relative(command)

    assert update.outcome is BaseMovementOutcome.RUNNING
    assert built == [command]
    assert executor._movement_plan is plan
    assert client.send_calls == 1


def test_sitting_relative_stands_before_resolving_requested_plan():
    stand_send_future = ManualFuture()
    stand_result_future = ManualFuture()
    client = FakeActionClient(stand_send_future)
    posture = FakePostureStateSource(PostureState.SITTING)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
        posture_state_source=posture,
    )
    command = object()
    built = []
    plan = movement_plan(executor)
    executor._build_stand_goal = lambda: object()
    executor.motion_planner.resolve_relative = (
        lambda value: built.append(value) or plan
    )
    executor.motion_planner.build_goal = lambda value, robot_name="": object()

    started = executor.relative(command)

    assert started.outcome is BaseMovementOutcome.RUNNING
    assert built == []
    assert client.send_calls == 1

    stand_handle = FakeGoalHandle(stand_result_future)
    stand_send_future.set_result(stand_handle)
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING

    stand_result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    waiting = executor.poll()

    assert waiting.outcome is BaseMovementOutcome.RUNNING
    assert "standing" in waiting.detail
    assert built == []

    posture.state = PostureState.STANDING
    client.send_future = ManualFuture()

    resumed = executor.poll()

    assert resumed.outcome is BaseMovementOutcome.RUNNING
    assert built == [command]
    assert executor._movement_plan is plan
    assert client.send_calls == 2


def test_relative_reports_missing_posture_after_bounded_wait():
    clock = ManualClock()
    posture = FakePostureStateSource(
        None,
        last_received_at=None,
        stale=True,
    )
    send_future = ManualFuture()
    client = FakeActionClient(send_future)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
        posture_state_source=posture,
        monotonic_clock=clock,
        ready_state_timeout_sec=2.0,
    )
    built = []
    executor.motion_planner.resolve_relative = (
        lambda value: built.append(value) or movement_plan(executor)
    )

    first = executor.relative(object())

    assert first.outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 0
    assert built == []

    clock.now = 2.0
    failed = executor.poll()

    assert (
        failed.outcome
        is BaseMovementOutcome.POSTURE_STATE_UNAVAILABLE
    )
    assert "2.0 s" in failed.detail
    assert client.send_calls == 0
    assert built == []
    assert not executor.active


def test_stand_confirmation_requires_reported_standing():
    clock = ManualClock()
    stand_send_future = ManualFuture()
    stand_result_future = ManualFuture()
    client = FakeActionClient(stand_send_future)
    posture = FakePostureStateSource(PostureState.SITTING)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=client,
        posture_state_source=posture,
        monotonic_clock=clock,
        ready_standing_timeout_sec=2.0,
    )
    executor._build_stand_goal = lambda: object()
    executor.motion_planner.resolve_relative = (
        lambda command: movement_plan(executor)
    )

    executor.relative(object())
    stand_send_future.set_result(
        FakeGoalHandle(stand_result_future)
    )
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING

    stand_result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING

    clock.now = 2.0
    failed = executor.poll()

    assert failed.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "did not report STANDING" in failed.detail
    assert not executor.active


def test_executor_rejects_second_base_operation_while_active():
    send_future = ManualFuture()
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )
    assert (
        executor.stand().outcome
        is BaseMovementOutcome.BUSY
    )


def test_executor_remains_busy_while_cancellation_is_pending():
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)

    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )
    assert (
        executor.poll().outcome
        is BaseMovementOutcome.RUNNING
    )

    executor.cancel()

    assert executor.active
    assert (
        executor.sit().outcome
        is BaseMovementOutcome.BUSY
    )

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert not executor.active


def test_cancel_retains_executor_until_goal_reaches_terminal_state():
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)

    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )
    assert (
        executor.poll().outcome
        is BaseMovementOutcome.RUNNING
    )

    executor.cancel()

    assert handle.cancel_calls == 1
    assert executor.active

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert not executor.active



def test_cancel_before_goal_acceptance_retains_ownership_until_terminal():
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)

    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )

    executor.cancel()

    assert executor.active
    assert handle.cancel_calls == 0

    send_future.set_result(handle)

    assert handle.cancel_calls == 1
    assert executor.active

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert not executor.active



def test_goal_response_timeout_retains_base_ownership_until_terminal():
    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
        monotonic_clock=clock,
        goal_response_timeout_sec=1.0,
    )
    executor._build_stand_goal = lambda: object()

    assert executor.stand().outcome is BaseMovementOutcome.RUNNING
    clock.now = 1.0

    timeout = executor.poll()

    assert timeout.outcome is BaseMovementOutcome.RUNNING
    assert executor.active
    assert executor.sit().outcome is BaseMovementOutcome.BUSY

    send_future.set_result(handle)

    assert handle.cancel_calls == 1
    assert executor.active

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert executor.active
    terminal = executor.poll()
    assert terminal.outcome is BaseMovementOutcome.GOAL_RESPONSE_TIMEOUT
    assert not executor.active


def test_result_timeout_retains_base_ownership_until_terminal():
    clock = ManualClock()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future = ManualFuture()
    send_future.set_result(handle)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
        monotonic_clock=clock,
        result_timeout_sec=1.0,
    )
    executor._build_stand_goal = lambda: object()

    assert executor.stand().outcome is BaseMovementOutcome.RUNNING
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    clock.now = 1.0

    timeout = executor.poll()

    assert timeout.outcome is BaseMovementOutcome.RUNNING
    assert handle.cancel_calls == 1
    assert executor.active
    assert executor.sit().outcome is BaseMovementOutcome.BUSY

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert executor.active
    terminal = executor.poll()
    assert terminal.outcome is BaseMovementOutcome.RESULT_TIMEOUT
    assert not executor.active



def test_explicit_cancel_during_timeout_cleanup_releases_without_polling():
    clock = ManualClock()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future = ManualFuture()
    send_future.set_result(handle)
    executor = BaseMovementExecutor(
        height_readiness=ReadyHeight(),
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
        monotonic_clock=clock,
        result_timeout_sec=1.0,
    )
    executor._build_stand_goal = lambda: object()

    assert executor.stand().outcome is BaseMovementOutcome.RUNNING
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    clock.now = 1.0
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    assert executor.active

    executor.cancel()
    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=False)
        )
    )

    assert not executor.active
