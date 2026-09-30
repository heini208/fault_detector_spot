"""Force reactions and guarded lifecycle progress without a BT tick."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStateSource
from fault_detector_spot.manipulation.guarded_probe_monitor import (
    GuardedProbeMonitor,
)
from test_arm_state_source_force import FakeNode, manipulator_state
from test_guarded_arm_movement_executor import (
    GoalDriver,
    ManualClock,
    execution,
    plan,
    pose,
)
from test_guarded_contact_telemetry import Telemetry


class MonitorNode(FakeNode):

    def create_timer(self, period, callback, *, callback_group, clock):
        self.timer_creations = getattr(self, 'timer_creations', 0) + 1
        self.timer = SimpleNamespace(
            period=period, callback=callback, group=callback_group,
            clock=clock, destroyed=False,
        )
        return self.timer

    def destroy_timer(self, timer):
        timer.destroyed = True


@pytest.fixture
def monitored():
    clock = ManualClock()
    node = MonitorNode()
    source = ArmStateSource(node, monotonic_clock=clock)
    node.callback(manipulator_state(force=(1.0, 2.0, 3.0)))
    driver = GoalDriver()
    current = pose(0.0)
    guard = execution(source, driver, clock, lambda: deepcopy(current))
    telemetry = Telemetry()
    guard.contact_telemetry = telemetry
    monitor = GuardedProbeMonitor(node, source, guard)
    monitor.start(plan)

    def emit(received_at, force=(-5.0, 2.0, 3.0), present=True):
        clock.now = received_at
        node.callback(manipulator_state(force=force, force_present=present))

    yield SimpleNamespace(
        clock=clock, node=node, source=source, driver=driver,
        guard=guard, monitor=monitor, emit=emit, current=current,
        telemetry=telemetry,
    )
    monitor.close()


def test_fresh_samples_confirm_contact_before_any_bt_poll(monitored):
    m = monitored
    m.emit(0.01)
    assert m.driver.stop_count == 0
    assert m.guard._force_contact_count == 1

    m.emit(0.02)

    assert m.driver.cancel_count == 1
    assert m.driver.stop_count == 1
    assert [o['authoritative_decision'] for o in m.telemetry.observations] == [
        'force_candidate', 'contact',
    ]
    assert m.guard.poll().outcome is ArmMovementOutcome.RUNNING
    assert m.driver.stop_count == 1


def test_poll_does_not_make_a_contact_decision(monitored, monkeypatch):
    m = monitored

    def unexpected(*_args):
        pytest.fail('BT poll must not evaluate force')

    monkeypatch.setattr(m.guard, '_check_force_guard', unexpected)
    assert m.guard.poll().outcome is ArmMovementOutcome.RUNNING


def test_idle_monitor_adds_no_timer_or_force_listener():
    clock = ManualClock()
    node = MonitorNode()
    source = ArmStateSource(node, monotonic_clock=clock)
    driver = GoalDriver()
    guard = execution(source, driver, clock, pose(0.0))
    monitor = GuardedProbeMonitor(node, source, guard)
    try:
        for _ in range(100):
            node.callback(manipulator_state())
        assert not hasattr(node, 'timer')
        assert not source._force_listeners
        assert not driver.started_goals
    finally:
        monitor.close()


def test_monitor_poll_only_reads_status(monitored):
    m = monitored
    m.driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'done')
    )
    for _ in range(100):
        assert m.monitor.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(m.driver.updates) == 1
    m.node.timer.callback()
    assert m.monitor.poll().outcome is ArmMovementOutcome.SUCCESS
    assert not m.driver.updates
    assert m.node.timer.destroyed
    assert not m.source._force_listeners


def test_monitor_removes_work_at_completion_and_rearms_on_next_start(monitored):
    m = monitored
    old_callback = m.monitor._listener
    old_timer = m.node.timer
    m.driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'done')
    )
    old_timer.callback()
    assert old_timer.destroyed
    assert not m.source._force_listeners
    m.monitor.start(plan)
    assert m.node.timer_creations == 2
    assert len(m.source._force_listeners) == 1
    old_callback(m.source.hand_force_sample())
    old_timer.callback()
    assert m.guard._force_contact_count == 0
    assert not m.node.timer.destroyed
    m.emit(0.01)
    assert m.guard._force_contact_count == 1


def test_duplicate_samples_do_not_advance_or_repeat_contact(monitored):
    m = monitored
    m.emit(0.01)
    m.emit(0.01)
    assert m.guard._force_contact_count == 1
    assert m.driver.stop_count == 0
    assert len(m.telemetry.observations) == 1

    m.emit(0.02)
    m.emit(0.02)
    m.emit(0.03)
    assert m.driver.stop_count == 1
    assert len(m.telemetry.observations) == 2


def test_sideways_force_remains_non_contact_without_polling(monitored):
    for received_at in (0.01, 0.02, 0.03):
        monitored.emit(received_at, force=(1.0, 22.0, 3.0))
    assert monitored.driver.stop_count == 0
    assert monitored.guard._force_contact_count == 0


def test_self_motion_is_suppressed_without_polling(monitored):
    m = monitored
    for received_at, x, y in ((0.1, 0.01, 0.005), (0.2, 0.02, 0.010),
                              (0.3, 0.03, 0.015)):
        m.current.pose.position.x = x
        m.current.pose.position.y = y
        m.emit(received_at)
    assert m.driver.stop_count == 0
    assert m.guard._self_motion_suppression_count == 2


@pytest.mark.parametrize('missing_force', [False, True])
def test_timer_stops_on_missing_or_stale_force_without_bt_poll(
    monitored, missing_force,
):
    m = monitored
    if missing_force:
        m.emit(0.1, present=False)
    m.clock.now = 0.25
    m.node.timer.callback()
    assert m.driver.cancel_count == 1
    assert m.driver.stop_count == 1

    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    assert not m.guard.active
    assert m.guard.poll().outcome is ArmMovementOutcome.FORCE_STALE


def test_unavailable_force_remains_distinct_from_stale(monitored):
    m = monitored
    # Simulate an unavailable source after the synthetic immediate baseline.
    m.source._last_received_at = None
    m.clock.now = 0.25
    m.node.timer.callback()
    assert m.driver.stop_count == 1
    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    assert m.guard.poll().outcome is ArmMovementOutcome.FORCE_UNAVAILABLE


def test_timer_completes_stop_and_retreat_and_latches_result(monitored):
    m = monitored
    m.current.pose.position.x = 0.008
    m.emit(0.01)
    m.emit(0.02)
    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    assert m.driver.started_goals[-1][0] == 'retreat'
    m.driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'done')
    )
    m.node.timer.callback()
    assert not m.guard.active
    finished = m.guard.poll()
    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert m.guard.poll() is finished


def test_cancellation_and_close_ignore_late_callbacks(monitored):
    m = monitored
    m.emit(0.01)
    m.guard.cancel()
    m.emit(0.02)
    m.node.timer.callback()
    assert m.driver.stop_count == 0

    m.monitor.start(plan)
    callback = m.monitor._listener
    m.monitor.close()
    callback(m.source.hand_force_sample())
    m.emit(0.03)
    m.node.timer.callback()
    assert m.driver.stop_count == 0
    assert m.guard._force_contact_count == 0
    assert m.node.timer.destroyed
    assert not m.source._force_listeners


def test_monitor_has_its_own_callback_group_and_steady_clock(monitored):
    from rclpy.clock import ClockType

    m = monitored
    assert m.node.timer.group is not m.source._callback_group
    assert m.node.timer.clock.clock_type is ClockType.STEADY_TIME


def test_delayed_stale_sample_does_not_advance_contact(monitored):
    m = monitored
    m.emit(0.01)
    sample = m.source.hand_force_sample()
    m.clock.now = 2.0
    m.monitor._listener(sample)
    assert m.guard._force_contact_count == 1
    assert m.driver.stop_count == 1
    assert not m.guard._stop_then_retreat
    assert m.guard._stop_terminal_outcome is ArmMovementOutcome.FORCE_STALE


def test_sample_waiting_for_lock_cannot_affect_a_new_operation(monitored):
    from threading import Event, RLock, Thread, current_thread

    m = monitored
    waiting = Event()

    class ObservedLock:

        def __init__(self):
            self._lock = RLock()

        def __enter__(self):
            if current_thread().name == 'force-sample':
                waiting.set()
            self._lock.acquire()

        def __exit__(self, *_args):
            self._lock.release()

    m.guard.lock = ObservedLock()
    worker = Thread(target=lambda: m.emit(0.01), name='force-sample')
    try:
        with m.guard.lock:
            worker.start()
            assert waiting.wait(timeout=1.0)
            m.guard.cancel()
            m.monitor.start(plan)
    finally:
        worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert m.guard._force_contact_count == 0
    assert m.driver.stop_count == 0
    m.emit(0.02)
    assert m.guard._force_contact_count == 1


def test_stop_submission_failure_is_latched_before_bt_poll(monitored):
    m = monitored
    m.guard._start_stop = lambda: ArmMovementUpdate(
        ArmMovementOutcome.EXECUTION_ERROR, 'service unavailable',
    )
    m.emit(0.01)
    m.emit(0.02)
    assert not m.guard.active
    result = m.guard.poll()
    assert result.outcome is ArmMovementOutcome.STOP_UNCONFIRMED
    assert 'service unavailable' in result.detail
    m.node.timer.callback()
    assert m.guard.poll() is result


def test_executor_wires_monitor_and_observes_its_terminal_result(monkeypatch):
    from geometry_msgs.msg import PoseStamped

    from fault_detector_spot.manipulation.arm_motion_parameters import (
        ArmMotionParameters,
    )
    from test_arm_movement_executor import (
        FakeArmStopServiceClient,
        FakeTransformer,
        capture_builder,
        executor_module,
        executor_with_client,
        transform,
    )
    from test_guarded_arm_movement_executor import (
        ImmediateBaseline,
        FixedForcePolicy,
    )

    clock = ManualClock()
    node = MonitorNode()
    source = ArmStateSource(node, monotonic_clock=clock)
    node.callback(manipulator_state(force=(1.0, 2.0, 3.0)))
    frame = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    tf = FakeTransformer({(frame, 'hand'): transform(frame, 'hand')})
    stop_client = FakeArmStopServiceClient()
    executor, _ = executor_with_client(
        tf, arm_state_source=source, arm_stop_service_client=stop_client,
        force_baseline_sampler=ImmediateBaseline(),
        force_contact_policy=FixedForcePolicy(), config=ArmMotionParameters(),
        monotonic_clock=clock,
    )
    assert not hasattr(node, 'timer')
    assert not source._force_listeners
    capture_builder(monkeypatch)
    target = PoseStamped()
    target.header.frame_id = frame
    target.pose.position.x = 0.1
    target.pose.orientation.w = 1.0
    try:
        started = executor.guarded_probe(target, 'hand')
        assert started.outcome is ArmMovementOutcome.RUNNING
        assert (
            executor.guarded_probe_execution.lock is executor._execution_lock
        )
        for received_at in (0.01, 0.02):
            clock.now = received_at
            node.callback(manipulator_state(force=(-5.0, 2.0, 3.0)))
        assert len(stop_client.requests) == 1
        stop_client.future.set_result(
            SimpleNamespace(success=True, message='accepted')
        )
        for _ in range(100):
            assert executor.poll().outcome is ArmMovementOutcome.RUNNING
        assert executor.guarded_probe_execution.active
        node.timer.callback()
        assert not executor.guarded_probe_execution.active
        assert executor.poll().outcome is ArmMovementOutcome.CONTACT
        assert not executor.active
    finally:
        executor.shutdown()
    assert node.timer.destroyed
    assert not source._force_listeners
