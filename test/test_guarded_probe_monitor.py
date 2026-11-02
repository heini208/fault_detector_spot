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


class MonitorTimer:
    def __init__(self, period, callback, group, clock):
        self.period = period
        self.callback = callback
        self.group = group
        self.clock = clock
        self.destroyed = False
        self.cancelled = False
        self.reset_count = 0

    def cancel(self):
        self.cancelled = True

    def reset(self):
        self.cancelled = False
        self.reset_count += 1


class MonitorNode(FakeNode):

    def create_timer(self, period, callback, *, callback_group, clock):
        self.timer_creations = getattr(self, 'timer_creations', 0) + 1
        self.timer = MonitorTimer(
            period,
            callback,
            callback_group,
            clock,
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
    monitor = GuardedProbeMonitor(node, source, guard, monotonic_clock=clock)
    monitor.start(plan)

    def emit(
        received_at,
        force=(-5.0, 2.0, 3.0),
        present=True,
        linear_velocity=None,
        angular_velocity=None,
    ):
        clock.now = received_at
        velocity_present = (
            linear_velocity is not None or angular_velocity is not None
        )
        node.callback(manipulator_state(
            force=force,
            force_present=present,
            velocity_present=velocity_present,
            linear_velocity=linear_velocity or (0.0, 0.0, 0.0),
            angular_velocity=angular_velocity or (0.0, 0.0, 0.0),
        ))

    yield SimpleNamespace(
        clock=clock, node=node, source=source, driver=driver,
        guard=guard, monitor=monitor, emit=emit, current=current,
        telemetry=telemetry,
    )
    monitor.close()


def confirm_physical_stop(monitored, first_sample_at):
    for received_at in (first_sample_at, first_sample_at + 0.4):
        monitored.emit(
            received_at,
            linear_velocity=(0.0, 0.0, 0.0),
        )
        monitored.node.timer.callback()


def test_fresh_samples_confirm_contact_before_any_bt_poll(monitored):
    m = monitored
    assert m.monitor.diagnostics["event"] == "motion_started"
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


@pytest.mark.parametrize("force", [(-5.0, 2.0, 3.0), (-20.0, 2.0, 3.0)])
def test_contact_stop_precedes_failing_diagnostics(monitored, force):
    m = monitored
    m.emit(0.01)
    observed_stops = []

    def fail_observation(**_values):
        observed_stops.append((m.driver.cancel_count, m.driver.stop_count))
        raise OSError("diagnostic storage failed")

    def fail_event(_event):
        raise OSError("diagnostic event failed")

    m.telemetry.observe = fail_observation
    m.telemetry.record_event = fail_event
    m.emit(0.02, force=force)
    assert observed_stops == [(1, 1)]
    assert m.driver.stop_count == 1
    assert m.monitor.diagnostics["stop_confirmation_state"] == "requested"
    assert m.monitor.diagnostics["stop_requested_at"] == 0.02


def test_cancel_and_confirmation_keep_correlated_diagnostics(monitored):
    m = monitored
    events = []
    m.telemetry.record_event = events.append
    m.guard.set_diagnostic_context(
        request_id="request-a", context_id="context-a", sensor_id="probe-a",
        attachment_revision=7,
    )
    m.clock.now = 0.01
    m.monitor.cancel(None)
    assert m.driver.stop_count == 1
    m.driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "accepted"))
    m.node.timer.callback()
    confirm_physical_stop(m, 0.1)
    snapshot = m.monitor.diagnostics
    assert snapshot["stop_confirmation_state"] == "confirmed"
    assert snapshot["request_id"] == "request-a"
    assert snapshot["context_id"] == "context-a"
    assert snapshot["sensor_id"] == "probe-a"
    assert snapshot["attachment_revision"] == 7
    assert snapshot["monitor_period_sec"] == 0.05
    snapshot["request_id"] = "mutated"
    assert m.monitor.diagnostics["request_id"] == "request-a"
    assert any(event["event"] == "stop_confirmed" for event in events)
    m.guard.set_diagnostic_context(request_id="request-b", context_id="context-b")
    assert m.monitor.diagnostics["request_id"] == "request-a"
    assert m.monitor.diagnostics["context_id"] == "context-a"
    m.monitor.start(plan)
    assert m.monitor.diagnostics["request_id"] == "request-b"
    assert "callback_duration_sec" not in m.monitor.diagnostics
    assert "outcome" not in m.monitor.diagnostics


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
    assert m.node.timer.cancelled
    assert not m.node.timer.destroyed
    assert not m.source._force_listeners


def test_terminal_start_does_not_destroy_live_timer(monitored, monkeypatch):
    m = monitored
    timer = m.node.timer
    monkeypatch.setattr(
        m.guard,
        "start",
        lambda *_args, **_kwargs: ArmMovementUpdate(
            ArmMovementOutcome.EXECUTION_ERROR,
            "orientation failed",
        ),
    )

    result = m.monitor.start(plan)

    assert result.outcome is ArmMovementOutcome.EXECUTION_ERROR
    assert timer.cancelled
    assert not timer.destroyed
    assert m.node.timer_creations == 1
    assert not m.source._force_listeners


def test_monitor_removes_work_at_completion_and_rearms_on_next_start(monitored):
    m = monitored
    old_callback = m.monitor._listener
    old_timer = m.node.timer
    m.driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'done')
    )
    old_timer.callback()
    assert old_timer.cancelled
    assert not old_timer.destroyed
    assert not m.source._force_listeners
    m.monitor.start(plan)
    assert m.node.timer_creations == 1
    assert m.node.timer is old_timer
    assert old_timer.reset_count == 1
    assert len(m.source._force_listeners) == 1
    old_callback(m.source.hand_force_sample())
    old_timer.callback()
    assert m.guard._force_contact_count == 0
    assert not old_timer.cancelled
    assert not old_timer.destroyed
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


def test_hard_force_limit_stops_once_without_bt_poll(monitored):
    m = monitored
    m.emit(0.01, force=(-20.0, 2.0, 3.0))
    m.emit(0.01, force=(-20.0, 2.0, 3.0))

    assert m.driver.cancel_count == 1
    assert m.driver.stop_count == 1
    assert m.guard._force_contact_count == 0
    assert m.telemetry.observations[-1]["authoritative_decision"] == (
        "hard_force_limit"
    )


def test_sideways_force_remains_non_contact_without_polling(monitored):
    for received_at in (0.01, 0.02, 0.03):
        monitored.emit(received_at, force=(1.0, 21.0, 3.0))
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
    assert m.guard.active
    confirm_physical_stop(m, 0.3)
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
    confirm_physical_stop(m, 0.3)
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
    assert m.driver.started_goals[-1] == ('arm_stop', 1)
    confirm_physical_stop(m, 0.1)
    assert m.driver.started_goals[-1][0] == 'retreat'
    m.driver.updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'done')
    )
    m.node.timer.callback()
    assert m.guard.active
    assert m.driver.started_goals[-1] == ('arm_stop', 2)
    assert not m.guard.poll().retry_safe
    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    confirm_physical_stop(m, 0.6)
    assert not m.guard.active
    finished = m.guard.poll()
    assert finished.outcome is ArmMovementOutcome.CONTACT
    assert finished.retry_safe
    assert m.guard.poll() is finished


def test_cancellation_and_close_ignore_late_callbacks(monitored):
    m = monitored
    m.emit(0.01)
    m.guard.cancel()
    m.emit(0.02)
    m.node.timer.callback()
    assert m.driver.stop_count == 1

    m.monitor.start(plan)
    callback = m.monitor._listener
    m.monitor.close()
    callback(m.source.hand_force_sample())
    m.emit(0.03)
    m.node.timer.callback()
    assert m.driver.stop_count == 1
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
    assert m.driver.stop_count == 1
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
    assert m.monitor.diagnostics["stop_confirmation_state"] == "unconfirmed"
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
        surface_source=SimpleNamespace(active_attachment=lambda: ("hand", 7)),
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
        assert executor.guarded_probe_execution.diagnostics["sensor_id"] == "hand"
        assert executor.guarded_probe_execution.diagnostics["attachment_revision"] == 7
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
        assert executor.guarded_probe_execution.active
        for received_at in (0.1, 0.5):
            clock.now = received_at
            node.callback(manipulator_state(
                force=(-5.0, 2.0, 3.0),
                velocity_present=True,
            ))
            node.timer.callback()
        assert not executor.guarded_probe_execution.active
        assert executor.poll().outcome is ArmMovementOutcome.CONTACT
        assert not executor.active
    finally:
        # This monitor test deliberately leaves the action response pending.
        executor.shutdown(stop_unconfirmed=True)
    assert node.timer.destroyed
    assert not source._force_listeners


def test_monitor_cancellation_confirms_stop_without_bt_poll(monitored):
    m = monitored
    completed = []
    m.monitor.cancel(completed.append)
    assert m.guard.active
    assert m.driver.stop_count == 1
    assert not completed
    m.driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'stopped'))
    m.node.timer.callback()
    confirm_physical_stop(m, 0.1)
    assert completed[-1].outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
    assert not m.guard.active
    assert not any(isinstance(goal, tuple) and goal[0] == 'retreat' for goal in m.driver.started_goals)


def test_cancellation_keeps_observing_after_stop_timeout_without_bt_poll(monitored):
    m = monitored
    completed = []
    m.monitor.cancel(completed.append, continue_after_timeout=True)
    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    m.emit(3.0, linear_velocity=(0.05, 0.0, 0.0))
    m.node.timer.callback()

    assert m.monitor.poll().outcome is ArmMovementOutcome.RUNNING
    assert 'continuing physical stop confirmation' in m.monitor.poll().detail
    assert m.monitor.diagnostics["stop_confirmation_state"] == "unconfirmed"
    assert m.guard.active
    assert not m.node.timer.cancelled
    assert not completed
    assert m.driver.stop_count == 1

    confirm_physical_stop(m, 3.1)

    assert completed[-1].outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
    assert m.node.timer.cancelled
    assert m.driver.stop_count == 1
    assert not m.guard.active
    assert m.monitor.diagnostics["stop_confirmation_state"] == "confirmed"


def test_cancellation_requires_new_stable_interval_after_velocity_goes_stale(monitored):
    m = monitored
    completed = []
    m.monitor.cancel(completed.append, continue_after_timeout=True)
    m.driver.stop_updates.append(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, 'accepted')
    )
    m.node.timer.callback()
    m.emit(0.1, linear_velocity=(0.0, 0.0, 0.0))
    m.node.timer.callback()

    m.clock.now = 3.0
    assert m.source.hand_velocity_sample() is None
    m.node.timer.callback()
    m.emit(3.1, linear_velocity=(0.0, 0.0, 0.0))
    m.node.timer.callback()
    assert not completed
    assert m.guard.active

    m.emit(3.5, linear_velocity=(0.0, 0.0, 0.0))
    m.node.timer.callback()
    assert completed[-1].outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
    assert m.driver.stop_count == 1
