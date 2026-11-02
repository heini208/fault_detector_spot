"""Offline 50 ms callback budget: storage must never delay guard stop dispatch.

This is one configured monitor period, not a deployed hardware stop guarantee.
"""

from contextlib import contextmanager
from pathlib import Path
from threading import Event
import time
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_contact_telemetry import ArmContactTelemetry
from fault_detector_spot.manipulation.arm_state_source import ArmStateSource
from fault_detector_spot.manipulation.guarded_probe_monitor import GuardedProbeMonitor
from test_arm_state_source_force import manipulator_state
from test_guarded_arm_movement_executor import GoalDriver, execution, plan, pose
from test_guarded_probe_monitor import MonitorNode


CALLBACK_BUDGET_SEC = 0.05


class TimedDriver(GoalDriver):
    stop_at = None

    def start_stop(self):
        self.stop_at = time.monotonic()
        return super().start_stop()


@contextmanager
def guarded_telemetry(root, enabled=True, **telemetry_options):
    node = MonitorNode()
    source = ArmStateSource(node, monotonic_clock=time.monotonic)
    node.callback(manipulator_state(force=(1.0, 2.0, 3.0)))
    driver = TimedDriver()
    guard = execution(source, driver, time.monotonic, pose(0.0))
    telemetry = ArmContactTelemetry(
        source, SimpleNamespace(sample=lambda: None), raw_logging_enabled=enabled,
        raw_log_root=root, **telemetry_options,
    )
    guard.contact_telemetry = telemetry
    monitor = GuardedProbeMonitor(node, source, guard)
    monitor.start(plan)
    try:
        yield SimpleNamespace(node=node, source=source, driver=driver,
                              telemetry=telemetry, monitor=monitor, guard=guard)
    finally:
        monitor.close()
        telemetry.close()


def emit(harness, force=(-20.0, 2.0, 3.0)):
    started = time.monotonic()
    harness.node.callback(manipulator_state(force=force))
    duration = time.monotonic() - started
    assert duration < CALLBACK_BUDGET_SEC, f'Force callback took {duration:.6f}s'
    return started


def assert_stop_dispatched(harness, started):
    assert harness.driver.cancel_count == harness.driver.stop_count == 1
    latency = harness.driver.stop_at - started
    assert 0 <= latency < CALLBACK_BUDGET_SEC, f'Stop request took {latency:.6f}s'


@pytest.mark.parametrize('enabled', [False, True])
def test_guard_callback_budget_with_and_without_raw_logging(tmp_path, enabled):
    with guarded_telemetry(tmp_path, enabled) as harness:
        started = emit(harness)
        assert_stop_dispatched(harness, started)
        timing = harness.monitor.diagnostics
        assert timing['monitor_period_sec'] == harness.node.timer.period == CALLBACK_BUDGET_SEC
        assert 0 <= timing['callback_delay_sec'] < CALLBACK_BUDGET_SEC
        assert 0 <= timing['observation_age_sec'] < CALLBACK_BUDGET_SEC
        assert 0 <= timing['callback_duration_sec'] < CALLBACK_BUDGET_SEC
        assert timing['stop_requested_at'] <= harness.driver.stop_at


@pytest.mark.parametrize('operation', ['open', 'write', 'flush'])
@pytest.mark.parametrize('trigger', ['force', 'cancel'])
def test_stop_dispatch_does_not_wait_for_blocked_storage(tmp_path, monkeypatch, operation, trigger):
    entered, release, timed_out = Event(), Event(), Event()
    open_file = Path.open

    def stall():
        entered.set()
        if not release.wait(2.0):
            timed_out.set()
            raise TimeoutError('test sink was not released')

    def blocked_open(path, *args, **kwargs):
        if path.parent == tmp_path and operation == 'open':
            stall()
        stream = open_file(path, *args, **kwargs)
        if path.parent == tmp_path and operation in ('write', 'flush'):
            original = getattr(stream, operation)

            def blocked(*values):
                stall()
                return original(*values)

            monkeypatch.setattr(stream, operation, blocked)
        return stream

    monkeypatch.setattr(Path, 'open', blocked_open)
    with guarded_telemetry(tmp_path, raw_queue_capacity=2) as harness:
        try:
            emit(harness, (1.0, 2.0, 3.0))
            assert entered.wait(1.0), 'writer never reached the injected storage stall'
            # Saturation must not turn subsequent callbacks into blocking writes.
            for _ in range(4):
                emit(harness, (1.0, 2.0, 3.0))
            if trigger == 'force':
                started = emit(harness)
            else:
                started = time.monotonic()
                harness.monitor.cancel(lambda _update: None)
                assert time.monotonic() - started < CALLBACK_BUDGET_SEC
            assert_stop_dispatched(harness, started)
            assert not release.is_set()
            assert not timed_out.is_set()
        finally:
            release.set()


@pytest.mark.parametrize('operation', ['open', 'write', 'flush', 'observe'])
def test_diagnostic_exceptions_do_not_delay_stop(tmp_path, monkeypatch, operation):
    failed = Event()
    open_file = Path.open

    def fail(*_args, **_kwargs):
        failed.set()
        raise OSError('injected diagnostic failure')

    def failing_open(path, *args, **kwargs):
        if path.parent == tmp_path and operation == 'open':
            return fail()
        stream = open_file(path, *args, **kwargs)
        if path.parent == tmp_path and operation in ('write', 'flush'):
            monkeypatch.setattr(stream, operation, fail)
        return stream

    monkeypatch.setattr(Path, 'open', failing_open)
    with guarded_telemetry(tmp_path) as harness:
        if operation == 'observe':
            monkeypatch.setattr(harness.telemetry.arm_joint_state_source, 'sample', fail)
        assert_stop_dispatched(harness, emit(harness))
        assert failed.wait(1.0), 'diagnostic failure was not exercised'
