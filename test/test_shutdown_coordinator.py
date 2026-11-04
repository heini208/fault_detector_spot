"""No ROS nodes: prove deadlines, callback lifetime and retryable teardown."""

from threading import Event
import time

import pytest

from fault_detector_spot.shared.ros.shutdown_coordinator import ShutdownCoordinator


def coordinator(**overrides):
    arguments = dict(
        close_admission={}, request_stop={}, confirmations={}, cleanup={},
        progress=Event().wait, report_unconfirmed=lambda unresolved: None,
        timeout_sec=0.08, cleanup_timeout_sec=0.08,
    )
    arguments.update(overrides)
    return ShutdownCoordinator(**arguments)


def test_stuck_owner_cannot_block_other_cancellation_or_deadline():
    release = Event()
    other_cancelled = Event()
    reports, events = [], []
    shutdown = coordinator(
        request_stop={"arm": release.wait, "base": other_cancelled.set},
        confirmations={"arm": lambda: (True, "unused"),
                       "base": lambda: (False, "fresh base pose missing")},
        report_unconfirmed=lambda unresolved: (reports.append(unresolved), events.append("report")),
        cleanup={"clients": lambda: events.append("destroy")},
    )
    start = time.monotonic()
    try:
        report = shutdown.close()
        assert time.monotonic() - start < 0.5
        assert other_cancelled.is_set()
        assert not report.confirmed
        assert set(report.unresolved) == {"arm", "base", "request/arm"}
        assert reports == [report.unresolved]
        assert events == ["report", "destroy"]
        assert shutdown.close() == report
        assert events == ["report", "destroy"]
    finally:
        release.set()


def test_confirmation_exception_remains_uncertain_and_other_owner_is_polled():
    other = Event()

    def failed_poll():
        raise RuntimeError("result unavailable")

    shutdown = coordinator(confirmations={
        "arm": failed_poll, "base": lambda: (other.set() or True, "fresh stop"),
    })
    report = shutdown.close()
    assert not report.confirmed
    assert other.is_set()
    assert set(report.unresolved) == {"arm"}
    assert "result unavailable" in report.unresolved["arm"]


def test_earlier_stationary_evidence_is_rechecked_while_another_owner_finishes():
    polls = {"arm": 0, "base": 0}

    def arm():
        polls["arm"] += 1
        return polls["arm"] >= 3, "delayed action result"

    def base():
        polls["base"] += 1
        return polls["base"] == 1, "base moved after earlier stop proof"

    shutdown = coordinator(confirmations={"arm": arm, "base": base}, timeout_sec=0.2)
    result = shutdown.close()
    assert not result.confirmed and "base" in result.unresolved
    assert polls["arm"] >= 3 and polls["base"] >= 3


def test_cleanup_failure_retries_only_failed_resource_and_continues():
    attempts, events = [], []

    def close_client():
        attempts.append(True)
        if len(attempts) == 1:
            raise RuntimeError("destroy failed")

    shutdown = coordinator(cleanup={"client": close_client, "executor": lambda: events.append("executor")})
    first = shutdown.close()
    assert first.confirmed and first.cleanup_errors == {"client": "destroy failed"}
    assert events == ["executor"]
    assert not shutdown.close().cleanup_errors
    assert shutdown.close().confirmed
    assert len(attempts) == 2 and events == ["executor"]


def test_hung_cleanup_is_bounded_and_not_started_twice():
    release = Event()
    calls, independent = [], []

    def hung():
        calls.append(True)
        release.wait()

    shutdown = coordinator(cleanup={"busy": hung, "independent": lambda: independent.append(True)})
    try:
        assert "busy" in shutdown.close().cleanup_errors
        assert "busy" in shutdown.close().cleanup_errors
        assert calls == [True] and independent == [True]
    finally:
        release.set()
    assert not shutdown.close().cleanup_errors


def test_reporting_error_is_retryable_and_cannot_turn_timeout_into_stopped():
    reports = []

    def report(unresolved):
        reports.append(unresolved)
        if len(reports) == 1:
            raise OSError("read only persistence")

    shutdown = coordinator(confirmations={"nav2": lambda: (False, "missing result")},
                           report_unconfirmed=report)
    first = shutdown.close()
    assert not first.confirmed
    assert "report stop-unconfirmed" in first.cleanup_errors
    second = shutdown.close()
    assert not second.confirmed and not second.cleanup_errors
    assert reports == [first.unresolved, first.unresolved]


def test_detached_and_historical_work_is_reported_once_without_waiting_for_stop():
    events, reports = [], []
    shutdown = coordinator(
        close_admission={"commands": lambda: events.append("close")},
        request_stop={"arm": lambda: events.append("detach")},
        passive_confirmations={"arm": lambda: (events.append("snapshot") or False, "Goal still running")},
        retained_unresolved={"prior shutdown/base": "Previous session ownership is unknown"},
        report_unconfirmed=reports.append,
        cleanup={"clients": lambda: events.append("destroy")},
        timeout_sec=5.0,
    )
    started = time.monotonic()
    report = shutdown.close()

    assert time.monotonic() - started < 1.0
    assert not report.confirmed
    assert report.unresolved == {
        "arm": "Goal still running", "prior shutdown/base": "Previous session ownership is unknown",
    }
    assert events == ["close", "detach", "snapshot", "destroy"]
    assert reports == [report.unresolved]
    assert shutdown.close() == report


def test_detached_uncertainty_does_not_skip_local_callback_drain():
    polls = []

    def callbacks():
        polls.append(True)
        return len(polls) >= 3, "Local callback still finishing"

    shutdown = coordinator(
        passive_confirmations={"commands": lambda: (False, "Execution detached")},
        confirmations={"callbacks": callbacks}, timeout_sec=5.0,
    )
    started = time.monotonic()
    report = shutdown.close()
    assert time.monotonic() - started < 1.0
    assert len(polls) == 3
    assert report.unresolved == {"commands": "Execution detached"}


@pytest.mark.parametrize("value", [0, -1, float("inf"), float("nan")])
def test_deadline_must_be_positive_finite(value):
    with pytest.raises(ValueError):
        coordinator(timeout_sec=value)
