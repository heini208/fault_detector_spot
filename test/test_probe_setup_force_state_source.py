"""Tests for fresh Spot end-effector force access during probe setup."""

from collections import deque
from threading import RLock

import pytest
from geometry_msgs.msg import Vector3Stamped

import fault_detector_spot.inspection.setup.probe_setup_motion_state_source as source_module
from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES,
    ProbeSetupMotionStateSource,
)


def force_message(x=1.0, y=2.0, z=3.0):
    message = Vector3Stamped()
    message.header.stamp.sec = 10
    message.header.frame_id = "hand"
    message.vector.x = x
    message.vector.y = y
    message.vector.z = z
    return message


def state_source():
    source = ProbeSetupMotionStateSource.__new__(ProbeSetupMotionStateSource)
    source._lock = RLock()
    source._end_effector_force_history = deque(
        maxlen=END_EFFECTOR_FORCE_HISTORY_MAX_SAMPLES
    )
    return source


def test_force_callback_uses_local_receipt_time_and_returns_typed_sample(
    monkeypatch,
):
    source = state_source()
    times = iter((100.0, 100.1))
    monkeypatch.setattr(source_module.time, "monotonic", lambda: next(times))

    source._receive_end_effector_force(force_message())
    samples = source.end_effector_force_samples(maximum_age_sec=0.25)

    assert len(samples) == 1
    assert samples[0].receipt_time == pytest.approx(100.0)
    assert samples[0].stamp_seconds == pytest.approx(10.0)
    assert samples[0].frame_id == "hand"
    assert samples[0].force_hand.x == pytest.approx(1.0)
    assert samples[0].force_hand.y == pytest.approx(2.0)
    assert samples[0].force_hand.z == pytest.approx(3.0)


def test_force_access_rejects_stale_samples(monkeypatch):
    source = state_source()
    source._end_effector_force_history.append(
        (100.0, force_message())
    )
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 100.5)

    with pytest.raises(ValueError, match="No fresh end-effector force"):
        source.end_effector_force_samples(maximum_age_sec=0.25)


def test_force_access_honors_baseline_receipt_gate(monkeypatch):
    source = state_source()
    source._end_effector_force_history.extend(
        [
            (100.0, force_message(x=1.0)),
            (100.1, force_message(x=2.0)),
            (100.2, force_message(x=3.0)),
        ]
    )
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 100.25)

    samples = source.end_effector_force_samples(
        receipt_not_before=100.1,
        maximum_age_sec=0.5,
    )

    assert [sample.force_hand.x for sample in samples] == [2.0, 3.0]
