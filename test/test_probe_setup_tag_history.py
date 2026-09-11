"""Tests for capture-window access to authoritative base-tag history."""

from threading import RLock

import pytest
from fault_detector_msgs.msg import TagElement, TagElementArray

from fault_detector_spot.inspection.setup import (
    probe_setup_motion_state_source as source_module,
)


def make_source():
    source = source_module.ProbeSetupMotionStateSource.__new__(
        source_module.ProbeSetupMotionStateSource
    )
    source._lock = RLock()
    source._base_tag_histories = {}
    return source


def make_tags(tag_id, sec, nanosec=0):
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = sec
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    message = TagElementArray()
    message.elements = [tag]
    return message


def test_reference_tag_history_filters_by_local_receipt_window(monkeypatch):
    source = make_source()
    receipt_times = iter((100.0, 100.5, 101.0))
    monkeypatch.setattr(
        source_module.time,
        "monotonic",
        lambda: next(receipt_times),
    )

    source._receive_base_tags(make_tags(23, 10, 100))
    source._receive_base_tags(make_tags(23, 10, 200))
    source._receive_base_tags(make_tags(23, 10, 300))

    history = source.reference_tag_history(
        23,
        receipt_not_before=100.25,
        receipt_not_after=100.75,
    )

    assert len(history) == 1
    assert history[0].pose.header.stamp.nanosec == 200


def test_reference_tag_history_returns_defensive_copies(monkeypatch):
    source = make_source()
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 100.0)
    source._receive_base_tags(make_tags(23, 10, 100))

    first = source.reference_tag_history(23, 99.0, 101.0)
    first[0].pose.pose.position.x = 42.0
    second = source.reference_tag_history(23, 99.0, 101.0)

    assert second[0].pose.pose.position.x == pytest.approx(0.0)


def test_reference_tag_history_rejects_reversed_window():
    source = make_source()

    with pytest.raises(ValueError, match="must not precede"):
        source.reference_tag_history(23, 2.0, 1.0)
