"""Tests for server-owned registered hand-depth sampling."""

from collections import deque
from threading import RLock

from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup import (
    probe_setup_motion_state_source as source_module,
)


class _Clock:
    def now(self):
        return Time(seconds=10.0, clock_type=ClockType.ROS_TIME)


class _Node:
    def get_clock(self):
        return _Clock()


def _image(stamp_seconds):
    message = Image()
    message.header.stamp = Time(seconds=stamp_seconds).to_msg()
    message.header.frame_id = "hand_depth"
    return message


def test_surface_samples_ignore_pre_settle_receipts(monkeypatch):
    source = source_module.ProbeSetupMotionStateSource.__new__(
        source_module.ProbeSetupMotionStateSource
    )
    source.node = _Node()
    source._lock = RLock()
    source._hand_depth_camera_info = CameraInfo()
    source._hand_depth_camera_info.header.frame_id = "hand_depth"
    source._hand_depth_history = deque(
        (
            (3.7, _image(9.7)),
            (3.8, _image(9.8)),
            (3.9, _image(9.85)),
            (4.0, _image(9.9)),
        )
    )
    lookups = []
    source._lookup_pose = lambda target, frame, lookup_time=None: (
        lookups.append((target, frame, lookup_time.nanoseconds)) or object()
    )
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 4.1)
    monkeypatch.setattr(
        source_module,
        "measure_probe_surface_distance",
        lambda image, info, pose: image.header.stamp.sec
        + image.header.stamp.nanosec * 1e-9,
    )

    samples = source.surface_distance_samples(
        "hall_probe",
        receipt_not_before=3.8,
    )

    assert samples == (9.8, 9.85, 9.9)
    assert len(lookups) == 3
    assert all(frame == "hand_depth" for _, frame, _ in lookups)


def test_surface_samples_require_three_fresh_frames(monkeypatch):
    source = source_module.ProbeSetupMotionStateSource.__new__(
        source_module.ProbeSetupMotionStateSource
    )
    source.node = _Node()
    source._lock = RLock()
    source._hand_depth_camera_info = CameraInfo()
    source._hand_depth_history = deque(
        (
            (1.8, _image(9.8)),
            (2.0, _image(9.9)),
        )
    )
    source._lookup_pose = lambda *args, **kwargs: object()
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 2.1)
    monkeypatch.setattr(
        source_module,
        "measure_probe_surface_distance",
        lambda image, info, pose: 0.05,
    )

    try:
        source.surface_distance_samples("hall_probe")
    except ValueError as exception:
        assert "at least three fresh" in str(exception)
    else:
        raise AssertionError("Expected insufficient fresh depth to fail")


def test_surface_samples_use_receipt_time_not_ros_header_age(monkeypatch):
    source = source_module.ProbeSetupMotionStateSource.__new__(
        source_module.ProbeSetupMotionStateSource
    )
    source.node = _Node()
    source._lock = RLock()
    source._hand_depth_camera_info = CameraInfo()
    source._hand_depth_camera_info.header.frame_id = "hand_depth"
    source._hand_depth_history = deque(
        (
            (9.7, _image(1.0)),
            (9.8, _image(1.1)),
            (9.9, _image(1.2)),
        )
    )
    source._lookup_pose = lambda *args, **kwargs: object()
    monkeypatch.setattr(source_module.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(
        source_module,
        "measure_probe_surface_distance",
        lambda image, info, pose: 0.05,
    )

    samples = source.surface_distance_samples("hall_probe")

    assert samples == (0.05, 0.05, 0.05)
