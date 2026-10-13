"""Pose settling survives polling between camera updates without accepting stale data."""

from collections import deque
from copy import deepcopy
from threading import RLock
from types import MethodType, SimpleNamespace

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.sensing import probe_surface_source
from fault_detector_spot.inspection.sensing.probe_surface_source import ProbeSurfaceSource
from fault_detector_spot.inspection.model.sensor_models import sensor_probe_frame
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import MeasureSensorTipDistanceOutcome
from test_measure_sensor_tip_distance_execution import harness


def camera_backed_pose_source(case, monkeypatch):
    source = case.source
    source._lock = RLock()
    source._hand_depth_history = deque(maxlen=32)
    info = CameraInfo()
    info.header.frame_id = "hand_depth"
    source._hand_depth_camera_info = info
    source.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(case.clock.now * 1e9))))
    for name in ("sensor_tip_pose_sample", "latest_hand_depth", "_validate_pose_acquisition_age"):
        setattr(source, name, MethodType(getattr(ProbeSurfaceSource, name), source))
    monkeypatch.setattr(probe_surface_source.time, "monotonic", case.clock)

    def lookup(target, frame, lookup_time):
        assert target == "odom"
        planner = case.executor.probe_motion_planner
        return deepcopy({"hand": planner.hand, sensor_probe_frame("probe"): planner.tip,
                         "body": planner.body}[frame])

    source._lookup_pose = lookup

    def publish(received_at):
        image = Image()
        image.header.frame_id = "hand_depth"
        stamp = round((received_at - .35) * 1e9)
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        source._hand_depth_history.append((received_at, image))

    return publish


def test_delayed_camera_frames_do_not_erase_stability_when_previous_frame_ages(monkeypatch):
    case = harness(visible=True)
    publish = camera_backed_pose_source(case, monkeypatch)
    outcome = case.execution.start(case.command)
    saw_interframe_wait = False
    for index in range(1, 80):
        case.clock.now = 100.0 + index * .1
        if index % 3 == 0:
            publish(case.clock.now)
        outcome = case.execution.poll()
        if "distinct, advancing" in case.execution.feedback_message:
            saw_interframe_wait = True
        if outcome is not MeasureSensorTipDistanceOutcome.RUNNING:
            break
    assert saw_interframe_wait
    assert outcome is MeasureSensorTipDistanceOutcome.SUCCESS
    assert case.execution.result.distance_m == pytest.approx(.25)
    assert case.executor.calls == []


def test_camera_stall_after_one_valid_pose_cannot_complete_or_start_motion(monkeypatch):
    case = harness(visible=False)
    publish = camera_backed_pose_source(case, monkeypatch)
    outcome = case.execution.start(case.command)
    case.clock.now = 101.0
    publish(case.clock.now)
    outcome = case.execution.poll()
    for index in range(1, 80):
        case.clock.now = 101.0 + index * .1
        outcome = case.execution.poll()
        if outcome is not MeasureSensorTipDistanceOutcome.RUNNING:
            break
    assert outcome is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert case.execution.result is None
    assert "distinct, advancing" in case.execution.feedback_message
