"""Landmarks require fresh, distinct, stable observations and a trusted map pose."""

from copy import deepcopy
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import TagElement, TagElementArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.time_source import TimeSource
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException

from fault_detector_spot.navigation.landmark_observations import LandmarkPoseWindow
from fault_detector_spot.navigation.setup import navigation_setup_state_source as module
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose


def localization(stamp, variance=0.01):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        message.pose.covariance[index] = variance
    return message


def tag(stamp, *, frame="body", x=1.0, z=0.8, roll=0.2):
    element = TagElement()
    element.id = 7
    element.pose.header.frame_id = frame
    element.pose.header.stamp = Time(seconds=stamp).to_msg()
    pose = PoseData.identity()
    pose.position.x = x
    pose.position.z = z
    pose.orientation = quaternion_from_euler("x", roll)
    element.pose.pose = pose_data_to_pose(pose)
    return element


@pytest.fixture
def rig(monkeypatch):
    clock = ROSClock()
    time_source = TimeSource()
    time_source.attach_clock(clock)
    time_source.ros_time_is_active = True
    clock.set_ros_time_override(Time(seconds=10.0))
    steady = [100.0]
    params = {}
    node = Mock()
    node.get_clock.return_value = clock
    node.has_parameter.side_effect = lambda name: name in params
    node.declare_parameter.side_effect = lambda name, value: params.setdefault(name, value)
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=params[name])
    buffer = Mock()

    def transform(source, target, **_kwargs):
        result = deepcopy(source)
        result.header.frame_id = target
        result.pose.position.x += 10.0
        return result

    buffer.transform.side_effect = transform
    monkeypatch.setattr(module.tf2_ros, "Buffer", lambda: buffer)
    monkeypatch.setattr(module.tf2_ros, "TransformListener", Mock())
    monkeypatch.setattr(module, "parameter_override", lambda _key, value: value)
    source = module.NavigationSetupStateSource(node, monotonic_clock=lambda: steady[0])
    source._receive_active_map(String(data="map_a"))

    def advance(seconds, *, ros=True):
        steady[0] += seconds
        if ros:
            clock.set_ros_time_override(Time(nanoseconds=clock.now().nanoseconds + int(seconds * 1e9)))

    def observe(**kwargs):
        now = clock.now().nanoseconds / 1e9
        source._receive_localization_pose(localization(now))
        source._receive_visible_tags(TagElementArray(elements=[tag(now, **kwargs)]))

    def qualify():
        for _ in range(3):
            observe()
            advance(0.25)

    yield SimpleNamespace(
        source=source, clock=clock, steady=steady, advance=advance,
        observe=observe, qualify=qualify, buffer=buffer, transform=transform,
    )
    source.close()


def test_save_returns_full_latest_map_pose_after_distinct_stable_observations(rig):
    rig.qualify()
    pose = rig.source.visible_tag_pose(7)
    assert pose.position.x == 11.0
    assert pose.position.z == 0.8
    assert pose.orientation.x == pytest.approx(math.sin(0.1))
    assert pose.orientation.w == pytest.approx(math.cos(0.1))
    assert all(call.kwargs["timeout"].nanoseconds == 0 for call in rig.buffer.transform.call_args_list)
    pose.position.x = 50.0
    assert rig.source.visible_tag_pose(7).position.x == 11.0


def test_repeated_camera_stamp_does_not_supply_multiple_samples(rig):
    for _ in range(10):
        rig.observe()
    with pytest.raises(ValueError, match="1/3"):
        rig.source.visible_tag_pose(7)
    assert rig.buffer.transform.call_count == 1


def test_save_cannot_use_previous_qualification_during_new_tag_transform(rig):
    rig.qualify()
    pending_saves = []

    def transform_while_saving(source, target, **kwargs):
        with pytest.raises(ValueError, match="latest tag observation"):
            rig.source.visible_tag_pose(7)
        pending_saves.append(True)
        return rig.transform(source, target, **kwargs)

    rig.buffer.transform.side_effect = transform_while_saving
    rig.observe(x=2.0)

    assert pending_saves == [True]
    with pytest.raises(ValueError, match="unstable"):
        rig.source.visible_tag_pose(7)


def test_delayed_tags_from_before_localization_recovery_cannot_be_saved(rig):
    rig.qualify()
    bad_stamp = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(bad_stamp, 9999.0))
    rig.advance(0.5)
    recovered_stamp = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(recovered_stamp))
    rig.buffer.transform.reset_mock()

    for offset in (0.1, 0.2, 0.3):
        rig.source._receive_visible_tags(TagElementArray(elements=[tag(bad_stamp + offset)]))
    with pytest.raises(ValueError, match="after trustworthy localization resumed"):
        rig.source.visible_tag_pose(7)
    rig.buffer.transform.assert_not_called()

    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0


@pytest.mark.parametrize("gap_clock", ["both", "ros", "steady"])
def test_localization_gap_restarts_capture_boundary_without_intervening_save(rig, gap_clock):
    rig.qualify()
    if gap_clock == "ros":
        rig.clock.set_ros_time_override(Time(nanoseconds=rig.clock.now().nanoseconds + 1_600_000_000))
    else:
        rig.advance(1.6, ros=gap_clock == "both")
    recovered_stamp = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(recovered_stamp))
    rig.buffer.transform.reset_mock()

    for offset in (0.15, 0.10, 0.05):
        rig.source._receive_visible_tags(TagElementArray(elements=[tag(recovered_stamp - offset)]))
    with pytest.raises(ValueError, match="after trustworthy localization resumed"):
        rig.source.visible_tag_pose(7)
    rig.buffer.transform.assert_not_called()

    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0


def test_healthy_localization_updates_allow_normal_camera_delay(rig):
    rig.qualify()
    first_boundary = rig.source._landmark_healthy_since_ns
    for _ in range(3):
        now = rig.clock.now().nanoseconds / 1e9
        rig.source._receive_localization_pose(localization(now))
        rig.source._receive_visible_tags(TagElementArray(elements=[tag(now - 0.1)]))
        rig.advance(0.25)

    assert rig.source._landmark_healthy_since_ns == first_boundary
    assert rig.source.visible_tag_pose(7).position.x == 11.0


def test_quality_timeout_expires_with_paused_ros_clock(rig):
    rig.qualify()
    rig.source._localization_quality_config = module.LocalizationQualityConfig(
        maximum_age_sec=0.3,
    )
    rig.advance(0.1, ros=False)

    with pytest.raises(ValueError, match="localization estimate is stale"):
        rig.source.visible_tag_pose(7)
    assert rig.source._landmark_healthy_since_ns is None
    assert not rig.source._qualified_landmarks


def test_localization_health_change_during_transform_discards_old_candidate(rig):
    rig.qualify()

    def transform_across_recovery(source, target, **kwargs):
        now = rig.clock.now().nanoseconds / 1e9
        rig.advance(0.05)
        rig.source._receive_localization_pose(localization(now + 0.05, 9999.0))
        rig.advance(0.05)
        rig.source._receive_localization_pose(localization(now + 0.1))
        return rig.transform(source, target, **kwargs)

    rig.buffer.transform.side_effect = transform_across_recovery
    rig.observe()

    assert not rig.source._qualified_landmarks
    assert not rig.source._landmark_windows


def test_real_tf_uses_each_camera_timestamp_while_robot_moves(rig):
    buffer = Buffer()
    rig.source._tf_buffer = buffer
    for offset in range(3):
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.header.stamp = rig.clock.now().to_msg()
        transform.child_frame_id = "body"
        transform.transform.translation.x = 10.0 + offset
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, "test")
        rig.observe(x=1.0 - offset)
        rig.advance(0.25)
    assert rig.source.visible_tag_pose(7).position.x == 11.0


@pytest.mark.parametrize("variance", [0.0, 0.5, 9999.0, float("nan")])
def test_low_or_unknown_localization_certainty_cannot_save(rig, variance):
    rig.qualify()
    now = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(now, variance))
    with pytest.raises(ValueError, match="covariance|uncertainty"):
        rig.source.visible_tag_pose(7)
    assert not rig.source._qualified_landmarks


@pytest.mark.parametrize("fault", ["source_stale", "producer_stale", "localization_stale"])
def test_stale_tag_or_localization_cannot_save(rig, fault):
    rig.qualify()
    rig.advance(1.6, ros=fault != "producer_stale")
    if fault != "localization_stale":
        now = rig.clock.now().nanoseconds / 1e9
        rig.source._receive_localization_pose(localization(now))
    with pytest.raises(ValueError, match="stale|fresh distinct|localization resumed"):
        rig.source.visible_tag_pose(7)


@pytest.mark.parametrize("fault", ["future", "zero", "missing_frame", "nonfinite", "before_map"])
def test_invalid_tag_acquisition_is_not_transformed(rig, fault):
    rig.source._receive_localization_pose(localization(10.0))
    element = tag(10.0)
    if fault == "future":
        element.pose.header.stamp = Time(seconds=10.1).to_msg()
    elif fault == "zero":
        element.pose.header.stamp = Time(seconds=0).to_msg()
    elif fault == "before_map":
        element.pose.header.stamp = Time(seconds=9.9).to_msg()
    elif fault == "missing_frame":
        element.pose.header.frame_id = ""
    else:
        element.pose.pose.position.x = float("nan")
    rig.source._receive_visible_tags(TagElementArray(elements=[element]))
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)
    rig.buffer.transform.assert_not_called()


@pytest.mark.parametrize("change", ["map", "clock"])
def test_map_or_clock_change_clears_visible_tags_and_qualified_samples(rig, change):
    rig.qualify()
    if change == "map":
        rig.source._receive_active_map(String(data="map_b"))
    else:
        rig.clock.set_ros_time_override(Time(seconds=5.0))
    now = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(now))
    with pytest.raises(ValueError, match="not currently visible"):
        rig.source.visible_tag_pose(7)
    assert not rig.source._landmark_windows
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0


@pytest.mark.parametrize("change", ["map", "clock"])
def test_map_or_clock_change_during_transform_cannot_cache_old_pose(rig, change):
    def changing_transform(source, target, **kwargs):
        if change == "map":
            rig.source._receive_active_map(String(data="map_b"))
        else:
            rig.clock.set_ros_time_override(Time(seconds=5.0))
        return rig.transform(source, target, **kwargs)

    rig.buffer.transform.side_effect = changing_transform
    rig.observe()
    assert not rig.source._landmark_windows
    assert not rig.source._qualified_landmarks


@pytest.mark.parametrize("variation", [{"z": 1.0}, {"roll": 0.4}, {"x": 1.1}])
def test_full_3d_instability_restarts_save_window(rig, variation):
    rig.qualify()
    rig.observe(**variation)
    with pytest.raises(ValueError, match="unstable"):
        rig.source.visible_tag_pose(7)
    assert rig.source._landmark_windows[7].sample_count == 1


def test_tag_loss_and_transform_failure_discard_previously_qualified_pose(rig):
    rig.qualify()
    rig.source._receive_visible_tags(TagElementArray())
    with pytest.raises(ValueError, match="not currently visible"):
        rig.source.visible_tag_pose(7)
    rig.qualify()
    rig.buffer.transform.side_effect = TransformException("no transform at capture")
    rig.observe()
    with pytest.raises(ValueError, match="fresh distinct|no transform"):
        rig.source.visible_tag_pose(7)


def test_window_restarts_on_frame_change_or_backward_stamp_and_expires_samples():
    window = LandmarkPoseWindow(3, 4.0, 0.05, math.radians(5))
    pose = PoseData.identity()
    for stamp in (1, 2, 3):
        result = window.add(stamp * 1_000_000_000, pose, "body")
    assert result is not None
    assert window.add(4_000_000_000, pose, "camera") is None
    assert window.sample_count == 1
    window.add(5_000_000_000, pose, "camera")
    assert window.add(3_000_000_000, pose, "camera") is None
    assert window.sample_count == 1
    window.add(8_000_000_000, pose, "camera")
    assert window.sample_count == 1
    pose.position.x = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        window.add(9_000_000_000, pose, "camera")
    assert window.sample_count == 0
