"""Waypoint saves require a current robot TF and continuing localization updates."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.time_source import TimeSource
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException

from fault_detector_spot.navigation.setup import navigation_setup_state_source as pose_source
from fault_detector_spot.shared.geometry.models import PoseData


def localization(stamp=9.0, frame="map"):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.pose.pose.position.x = 2.0
    message.pose.pose.orientation.w = 1.0
    return message


def robot_transform(stamp=9.9):
    message = TransformStamped()
    message.header.frame_id = "map"
    message.child_frame_id = "base_link"
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.transform.translation.x = 7.0
    message.transform.rotation.w = 1.0
    return message


@pytest.fixture
def rig(monkeypatch):
    clock = ROSClock()
    time_source = TimeSource()
    time_source.attach_clock(clock)
    time_source.ros_time_is_active = True
    clock.set_ros_time_override(Time(seconds=10.0))
    steady = [100.0]
    node = Mock()
    node.get_clock.return_value = clock
    buffer = Mock()
    buffer.lookup_transform.return_value = robot_transform()
    monkeypatch.setattr(pose_source.tf2_ros, "Buffer", lambda: buffer)
    monkeypatch.setattr(pose_source.tf2_ros, "TransformListener", Mock())
    active_map_changed = Mock()
    source = pose_source.NavigationSetupStateSource(
        node, active_map_changed=active_map_changed,
        monotonic_clock=lambda: steady[0],
    )

    def advance(seconds):
        steady[0] += seconds
        now_ns = clock.now().nanoseconds + int(seconds * 1e9)
        clock.set_ros_time_override(Time(nanoseconds=now_ns))
        buffer.lookup_transform.return_value = robot_transform(now_ns / 1e9 - 0.05)

    yield SimpleNamespace(
        source=source, buffer=buffer, clock=clock, steady=steady,
        advance=advance, active_map_changed=active_map_changed,
    )
    source.close()


def test_delayed_processed_estimate_saves_current_tf_not_historical_sensor_pose(rig):
    rig.source._receive_localization_pose(localization(stamp=8.7))
    rig.advance(0.6)
    result = rig.source.current_pose()
    assert isinstance(result, PoseData)
    assert result.position.x == 7.0  # TF now; the acquisition pose was x=2.
    args, kwargs = rig.buffer.lookup_transform.call_args
    assert args[:2] == ("map", "base_link")
    assert args[2].nanoseconds == 0
    assert kwargs["timeout"].nanoseconds == 0
    result.position.x = 50.0
    assert rig.source.current_pose().position.x == 7.0


def test_missing_estimate_has_actionable_error_even_if_tf_is_available(rig):
    with pytest.raises(ValueError, match="No localization estimate received yet"):
        rig.source.current_pose()
    rig.buffer.lookup_transform.assert_not_called()


@pytest.mark.parametrize("fault,detail", [
    ("wrong_frame", "map frame"),
    ("zero_stamp", "invalid or future timestamp"),
    ("future_stamp", "invalid or future timestamp"),
    ("late_stamp", "arrived 1.60 seconds old"),
    ("nan_position", "non-finite"),
    ("zero_quaternion", "Quaternion norm is zero"),
])
def test_invalid_acquisition_is_rejected_with_reason(rig, fault, detail):
    message = localization()
    if fault == "wrong_frame":
        message.header.frame_id = "odom"
    elif fault in {"zero_stamp", "future_stamp", "late_stamp"}:
        stamp = {"zero_stamp": 0.0, "future_stamp": 10.1, "late_stamp": 8.4}[fault]
        message.header.stamp = Time(seconds=stamp).to_msg()
    elif fault == "nan_position":
        message.pose.pose.position.x = float("nan")
    else:
        message.pose.pose.orientation.w = 0.0
    rig.source._receive_localization_pose(message)
    with pytest.raises(ValueError, match=detail):
        rig.source.current_pose()
    rig.buffer.lookup_transform.assert_not_called()


def test_age_boundaries_allow_exact_limit_without_extending_it(rig):
    rig.source._receive_localization_pose(localization(stamp=8.5))
    rig.advance(1.5)
    rig.buffer.lookup_transform.return_value = robot_transform(stamp=10.0)
    assert rig.source.current_pose().position.x == 7.0
    rig.advance(0.000001)
    with pytest.raises(ValueError, match="Localization estimate is stale"):
        rig.source.current_pose()


@pytest.mark.parametrize("fault", ["repeated", "out_of_order", "wrong_frame", "invalid_pose", "late"])
def test_rejected_updates_cannot_keep_localization_alive(rig, fault):
    rig.source._receive_localization_pose(localization(stamp=9.7))
    rig.advance(0.7)
    message = localization(stamp=10.5)
    if fault in {"repeated", "out_of_order", "late"}:
        stamp = {"repeated": 9.7, "out_of_order": 9.6, "late": 8.0}[fault]
        message.header.stamp = Time(seconds=stamp).to_msg()
    elif fault == "wrong_frame":
        message.header.frame_id = "odom"
    else:
        message.pose.pose.position.x = float("inf")
    rig.source._receive_localization_pose(message)
    assert rig.source.current_pose().position.x == 7.0
    rig.advance(0.81)
    # A continuously republished TF correction must not mask missing estimates.
    with pytest.raises(ValueError, match="Localization estimate is stale"):
        rig.source.current_pose()


def test_advancing_valid_estimate_renews_localization_liveness(rig):
    rig.source._receive_localization_pose(localization(stamp=8.7))
    rig.advance(1.0)
    rig.source._receive_localization_pose(localization(stamp=9.7))
    rig.advance(1.0)
    assert rig.source.current_pose().position.x == 7.0


def test_missing_tf_does_not_fall_back_to_historical_pose(rig):
    rig.source._receive_localization_pose(localization())
    rig.buffer.lookup_transform.side_effect = TransformException("missing odom")
    with pytest.raises(ValueError, match="map <- base_link is unavailable"):
        rig.source.current_pose()


@pytest.mark.parametrize("fault,detail", [
    ("stale", "transform is stale"),
    ("before_estimate", "transform is stale"),
    ("zero_stamp", "invalid or future timestamp"),
    ("future", "invalid or future timestamp"),
    ("wrong_parent", "must be map <- base_link"),
    ("wrong_child", "must be map <- base_link"),
    ("nan_position", "transform is invalid"),
    ("zero_rotation", "transform is invalid"),
])
def test_unusable_tf_is_rejected(rig, fault, detail):
    rig.source._receive_localization_pose(localization(stamp=9.0))
    transform = robot_transform()
    if fault in {"stale", "before_estimate", "zero_stamp", "future"}:
        stamp = {"stale": 8.4, "before_estimate": 8.9, "zero_stamp": 0.0, "future": 10.1}[fault]
        transform.header.stamp = Time(seconds=stamp).to_msg()
    elif fault == "wrong_parent":
        transform.header.frame_id = "odom"
    elif fault == "wrong_child":
        transform.child_frame_id = "body"
    elif fault == "nan_position":
        transform.transform.translation.x = float("nan")
    else:
        transform.transform.rotation.w = 0.0
    rig.buffer.lookup_transform.return_value = transform
    with pytest.raises(ValueError, match=detail):
        rig.source.current_pose()


def test_map_change_rejects_buffered_old_map_pose_and_retains_static_tf(rig):
    rig.source._receive_localization_pose(localization(stamp=9.5))
    rig.source._receive_active_map(String(data="new_map"))
    rig.active_map_changed.assert_called_once_with("new_map")
    with pytest.raises(ValueError, match="Map changed"):
        rig.source.current_pose()
    rig.advance(0.2)
    rig.source._receive_localization_pose(localization(stamp=9.8))
    with pytest.raises(ValueError, match="acquired after the map change"):
        rig.source.current_pose()
    rig.source._receive_localization_pose(localization(stamp=10.1))
    assert rig.source.current_pose().position.x == 7.0
    rig.source._receive_active_map(String(data="new_map"))
    assert rig.source.current_pose().position.x == 7.0
    rig.buffer.clear.assert_not_called()


def test_map_change_during_tf_lookup_prevents_cross_map_pose(rig):
    rig.source._receive_localization_pose(localization())

    def changing_lookup(*_args, **_kwargs):
        rig.source._receive_active_map(String(data="new_map"))
        return robot_transform()

    rig.buffer.lookup_transform.side_effect = changing_lookup
    with pytest.raises(ValueError, match="Map or clock changed"):
        rig.source.current_pose()


def test_ros_clock_rewind_invalidates_sample_and_resets_old_map_time_boundary(rig):
    rig.source._receive_active_map(String(data="map_a"))
    rig.source._receive_localization_pose(localization(stamp=10.0))
    rig.clock.set_ros_time_override(Time(seconds=5.0))
    with pytest.raises(ValueError, match="Clock changed"):
        rig.source.current_pose()
    rig.source._receive_localization_pose(localization(stamp=10.0))
    with pytest.raises(ValueError, match="invalid or future timestamp"):
        rig.source.current_pose()
    rig.source._receive_localization_pose(localization(stamp=4.5))
    rig.buffer.lookup_transform.return_value = robot_transform(stamp=4.9)
    assert rig.source.current_pose().position.x == 7.0
    rig.buffer.clear.assert_called_once_with()


def test_clock_rewind_clears_real_dynamic_tf_while_preserving_static_mount(rig):
    buffer = Buffer()
    rig.source._tf_buffer = buffer
    mount = robot_transform(stamp=0.0)
    mount.header.frame_id = "body"
    buffer.set_transform_static(mount, "test")

    def update_dynamic(stamp, map_offset, odom_position):
        for parent, child, position in (
            ("map", "odom", map_offset), ("odom", "body", odom_position),
        ):
            transform = robot_transform(stamp)
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = position
            buffer.set_transform(transform, "test")

    update_dynamic(9.9, 100.0, 1.0)
    rig.source._receive_localization_pose(localization(stamp=9.0))
    assert rig.source.current_pose().position.x == 108.0

    rig.clock.set_ros_time_override(Time(seconds=5.0))
    assert buffer.can_transform("body", "base_link", Time())
    assert not buffer.can_transform("map", "base_link", Time())
    update_dynamic(4.9, 20.0, 2.0)
    rig.source._receive_localization_pose(localization(stamp=4.5))
    assert rig.source.current_pose().position.x == 29.0


def test_close_unregisters_clock_jump_and_tf_listener_once(rig):
    handle = rig.source._clock_jump
    listener = rig.source._tf_listener
    rig.source.close()
    rig.source.close()
    listener.unregister.assert_called_once_with()
    assert rig.source._clock_jump is None
    assert rig.source._tf_listener is None
    # A real clock rewind after cleanup must not invoke the source callback.
    generation = rig.source._pose_generation
    rig.clock.set_ros_time_override(Time(seconds=5.0))
    assert rig.source._pose_generation == generation
    assert handle is not None


@pytest.mark.parametrize("maximum_age", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_freshness_limit_is_rejected(maximum_age):
    with pytest.raises(ValueError, match="positive and finite"):
        pose_source.NavigationSetupStateSource(Mock(), maximum_pose_age_sec=maximum_age)
