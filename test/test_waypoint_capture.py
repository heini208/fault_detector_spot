"""Waypoint authoring waits for new processed localization without an idle cache."""

from copy import deepcopy

import pytest
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from fault_detector_spot.navigation.landmark_observations import NavigationObservationPending
from test_landmark_save_quality import localization, rig as source_rig  # noqa: F401
from test_navigation_pose_boundary import robot_transform


@pytest.fixture
def rig(source_rig):
    source_rig.source.finish_landmark_capture(source_rig.capture_token)
    source_rig.capture_token = source_rig.source.begin_waypoint_capture()
    source_rig.buffer.lookup_transform.side_effect = None
    source_rig.buffer.lookup_transform.return_value = robot_transform(10.0)
    return source_rig


def arrive(rig, *, delay=2.05, variance=0.01):
    acquired = rig.clock.now().nanoseconds / 1e9 + 0.1
    rig.advance(delay + 0.1)
    rig.source._receive_localization_pose(localization(acquired, variance=variance))
    rig.buffer.lookup_transform.return_value = robot_transform(rig.clock.now().nanoseconds / 1e9 - 0.01)
    return acquired


@pytest.mark.parametrize("delay", [1.74, 1.94, 2.05, 3.2, 5.0])
def test_new_delayed_result_saves_current_robot_tf_not_old_sensor_pose(rig, delay):
    with pytest.raises(NavigationObservationPending, match="acquired after Save"):
        rig.source.waypoint_capture_pose(rig.capture_token)
    acquired = arrive(rig, delay=delay)
    pose = rig.source.waypoint_capture_pose(rig.capture_token)
    assert pose.position.x == 7.0
    assert rig.source._localization_stamp_ns == Time(seconds=acquired).nanoseconds
    assert rig.source._landmark_localization.header.stamp == Time(seconds=acquired).to_msg()
    assert rig.buffer.lookup_transform.call_args.args[2].nanoseconds == 0
    rig.buffer.transform.assert_not_called()
    assert not rig.source._landmark_windows


def test_received_after_click_but_acquired_before_click_remains_pending(rig):
    old_stamp = rig.clock.now().nanoseconds / 1e9 - 0.01
    rig.advance(0.2)
    rig.source._receive_localization_pose(localization(old_stamp))
    with pytest.raises(NavigationObservationPending, match="acquired after Save"):
        rig.source.waypoint_capture_pose(rig.capture_token)
    rig.buffer.lookup_transform.assert_not_called()


@pytest.mark.parametrize("variance", [0.0, 9999.0, 0.5, float("nan")])
def test_delay_allowance_does_not_relax_covariance_quality(rig, variance):
    arrive(rig, variance=variance)
    with pytest.raises(NavigationObservationPending, match="covariance|uncertainty"):
        rig.source.waypoint_capture_pose(rig.capture_token)


def test_processing_delay_beyond_capture_limit_is_rejected(rig):
    arrive(rig, delay=5.01)
    with pytest.raises(NavigationObservationPending):
        rig.source.waypoint_capture_pose(rig.capture_token)
    assert "arrived 5.01 seconds old" in rig.source._localization_error


@pytest.mark.parametrize("clock", ["ros", "steady", "both"])
def test_stopped_or_replayed_results_cannot_keep_capture_alive(rig, clock):
    arrive(rig)
    message = deepcopy(rig.source._landmark_localization)
    if clock == "ros":
        rig.clock.set_ros_time_override(Time(nanoseconds=rig.clock.now().nanoseconds + 1_600_000_000))
    else:
        rig.advance(1.6, ros=clock == "both")
    received = rig.source._localization_received_at
    rig.source._receive_localization_pose(message)
    assert rig.source._localization_received_at == received
    with pytest.raises(NavigationObservationPending, match="stale"):
        rig.source.waypoint_capture_pose(rig.capture_token)


@pytest.mark.parametrize("failure", ["stale", "missing", "before_estimate"])
def test_capture_waits_for_usable_current_tf(rig, failure):
    acquired = arrive(rig)
    if failure == "missing":
        rig.buffer.lookup_transform.side_effect = TransformException("TF pending")
    else:
        stamp = (rig.clock.now().nanoseconds / 1e9 - 1.6
                 if failure == "stale" else acquired - 0.01)
        rig.buffer.lookup_transform.return_value = robot_transform(stamp)
    with pytest.raises(NavigationObservationPending, match="transform.*stale|unavailable"):
        rig.source.waypoint_capture_pose(rig.capture_token)
    rig.buffer.lookup_transform.side_effect = None
    rig.buffer.lookup_transform.return_value = robot_transform(rig.clock.now().nanoseconds / 1e9)
    assert rig.source.waypoint_capture_pose(rig.capture_token).position.x == 7.0


def test_finishing_capture_restores_strict_live_reader_and_second_click_needs_new_data(rig):
    arrive(rig)
    assert rig.source.waypoint_capture_pose(rig.capture_token).position.x == 7.0
    rig.source.finish_waypoint_capture(rig.capture_token)
    with pytest.raises(ValueError, match="outside its requested capture"):
        rig.source.current_pose()
    second = rig.source.begin_waypoint_capture()
    rig.source.finish_waypoint_capture(rig.capture_token)
    with pytest.raises(NavigationObservationPending, match="acquired after Save"):
        rig.source.waypoint_capture_pose(second)
    assert rig.source._waypoint_capture is second


@pytest.mark.parametrize("change", ["map", "clock", "cancel"])
def test_capture_context_changes_are_terminal_not_pending(rig, change):
    arrive(rig)
    if change == "map":
        rig.source._receive_active_map(String(data="map_b"))
    elif change == "clock":
        rig.clock.set_ros_time_override(Time(seconds=5.0))
    else:
        rig.source.finish_waypoint_capture(rig.capture_token)
    with pytest.raises(ValueError) as exception:
        rig.source.waypoint_capture_pose(rig.capture_token)
    assert not isinstance(exception.value, NavigationObservationPending)


def test_map_change_during_tf_read_is_terminal(rig):
    arrive(rig)

    def lookup(*_args, **_kwargs):
        rig.source._receive_active_map(String(data="map_b"))
        return robot_transform(rig.clock.now().nanoseconds / 1e9)

    rig.buffer.lookup_transform.side_effect = lookup
    with pytest.raises(ValueError, match="Map or clock changed") as exception:
        rig.source.waypoint_capture_pose(rig.capture_token)
    assert not isinstance(exception.value, NavigationObservationPending)


def test_capture_exclusivity_and_close_release_both_kinds(rig):
    with pytest.raises(RuntimeError, match="already active"):
        rig.source.begin_landmark_capture(7)
    with pytest.raises(RuntimeError, match="already active"):
        rig.source.begin_waypoint_capture()
    rig.source.close()
    assert rig.source._waypoint_capture is None
    assert rig.source._landmark_capture is None
