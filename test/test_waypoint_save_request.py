"""Waypoint Save shares the bounded, cancellable navigation capture workflow."""

from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import NavigationSetupIntent, NavigationSetupState
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from std_msgs.msg import String

from fault_detector_spot.navigation.landmark_observations import NavigationObservationPending
from test_landmark_save_request import api_rig, result, rig as navigation_rig  # noqa: F401
from test_landmark_save_quality import localization, rig as state_rig  # noqa: F401
from test_navigation_setup_coordinator import map_pose


@pytest.fixture
def rig(navigation_rig):
    navigation = navigation_rig.navigation
    navigation.begin_waypoint_capture = Mock(return_value=object())
    navigation.waypoint_capture_pose = Mock(side_effect=NavigationObservationPending(
        "Waiting for localization acquired after Save was requested",
    ))
    navigation.finish_waypoint_capture = Mock()
    return navigation_rig


def waypoint_api(rig):
    return api_rig(rig, operation=NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT)


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_waypoint_action_waits_for_new_data_and_reports_progress(rig, mode):
    navigation = rig.navigation
    navigation.observe_runtime(mode, "plant")
    rig.context = navigation.context(rig.context.context_id, rig.context.client_id)
    api, handle, coroutine, poll = waypoint_api(rig)
    feedback = handle.publish_feedback.call_args.args[0].state
    assert feedback.state == NavigationSetupState.STATE_RUNNING
    assert "Collecting waypoint" in feedback.detail
    assert "acquired after Save" in feedback.detail
    assert not navigation.map_repository.load("plant").waypoints
    navigation.waypoint_capture_pose.side_effect = None
    navigation.waypoint_capture_pose.return_value = map_pose(4.0)
    poll()
    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_SUCCEEDED
    assert final.state.detail == "Waypoint saved"
    assert final.state.request_id == feedback.request_id
    assert navigation.map_repository.get_waypoint("plant", "motor_front").pose_map == map_pose(4.0)
    navigation.finish_waypoint_capture.assert_called_once()
    assert not navigation._pose_save
    assert not api._pose_waiters
    assert rig.commands.submitted == []
    api.node.destroy_timer.assert_called_once_with(poll)


@pytest.mark.parametrize("interruption", ["cancel", "shutdown", "close", "timeout", "runtime"])
def test_interrupted_waypoint_capture_never_writes(rig, interruption):
    api, handle, coroutine, poll = waypoint_api(rig)
    if interruption == "cancel":
        handle.is_cancel_requested = True
    elif interruption == "shutdown":
        api.request_shutdown()
    elif interruption == "close":
        rig.navigation.close_context(rig.context)
    elif interruption == "runtime":
        rig.navigation.observe_runtime("mapping", "other")
    else:
        rig.now[0] += 10.0
    if interruption != "shutdown":
        poll()
    final = result(coroutine)
    expected = (NavigationSetupState.STATE_CANCELLED if interruption == "cancel"
                else NavigationSetupState.STATE_FAILED)
    assert final.state.state == expected
    if interruption == "timeout":
        assert "Waypoint not saved after 10 seconds" in final.state.detail
        assert "acquired after Save" in final.state.detail
    assert not rig.navigation.map_repository.load("plant").waypoints
    rig.navigation.finish_waypoint_capture.assert_called_once()
    assert not rig.navigation._pose_save
    assert not api._pose_waiters
    api.node.destroy_timer.assert_called_once_with(poll)


def test_waypoint_capture_reserves_same_lane_as_landmarks_and_late_finish_is_safe(rig):
    navigation = rig.navigation
    request = navigation.begin_waypoint_save(rig.context, "plant", "motor_front")
    other = navigation.open_context("other-ui").context
    with pytest.raises(RuntimeError, match="active operation"):
        navigation.begin_landmark_save(other, "plant", 7)
    with pytest.raises(RuntimeError, match="active operation"):
        navigation.begin_waypoint_save(other, "plant", "other")
    navigation.finish_pose_save(request)
    newer = navigation.begin_waypoint_save(rig.context, "plant", "motor_front")
    navigation.finish_pose_save(request)
    assert navigation._pose_save is newer
    navigation.finish_waypoint_capture.assert_called_once_with(request.capture)
    navigation.finish_pose_save(newer)


def test_duplicate_waypoint_rejected_before_starting_capture(rig):
    navigation = rig.navigation
    rig.context = navigation.add_current_waypoint(rig.context, "plant", "motor_front").context
    with pytest.raises(FileExistsError, match="already exists"):
        navigation.begin_waypoint_save(rig.context, "plant", "motor_front")
    navigation.begin_waypoint_capture.assert_not_called()


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_waypoint_action_saves_current_tf_after_delayed_post_click_localization(rig, state_rig, mode):
    source = state_rig.source
    source.finish_landmark_capture(state_rig.capture_token)
    source._receive_active_map(String(data="plant"))
    before_click = state_rig.clock.now().nanoseconds / 1e9
    source._receive_localization_pose(localization(before_click))
    state_rig.advance(0.1)
    navigation = rig.navigation
    navigation.observe_runtime(mode, "plant")
    rig.context = navigation.context(rig.context.context_id, rig.context.client_id)
    navigation.begin_waypoint_capture = source.begin_waypoint_capture
    navigation.waypoint_capture_pose = source.waypoint_capture_pose
    navigation.finish_waypoint_capture = source.finish_waypoint_capture
    navigation._monotonic_clock = source._monotonic_clock
    api, handle, coroutine, poll = waypoint_api(rig)

    # A cached or re-delivered estimate acquired before Save cannot complete it.
    source._receive_localization_pose(localization(before_click))
    poll()
    assert not navigation.map_repository.load("plant").waypoints
    state_rig.buffer.lookup_transform.assert_not_called()

    state_rig.advance(0.1)
    acquired = state_rig.clock.now().nanoseconds / 1e9
    state_rig.advance(1.94)
    source._receive_localization_pose(localization(acquired))
    current_transform = TransformStamped()
    current_transform.header.frame_id = "map"
    current_transform.child_frame_id = "base_link"
    current_transform.header.stamp = state_rig.clock.now().to_msg()
    current_transform.transform.translation.x = 4.2
    current_transform.transform.rotation.w = 1.0
    state_rig.buffer.lookup_transform.return_value = current_transform
    poll()

    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_SUCCEEDED
    saved = navigation.map_repository.get_waypoint("plant", "motor_front")
    assert saved.pose_map.position.x == pytest.approx(4.2)
    assert source._localization_stamp_ns == Time(seconds=acquired).nanoseconds
    assert source._waypoint_capture is None
    assert not navigation._pose_save
    assert not api._pose_waiters
    assert rig.commands.submitted == []
    handle.succeed.assert_called_once()
