"""On-demand landmark saves yield the executor and bound data collection."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.action import ExecuteNavigationSetup
from fault_detector_msgs.msg import NavigationSetupIntent, NavigationSetupState, TagElementArray
from std_msgs.msg import String

from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
from fault_detector_spot.navigation.landmark_observations import LandmarkObservationPending
from test_command_request_correlation import FakeClock
from test_navigation_setup_coordinator import coordinator, map_pose
from test_landmark_save_quality import (
    localization as state_localization,
    rig as state_rig,  # noqa: F401
    tag as state_tag,
)


@pytest.fixture
def rig(tmp_path):
    navigation, commands = coordinator(tmp_path)
    navigation.map_repository.create_empty("plant")
    navigation.observe_runtime("localization", "plant")
    context = navigation.open_context("navigation-ui").context
    now = [100.0]
    navigation._monotonic_clock = lambda: now[0]
    navigation.begin_landmark_capture = Mock(return_value=object())
    navigation.finish_landmark_capture = Mock()
    navigation.visible_tag_pose = Mock(side_effect=LandmarkObservationPending("Waiting for localization"))
    return SimpleNamespace(navigation=navigation, commands=commands, context=context, now=now)


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_waiting_save_writes_only_after_new_observations_are_ready(rig, mode):
    navigation = rig.navigation
    navigation.observe_runtime(mode, "plant")
    context = navigation.context(rig.context.context_id, rig.context.client_id)
    request = navigation.begin_landmark_save(context, "plant", 7)
    with pytest.raises(LandmarkObservationPending, match="localization"):
        navigation.poll_pose_save(request)
    assert not navigation.map_repository.load("plant").localization_landmarks
    navigation.visible_tag_pose.side_effect = None
    navigation.visible_tag_pose.return_value = map_pose(7.0)
    snapshot = navigation.poll_pose_save(request)
    assert snapshot.landmark_names == ("Tag_7",)
    navigation.finish_landmark_capture.assert_called_once_with(request.capture)
    assert navigation._pose_save is None
    assert rig.commands.submitted == []


def test_timeout_retains_missing_data_reason_and_releases_collection(rig):
    request = rig.navigation.begin_landmark_save(rig.context, "plant", 7)
    with pytest.raises(LandmarkObservationPending):
        rig.navigation.poll_pose_save(request)
    rig.now[0] += 10.0  # Monotonic time advances independently of ROS time.
    with pytest.raises(TimeoutError, match="10 seconds: Waiting for localization"):
        rig.navigation.poll_pose_save(request)
    assert not rig.navigation.map_repository.load("plant").localization_landmarks
    assert rig.navigation._pose_save is None
    rig.navigation.finish_landmark_capture.assert_called_once_with(request.capture)


def test_pending_save_reserves_authoring_across_contexts_and_close_discards_it(rig):
    request = rig.navigation.begin_landmark_save(rig.context, "plant", 7)
    other = rig.navigation.open_context("other-ui").context
    with pytest.raises(RuntimeError, match="active operation"):
        rig.navigation.begin_landmark_save(other, "plant", 8)
    with pytest.raises(RuntimeError, match="active operation"):
        rig.navigation.add_current_waypoint(other, "plant", "waypoint")
    rig.navigation.close_context(rig.context)
    with pytest.raises(RuntimeError, match="closed or cancelled"):
        rig.navigation.poll_pose_save(request)
    rig.navigation.finish_landmark_capture.assert_called_once_with(request.capture)
    assert not rig.navigation.map_repository.load("plant").localization_landmarks


def test_runtime_change_cannot_save_in_old_context_or_map(rig):
    request = rig.navigation.begin_landmark_save(rig.context, "plant", 7)
    rig.navigation.observe_runtime("mapping", "other")
    rig.navigation.visible_tag_pose.side_effect = None
    rig.navigation.visible_tag_pose.return_value = map_pose()
    with pytest.raises((LookupError, ValueError, RuntimeError)):
        rig.navigation.poll_pose_save(request)
    assert not rig.navigation.map_repository.load("plant").localization_landmarks
    assert rig.navigation._pose_save is None


def api_rig(rig, *, operation=NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK):
    callbacks = []
    node = Mock()
    node.get_clock.return_value = FakeClock()
    node.create_timer.side_effect = lambda _period, callback, **kwargs: callbacks.append(callback) or callback
    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.node = node
    api.coordinator = rig.navigation
    api._lock = RLock()
    api._shutdown = Event()
    api._pose_waiters = set()
    api._state_publisher = Mock()
    api._callback_group = object()
    goal = ExecuteNavigationSetup.Goal(
        client_id=rig.context.client_id, context_id=rig.context.context_id,
        intent=NavigationSetupIntent(
            operation=operation,
            map_name="plant", tag_id=7, waypoint_name="motor_front",
        ),
    )
    handle = Mock(request=goal, is_cancel_requested=False)
    coroutine = api._execute(handle)
    waiting = coroutine.send(None)  # Waiting yields; no executor thread sleeps.
    assert waiting in api._pose_waiters
    assert not waiting.done()
    return api, handle, coroutine, callbacks[0]


def result(coroutine):
    with pytest.raises(StopIteration) as stopped:
        coroutine.send(None)
    return stopped.value.value


def test_action_reports_progress_then_saves_without_blocking_callbacks(rig):
    api, handle, coroutine, poll = api_rig(rig)
    feedback = handle.publish_feedback.call_args.args[0].state
    assert feedback.state == NavigationSetupState.STATE_RUNNING
    assert "Waiting for localization" in feedback.detail
    handle.abort.assert_not_called()
    rig.navigation.visible_tag_pose.side_effect = None
    rig.navigation.visible_tag_pose.return_value = map_pose()
    poll()
    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_SUCCEEDED
    assert final.state.request_id == feedback.request_id
    handle.succeed.assert_called_once()
    api.node.destroy_timer.assert_called_once_with(poll)


def test_context_closed_during_progress_resolves_action_instead_of_escaping_timer(rig):
    api, handle, coroutine, poll = api_rig(rig)

    def close_before_progress(_request):
        rig.navigation.close_context(rig.context)
        raise LandmarkObservationPending("Waiting for a new observation")

    rig.navigation.poll_pose_save = close_before_progress
    poll()
    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_FAILED
    assert not api._pose_waiters
    assert rig.navigation._pose_save is None
    api.node.destroy_timer.assert_called_once_with(poll)


def test_runtime_revision_change_delivers_terminal_failure_to_the_client(rig):
    api, handle, coroutine, poll = api_rig(rig)
    feedback = handle.publish_feedback.call_args.args[0].state
    rig.navigation.observe_runtime("none", "plant")
    current = rig.navigation.current_snapshot(rig.context)
    poll()
    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_FAILED
    assert final.state.request_id == feedback.request_id
    assert final.state.revision == current.context.revision
    assert final.state.mode == NavigationSetupState.MODE_NONE
    assert not rig.navigation.map_repository.load("plant").localization_landmarks
    assert not api._pose_waiters


@pytest.mark.parametrize("interruption", ["cancel", "shutdown", "close", "timeout"])
def test_action_interruption_never_saves_and_releases_all_resources(rig, interruption):
    api, handle, coroutine, poll = api_rig(rig)
    if interruption == "cancel":
        handle.is_cancel_requested = True
    elif interruption == "shutdown":
        api.request_shutdown()  # Must resolve even when timers will no longer run.
    elif interruption == "close":
        rig.navigation.close_context(rig.context)
    else:
        rig.now[0] += 10.0
    if interruption != "shutdown":
        poll()
    final = result(coroutine)
    expected = (NavigationSetupState.STATE_CANCELLED if interruption == "cancel"
                else NavigationSetupState.STATE_FAILED)
    assert final.state.state == expected
    assert not rig.navigation.map_repository.load("plant").localization_landmarks
    rig.navigation.finish_landmark_capture.assert_called_once()
    assert rig.navigation._pose_save is None
    assert not api._pose_waiters
    api.node.destroy_timer.assert_called_once_with(poll)


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_action_collects_new_state_observations_then_persists_landmark(rig, state_rig, mode):
    source = state_rig.source
    source.finish_landmark_capture(state_rig.capture_token)
    source._receive_active_map(String(data="plant"))
    before_click = state_rig.clock.now().nanoseconds / 1e9
    source._receive_localization_pose(state_localization(before_click))
    source._receive_visible_tags(TagElementArray(elements=[state_tag(before_click)]))
    state_rig.buffer.transform.assert_not_called()
    state_rig.advance(0.1)

    navigation = rig.navigation
    navigation.observe_runtime(mode, "plant")
    rig.context = navigation.context(rig.context.context_id, rig.context.client_id)
    navigation.begin_landmark_capture = source.begin_landmark_capture
    navigation.finish_landmark_capture = source.finish_landmark_capture
    navigation.visible_tag_pose = source.visible_tag_pose
    navigation._monotonic_clock = source._monotonic_clock
    api, handle, coroutine, poll = api_rig(rig)
    assert "localization acquired after Save" in handle.publish_feedback.call_args.args[0].state.detail

    # Delivery after the click does not make an earlier acquisition eligible.
    source._receive_localization_pose(state_localization(before_click))
    source._receive_visible_tags(TagElementArray(elements=[state_tag(before_click)]))
    poll()
    assert not navigation.map_repository.load("plant").localization_landmarks
    state_rig.buffer.transform.assert_not_called()
    handle.abort.assert_not_called()

    for sample in range(3):
        state_rig.advance(0.1)
        state_rig.observe()
        poll()
        if sample < 2:
            assert not navigation.map_repository.load("plant").localization_landmarks

    final = result(coroutine)
    assert final.state.state == NavigationSetupState.STATE_SUCCEEDED
    saved = navigation.map_repository.load("plant").localization_landmarks
    assert len(saved) == 1
    assert saved[0].reference_tag.tag_id == 7
    assert saved[0].pose_map.position.x == pytest.approx(11.0)
    assert saved[0].pose_map.position.z == pytest.approx(0.8)
    assert source._landmark_capture is None
    assert not source._landmark_windows
    assert not source._landmark_pending_observations
    assert not navigation._pose_save
    assert not api._pose_waiters
    assert rig.commands.submitted == []
    handle.succeed.assert_called_once()
    api.node.destroy_timer.assert_called_once_with(poll)
