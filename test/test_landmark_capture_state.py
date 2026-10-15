"""Landmark qualification exists only for an explicit, cancellable capture."""

import pytest
from fault_detector_msgs.msg import TagElementArray
from rclpy.time import Time
from std_msgs.msg import String

from fault_detector_spot.navigation.landmark_observations import LandmarkObservationPending
from test_landmark_save_quality import localization, rig, tag  # noqa: F401


def test_idle_state_does_not_collect_or_transform_landmark_observations(rig):
    rig.source.finish_landmark_capture(rig.capture_token)
    for _ in range(12):
        rig.observe()
        rig.advance(0.1)

    rig.buffer.transform.assert_not_called()
    assert not rig.source._visible_tags
    assert not rig.source._landmark_windows
    assert not rig.source._landmark_pending_observations
    assert not rig.source._qualified_landmarks
    with pytest.raises(ValueError, match="No landmark capture") as error:
        rig.source.visible_tag_pose(7)
    assert not isinstance(error.value, LandmarkObservationPending)


def test_capture_requires_pose_and_tag_acquired_after_click(rig):
    rig.source.finish_landmark_capture(rig.capture_token)
    click = rig.clock.now().nanoseconds / 1e9
    token = rig.source.begin_landmark_capture(7)
    # Messages received after the click can still have been acquired before it.
    rig.source._receive_localization_pose(localization(click - 0.05))
    rig.advance(0.1)
    rig.source._receive_visible_tags(TagElementArray(elements=[tag(click + 0.1)]))
    with pytest.raises(LandmarkObservationPending, match="localization acquired after Save"):
        rig.source.visible_tag_pose(7)

    rig.source._receive_localization_pose(localization(click + 0.1))
    rig.source._receive_visible_tags(TagElementArray(elements=[tag(click)]))
    with pytest.raises(LandmarkObservationPending, match="tag observation acquired after Save"):
        rig.source.visible_tag_pose(7)
    rig.buffer.transform.assert_not_called()

    rig.advance(0.1)
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0
    rig.source.finish_landmark_capture(token)


def test_second_capture_never_reuses_previous_qualification(rig):
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0
    rig.source.finish_landmark_capture(rig.capture_token)
    token = rig.source.begin_landmark_capture(7)

    with pytest.raises(LandmarkObservationPending, match="localization acquired after Save"):
        rig.source.visible_tag_pose(7)
    assert not rig.source._qualified_landmarks
    assert not rig.source._landmark_windows
    rig.advance(0.1)
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0
    rig.source.finish_landmark_capture(token)


def test_only_requested_tag_is_qualified(rig):
    for _ in range(3):
        now = rig.clock.now().nanoseconds / 1e9
        other = tag(now)
        other.id = 8
        rig.source._receive_localization_pose(localization(now))
        rig.source._receive_visible_tags(TagElementArray(elements=[tag(now), other]))
        rig.advance(0.1)

    assert rig.buffer.transform.call_count == 3
    assert set(rig.source._qualified_landmarks) == {7}
    with pytest.raises(ValueError, match="does not match") as error:
        rig.source.visible_tag_pose(8)
    assert not isinstance(error.value, LandmarkObservationPending)


def test_capture_exclusivity_and_identity_safe_finish(rig):
    with pytest.raises(RuntimeError, match="already active"):
        rig.source.begin_landmark_capture(8)
    rig.source.finish_landmark_capture(object())
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0

    rig.source.finish_landmark_capture(rig.capture_token)
    newer = rig.source.begin_landmark_capture(7)
    rig.source.finish_landmark_capture(rig.capture_token)
    rig.source.finish_landmark_capture(rig.capture_token)
    assert rig.source._landmark_capture is newer
    rig.advance(0.1)
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0


def test_finish_during_tf_resolution_cannot_repopulate_capture(rig):
    def finish_while_transforming(source, target, **kwargs):
        rig.source.finish_landmark_capture(rig.capture_token)
        return rig.transform(source, target, **kwargs)

    rig.buffer.transform.side_effect = finish_while_transforming
    rig.observe()

    assert not rig.source._landmark_windows
    assert not rig.source._landmark_pending_observations
    assert not rig.source._qualified_landmarks
    with pytest.raises(ValueError, match="No landmark capture") as error:
        rig.source.visible_tag_pose(7)
    assert not isinstance(error.value, LandmarkObservationPending)


@pytest.mark.parametrize("change", ["map", "clock"])
def test_context_change_aborts_capture_instead_of_waiting_or_resuming(rig, change):
    rig.qualify()
    if change == "map":
        rig.source._receive_active_map(String(data="map_b"))
    else:
        rig.clock.set_ros_time_override(Time(seconds=5.0))
    rig.buffer.transform.reset_mock()
    rig.qualify()

    rig.buffer.transform.assert_not_called()
    with pytest.raises(ValueError, match="Map or clock changed") as error:
        rig.source.visible_tag_pose(7)
    assert not isinstance(error.value, LandmarkObservationPending)


@pytest.mark.parametrize("invalid_id", [True, -1, "7", None])
def test_invalid_capture_request_is_rejected_immediately(rig, invalid_id):
    rig.source.finish_landmark_capture(rig.capture_token)
    with pytest.raises(ValueError, match="non-negative integer") as error:
        rig.source.begin_landmark_capture(invalid_id)
    assert not isinstance(error.value, LandmarkObservationPending)


def test_missing_tag_reports_pending_and_can_recover_within_capture(rig):
    now = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(now))
    rig.source._receive_visible_tags(TagElementArray())
    with pytest.raises(LandmarkObservationPending, match="not currently visible"):
        rig.source.visible_tag_pose(7)
    rig.qualify()
    assert rig.source.visible_tag_pose(7).position.x == 11.0
