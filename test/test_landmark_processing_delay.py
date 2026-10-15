"""Processed localization updates keep their acquisition time and receipt age."""

from copy import deepcopy

import pytest
from fault_detector_msgs.msg import TagElementArray
from rclpy.time import Time

from test_landmark_save_quality import localization, rig as save_rig, tag  # noqa: F401
from test_landmark_recovery import (
    localization as recovery_localization,
    registration,
    rig as recovery_rig,  # noqa: F401
    tag as recovery_tag,
)
from test_navigation_setup_coordinator import coordinator


def receive_delayed_stream(rig, *, update_period=0.8, processing_delay=1.3):
    # The first acquisition must be newer than the active-map boundary.
    rig.advance(processing_delay + 0.1)
    latest = None
    for _ in range(3):
        now = rig.clock.now().nanoseconds / 1e9
        latest = localization(now - processing_delay)
        rig.source._receive_localization_pose(latest)
        for _ in range(round(update_period / 0.1)):
            now = rig.clock.now().nanoseconds / 1e9
            rig.source._receive_visible_tags(TagElementArray(elements=[tag(now)]))
            rig.advance(0.1)
    return latest


@pytest.mark.parametrize("update_period", [0.5, 0.8])
@pytest.mark.parametrize("processing_delay", [1.3, 2.05, 3.2])
def test_processed_localization_delay_does_not_reset_stable_save_captures(save_rig, update_period, processing_delay):
    latest = receive_delayed_stream(save_rig, update_period=update_period, processing_delay=processing_delay)

    assert save_rig.source.visible_tag_pose(7).position.x == pytest.approx(11.0)
    # TF and registration must continue to use the original acquisition time.
    assert save_rig.source._landmark_localization.header.stamp == latest.header.stamp
    acquisition_age = (
        save_rig.clock.now().nanoseconds - Time.from_msg(latest.header.stamp).nanoseconds
    ) / 1e9
    assert acquisition_age > 1.5


@pytest.mark.parametrize("gap_clock", ["both", "ros", "steady"])
def test_delayed_localization_still_expires_when_processed_updates_stop(save_rig, gap_clock):
    receive_delayed_stream(save_rig)
    if gap_clock == "ros":
        save_rig.clock.set_ros_time_override(
            Time(nanoseconds=save_rig.clock.now().nanoseconds + 1_600_000_000),
        )
    else:
        save_rig.advance(1.6, ros=gap_clock == "both")

    with pytest.raises(ValueError, match="stale"):
        save_rig.source.visible_tag_pose(7)
    assert not save_rig.source._qualified_landmarks


def test_estimate_already_too_old_on_arrival_cannot_save(save_rig):
    receive_delayed_stream(save_rig)
    save_rig.advance(5.0)
    now = save_rig.clock.now().nanoseconds / 1e9
    save_rig.source._receive_localization_pose(localization(now - 5.01))

    with pytest.raises(ValueError, match="old|stale"):
        save_rig.source.visible_tag_pose(7)
    assert not save_rig.source._qualified_landmarks


def test_replayed_delayed_estimate_does_not_extend_receipt_freshness(save_rig):
    receive_delayed_stream(save_rig)
    now = save_rig.clock.now().nanoseconds / 1e9
    latest = localization(now - 1.3)
    save_rig.source._receive_localization_pose(latest)
    original_receipt = save_rig.source._localization_received_at
    for _ in range(16):
        save_rig.advance(0.1, ros=False)
        save_rig.source._receive_localization_pose(deepcopy(latest))

    assert save_rig.source._localization_received_at == original_receipt
    with pytest.raises(ValueError, match="stale"):
        save_rig.source.visible_tag_pose(7)


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_delayed_stream_can_save_landmark_in_both_authoring_modes(save_rig, tmp_path, mode):
    receive_delayed_stream(save_rig, processing_delay=2.05)
    navigation, commands = coordinator(tmp_path)
    navigation.map_repository.create_empty("map_a")
    navigation.visible_tag_pose = save_rig.source.visible_tag_pose
    navigation.observe_runtime(mode, "map_a")
    context = navigation.open_context("navigation-ui").context

    navigation.add_visible_tag_landmark(context, "map_a", 7)

    saved = navigation.map_repository.load("map_a").localization_landmarks
    assert len(saved) == 1
    assert saved[0].reference_tag.tag_id == 7
    assert saved[0].pose_map.position.x == pytest.approx(11.0)
    assert commands.submitted == []


def test_fresh_delayed_localization_agreeing_with_landmark_never_resets_robot(recovery_rig):
    recovery_rig.advance(1.4)
    for _ in range(4):
        stamp = recovery_rig.clock[0] - 1.3
        recovery_rig.bb["last_pose_estimation"] = recovery_localization(stamp)
        recovery_rig.bb["last_localization_info"] = registration(stamp)
        for _ in range(8):
            recovery_rig.bb["visible_tags"] = {7: recovery_tag(recovery_rig.clock[0])}
            recovery_rig.tick()
            recovery_rig.advance(0.1)

    recovery_rig.publisher.publish.assert_not_called()
    assert recovery_rig.behavior.attempts == 0
    assert recovery_rig.behavior._quality_error(
        recovery_rig.bb["last_pose_estimation"],
        Time(seconds=recovery_rig.clock[0]).nanoseconds,
        recovery_rig.steady[0],
    ) == ""
