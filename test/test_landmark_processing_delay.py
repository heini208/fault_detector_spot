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
from fault_detector_spot.navigation.behaviours.landmark_relocalizer import LandmarkRecoveryConfig
from fault_detector_spot.navigation.ros.landmark_feedback import LandmarkFeedbackEvent


# Acquisition gaps and covariance from the reported RTAB-Map stream.
REPORTED_GAPS = (2.801172091, 1.899048059, 2.200392315, 2.399466294, 1.900419872)
NOT_LOCALIZED_COVARIANCE = [
    9999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 9999.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0e-8, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 3.0e-8, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 3.0e-8, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 9999.0,
]


def reported_unlocalized_pose(stamp):
    message = recovery_localization(stamp, x=0.8191075325012207)
    message.pose.pose.position.y = -1.3562772274017334
    message.pose.pose.position.z = 5.065793514251709
    message.pose.pose.orientation.z = 0.6968623779248597
    message.pose.pose.orientation.w = 0.7172049002936353
    message.pose.covariance = NOT_LOCALIZED_COVARIANCE
    return message


def receive_pose(rig, message):
    rig.bb["last_pose_estimation"] = message
    rig.bb["visible_tags"] = {7: recovery_tag(rig.clock[0])}
    rig.tick()


def tick_visible_tag(rig, duration):
    """Keep camera, odometry and the tree running between slow lidar results."""
    while duration > 1e-8:
        step = min(0.1, duration)
        rig.advance(step)
        rig.bb["visible_tags"] = {7: recovery_tag(rig.clock[0])}
        rig.tick()
        duration -= step


@pytest.mark.parametrize("delay", [0.0, 2.05, 3.2])
def test_reported_slow_not_localized_stream_requests_landmark_recovery(recovery_rig, delay):
    rig = recovery_rig
    rig.behavior.config = LandmarkRecoveryConfig()
    rig.advance(delay + 0.1)
    receive_pose(rig, reported_unlocalized_pose(rig.clock[0] - delay))
    for index, gap in enumerate(REPORTED_GAPS[:2]):
        tick_visible_tag(rig, gap)
        rig.publisher.publish.assert_not_called()
        assert rig.behavior._uncertain_count == index + 1
        receive_pose(rig, reported_unlocalized_pose(rig.clock[0] - delay))

    assert rig.publisher.publish.call_count == 1, rig.behavior.feedback_message
    seed = rig.publisher.publish.call_args.args[0]
    # Correction comes from the saved tag, not the bad five-metre height estimate.
    assert seed.pose.pose.position.x == pytest.approx(0.0)
    assert seed.pose.pose.position.z == pytest.approx(0.0)
    assert "recovery requested" in rig.behavior.feedback_message
    # The remaining supplied reports must not bypass the 15-second retry limit.
    for gap in REPORTED_GAPS[2:]:
        tick_visible_tag(rig, gap)
        receive_pose(rig, reported_unlocalized_pose(rig.clock[0] - delay))
    rig.publisher.publish.assert_called_once()


@pytest.mark.parametrize("gap_clock", ["both", "ros", "steady"])
def test_stopped_or_replayed_recovery_stream_expires_its_uncertainty(recovery_rig, gap_clock):
    rig = recovery_rig
    rig.behavior.config = LandmarkRecoveryConfig()
    rig.advance(0.1)
    receive_pose(rig, reported_unlocalized_pose(rig.clock[0]))
    tick_visible_tag(rig, REPORTED_GAPS[0])
    latest = reported_unlocalized_pose(rig.clock[0])
    receive_pose(rig, latest)
    assert rig.behavior._uncertain_count == 2

    if gap_clock == "ros":
        rig.clock[0] += 5.1
    else:
        rig.advance(5.1, ros=gap_clock == "both")
    receive_pose(rig, deepcopy(latest))
    assert rig.behavior._uncertain_count == 0
    assert "No advancing localization estimates" in rig.behavior.feedback_message
    rig.publisher.publish.assert_not_called()
    # A new estimate starts a new run; prior uncertainty cannot survive the gap.
    rig.advance(0.1)
    receive_pose(rig, reported_unlocalized_pose(rig.clock[0]))
    assert rig.behavior._uncertain_count == 1
    rig.publisher.publish.assert_not_called()


def test_confident_report_interrupts_slow_uncertainty_run(recovery_rig):
    rig = recovery_rig
    rig.behavior.config = LandmarkRecoveryConfig()
    rig.advance(0.1)
    receive_pose(rig, reported_unlocalized_pose(rig.clock[0]))
    tick_visible_tag(rig, REPORTED_GAPS[0])
    receive_pose(rig, reported_unlocalized_pose(rig.clock[0]))
    tick_visible_tag(rig, REPORTED_GAPS[1])
    receive_pose(rig, recovery_localization(rig.clock[0], x=1.0))
    assert rig.behavior._uncertain_count == 0
    for gap in REPORTED_GAPS[2:]:
        tick_visible_tag(rig, gap)
        rig.publisher.publish.assert_not_called()
        receive_pose(rig, reported_unlocalized_pose(rig.clock[0]))
    rig.publisher.publish.assert_called_once()


def test_slow_accepted_results_confirm_recovery_without_another_seed(recovery_rig):
    rig = recovery_rig
    rig.trigger(variance=9999.0)
    rig.behavior.config = LandmarkRecoveryConfig()
    for gap in REPORTED_GAPS[:3]:
        tick_visible_tag(rig, gap)
        rig.bb["last_localization_info"] = registration(rig.clock[0])
        receive_pose(rig, recovery_localization(rig.clock[0]))
    assert rig.behavior.attempts == 0
    rig.publisher.publish.assert_called_once()
    events = [LandmarkFeedbackEvent.from_message(call.args[0]).event for call in rig.events.publish.call_args_list]
    assert events == ["requested", "confirmed"]


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


@pytest.mark.parametrize("delay", [1.3, 2.05, 3.2])
@pytest.mark.parametrize("mismatch", [0.0, 1.0])
def test_fresh_delayed_confident_localization_never_resets_robot(recovery_rig, delay, mismatch):
    recovery_rig.advance(delay + 0.1)
    for _ in range(4):
        stamp = recovery_rig.clock[0] - delay
        recovery_rig.bb["last_pose_estimation"] = recovery_localization(stamp, x=mismatch)
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
