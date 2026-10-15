"""Automatic refinement through real metadata and timestamped offline TF."""

from dataclasses import replace
import json
from unittest.mock import Mock

import py_trees
import pytest
from rclpy.time import Time

from fault_detector_spot.navigation.landmark_refinement import LandmarkRefinementConfig
from fault_detector_spot.navigation.ros.landmark_feedback import LandmarkFeedbackEvent
from test_landmark_recovery import localization, registration, rig, tag  # noqa: F401


pytestmark = pytest.mark.parametrize("rig", [LandmarkRefinementConfig()], indirect=True)


def lidar_registration(stamp):
    message = registration(stamp)
    message.proximity_detection_id = 42
    message.stats_keys.extend([
        "Proximity/Space_detections_added_icp_global/",
        "Proximity/Space_detections_added_icp_multi/",
    ])
    message.stats_values.extend([1.0, 0.0])
    return message


def observe(rig, *, count=14, mismatch=0.05, variance=0.001, accepted=True, distance=2.0):
    for _ in range(count):
        rig.advance()
        stamp = rig.clock[0]
        rig.bb["visible_tags"] = {7: tag(stamp, x=distance)}
        rig.bb["last_pose_estimation"] = localization(stamp, x=mismatch, variance=variance)
        rig.bb["last_localization_info"] = (
            lidar_registration(stamp) if accepted else registration(stamp, accepted=False)
        )
        rig.tick()


def saved(rig):
    return rig.repo.get_landmark("map_a", "Tag_7")


def test_small_qualified_refinement_preserves_original_and_updates_recovery_anchor(rig):
    observe(rig)
    anchor = saved(rig)
    assert anchor.pose_map.position.x == pytest.approx(2.005)
    assert anchor.refinement.original_pose_map.position.x == 2.0
    assert anchor.refinement.update_count == 1
    assert anchor.refinement.last_refined_at == pytest.approx(1_700_000_000 + 102.75)
    assert rig.behavior.landmark_map[7].position.x == pytest.approx(2.005)
    rig.publisher.publish.assert_not_called()
    event = LandmarkFeedbackEvent.from_message(rig.events.publish.call_args.args[0])
    assert (event.event, event.map_id, event.tag_id) == ("refined", "map_a", 7)
    rig.events.publish.assert_called_once()


def test_continuous_visibility_cannot_repeat_even_after_cooldown_and_metadata_reload(rig):
    observe(rig)
    rig.advance(61.0)
    definition = rig.repo.load("map_a")
    definition.display_name = "Renamed map"
    rig.repo.save("map_a", definition)
    observe(rig)
    assert saved(rig).refinement.update_count == 1


def test_failed_refinement_has_a_bounded_tf_budget_and_skips_work_during_cooldown(rig):
    rig.buffer.unavailable = True
    rig.buffer.lookup_transform = Mock(wraps=rig.buffer.lookup_transform)
    observe(rig, count=124)
    # Thirty seconds at 2 Hz, even though the command tree ticks four times/sec.
    assert rig.buffer.lookup_transform.call_count == 60
    rig.buffer.lookup_transform.reset_mock()
    rig.buffer.unavailable = False
    observe(rig, count=100)
    rig.buffer.lookup_transform.assert_not_called()
    rig.events.publish.assert_not_called()
    assert saved(rig).refinement is None
    rig.advance(300.0)
    observe(rig)
    assert saved(rig).refinement.update_count == 1


def test_successful_encounter_skips_tf_and_geometry_even_after_attempt_interval(rig):
    observe(rig)
    rig.buffer.lookup_transform = Mock(wraps=rig.buffer.lookup_transform)
    rig.advance(301.0)
    observe(rig)
    rig.buffer.lookup_transform.assert_not_called()
    rig.events.publish.assert_called_once()


def test_new_encounter_requires_both_absence_and_interval(rig):
    observe(rig)
    rig.bb["visible_tags"] = {}
    rig.tick()
    rig.advance(6.0)
    observe(rig)
    assert saved(rig).refinement.update_count == 1
    rig.advance(300.0)
    observe(rig)
    assert saved(rig).refinement.update_count == 2
    assert saved(rig).refinement.original_pose_map.position.x == 2.0


def test_out_of_range_tag_does_not_consume_attempt_before_becoming_usable(rig):
    rig.buffer.lookup_transform = Mock(wraps=rig.buffer.lookup_transform)
    observe(rig, count=32, distance=2.51)
    rig.buffer.lookup_transform.assert_not_called()
    assert rig.behavior._refiner.attempt_tag_id is None
    observe(rig, distance=2.5, mismatch=-0.45)
    assert saved(rig).refinement.update_count == 1
    assert saved(rig).pose_map.position.x == pytest.approx(2.005)


def test_lost_tag_can_refine_again_after_rediscovery_in_range_but_range_alone_cannot_rearm(rig):
    observe(rig)
    rig.advance(301.0)
    observe(rig, count=32, distance=2.51)
    observe(rig)
    assert saved(rig).refinement.update_count == 1
    rig.bb["visible_tags"] = {}
    rig.tick()
    rig.advance(5.1)
    observe(rig, count=32, distance=2.51)
    observe(rig)
    assert saved(rig).refinement.update_count == 2
    events = [LandmarkFeedbackEvent.from_message(call.args[0]).event for call in rig.events.publish.call_args_list]
    assert events == ["refined", "refined"]


@pytest.mark.parametrize("period", [1.9, 2.8, 4.9])
def test_confident_slow_lidar_can_refine_with_continuous_stationarity_checks(rig, period):
    rig.behavior.config = replace(rig.behavior.config, stationary_duration_sec=1.0)
    rig.advance(2.05 + 0.25)
    for _ in range(6):
        # Each accepted result is new; camera/TF/tree continue between results.
        stamp = rig.clock[0]
        rig.bb["last_pose_estimation"] = localization(stamp - 2.05, x=0.05, variance=0.001)
        rig.bb["last_localization_info"] = lidar_registration(stamp - 2.05)
        remaining = period
        while remaining > 1e-8:
            rig.bb["visible_tags"] = {7: tag(rig.clock[0])}
            rig.tick()
            step = min(0.1, remaining)
            rig.advance(step)
            remaining -= step
    assert saved(rig).refinement.update_count == 1
    assert saved(rig).pose_map.position.x == pytest.approx(2.005)
    rig.publisher.publish.assert_not_called()
    rig.events.publish.assert_called_once()


@pytest.mark.parametrize("fault", ["motion", "missing_odometry"])
def test_stationarity_fault_between_lidar_results_discards_refinement_samples(rig, fault):
    observe(rig, count=9)  # Four qualified samples; one more would allow an update.
    assert saved(rig).refinement is None
    rig.advance(0.5)
    rig.buffer.position = 0.1 if fault == "motion" else 0.0
    rig.buffer.unavailable = fault == "missing_odometry"
    rig.bb["visible_tags"] = {7: tag(rig.clock[0])}
    rig.tick()  # The localization/Info pair has not changed.
    rig.buffer.position = 0.0
    rig.buffer.unavailable = False
    observe(rig, count=2)
    assert saved(rig).refinement is None
    rig.events.publish.assert_not_called()
    observe(rig)
    assert saved(rig).refinement.update_count == 1


@pytest.mark.parametrize("variance,accepted,mismatch", [
    (0.01, True, 0.05),  # Good enough for recovery, too uncertain for refinement.
    (0.001, False, 0.05),
    (0.001, True, 0.2),  # Stable observations may still be an outlier/moved tag.
])
def test_weak_evidence_or_large_discrepancy_does_not_modify_anchor(rig, variance, accepted, mismatch):
    observe(rig, variance=variance, accepted=accepted, mismatch=mismatch)
    assert saved(rig).refinement is None


def test_many_camera_captures_with_one_lidar_registration_do_not_qualify(rig):
    rig.advance()
    stamp = rig.clock[0]
    rig.bb["last_pose_estimation"] = localization(stamp, x=0.05, variance=0.001)
    rig.bb["last_localization_info"] = lidar_registration(stamp)
    for _ in range(10):
        rig.advance(0.1)
        rig.bb["visible_tags"] = {7: tag(rig.clock[0])}
        rig.tick()
    assert saved(rig).refinement is None


@pytest.mark.parametrize("expired_clock", ["both", "ros", "steady"])
def test_recovery_stream_gap_does_not_extend_refinement_receipt_freshness(rig, expired_clock):
    # Receive a healthy pose first; its separate Info callback has not arrived.
    rig.advance()
    stamp = rig.clock[0]
    rig.bb["last_pose_estimation"] = localization(stamp, x=0.05, variance=0.001)
    rig.tick()
    rig.behavior._refiner.begin_samples = Mock(wraps=rig.behavior._refiner.begin_samples)
    rig.buffer.lookup_transform = Mock(wraps=rig.buffer.lookup_transform)
    rig.repo.refine_landmark = Mock(wraps=rig.repo.refine_landmark)

    # Recovery may retain this estimate between slow processed results, but
    # delayed matching Info must not revive it for landmark refinement.
    if expired_clock == "ros":
        rig.clock[0] += 2.0
    else:
        rig.advance(2.0, ros=expired_clock == "both")
    rig.bb["last_localization_info"] = lidar_registration(stamp)
    rig.bb["visible_tags"] = {7: tag(rig.clock[0])}
    rig.tick()

    assert rig.behavior._quality_error(
        rig.bb["last_pose_estimation"],
        Time(seconds=rig.clock[0]).nanoseconds,
        rig.steady[0],
    ) == ""
    rig.behavior._refiner.begin_samples.assert_not_called()
    rig.buffer.lookup_transform.assert_not_called()
    rig.repo.refine_landmark.assert_not_called()
    assert saved(rig).refinement is None
    rig.publisher.publish.assert_not_called()
    rig.events.publish.assert_not_called()


def test_mismatched_registration_timestamps_do_not_qualify(rig):
    for _ in range(9):
        rig.advance()
        stamp = rig.clock[0]
        rig.bb["visible_tags"] = {7: tag(stamp)}
        rig.bb["last_pose_estimation"] = localization(stamp, x=0.05, variance=0.001)
        rig.bb["last_localization_info"] = lidar_registration(stamp - 0.1)
        rig.tick()
    assert saved(rig).refinement is None


@pytest.mark.parametrize("running,moving", [(True, False), (False, True)])
def test_refinement_waits_for_idle_stationary_robot(rig, running, moving):
    if running:
        rig.bb["command_tree_status"] = py_trees.common.Status.RUNNING
    for _ in range(10):
        if moving:
            rig.buffer.position += 0.1
        observe(rig, count=1)
    assert saved(rig).refinement is None


def test_seed_blocks_refinement_across_metadata_reload_and_clock_reset(rig):
    callback = rig.node.create_subscription.call_args.args[2]
    callback(localization(rig.clock[0]))
    observe(rig)
    definition = rig.repo.load("map_a")
    definition.localization_landmarks[0].pose_map.position.x += 0.001
    rig.repo.save("map_a", definition)
    rig.clock[0] -= 2.0
    rig.tick()
    observe(rig)
    assert saved(rig).refinement is None
    # A newly owned runtime must collect fresh evidence again.
    rig.runtime.process = object()
    rig.tick()
    observe(rig, count=4)
    assert saved(rig).refinement is None
    observe(rig)
    assert saved(rig).refinement.update_count == 1


def test_own_recovery_seed_blocks_before_publication_and_after_recovery(rig):
    rig.publisher.publish.side_effect = lambda _msg: (
        pytest.fail("Refinement provenance was not blocked before seed publication")
        if not rig.behavior._refinement_seeded("map_a") else None
    )
    rig.trigger()
    observe(rig, count=20)
    assert saved(rig).refinement is None


def test_attaching_to_existing_process_cannot_assume_seed_history(rig):
    rig.behavior.shutdown()
    rig.behavior.setup()
    observe(rig)
    assert saved(rig).refinement is None


def test_mapping_mode_does_not_refine_an_evolving_map(rig):
    rig.runtime.get_running_mode = lambda: "mapping"
    observe(rig)
    assert saved(rig).refinement is None


def test_refinement_can_be_enabled_with_recovery_disabled(rig):
    rig.behavior.config = replace(rig.behavior.config, enabled=False)
    observe(rig)
    assert saved(rig).refinement.update_count == 1


def test_capture_time_odometry_accounts_for_motion_between_lidar_and_tag(rig):
    rig.buffer.position = 0.2
    rig.advance(1.0)
    for _ in range(14):
        rig.advance()
        stamp = rig.clock[0]
        lidar_stamp = stamp - 0.5
        rig.buffer.history[Time(seconds=lidar_stamp).nanoseconds] = 0.0
        rig.buffer.history[Time(seconds=stamp).nanoseconds] = 0.2
        rig.bb["visible_tags"] = {7: tag(stamp, x=1.8)}
        rig.bb["last_pose_estimation"] = localization(lidar_stamp, x=0.05, variance=0.001)
        rig.bb["last_localization_info"] = lidar_registration(lidar_stamp)
        rig.tick()
    assert saved(rig).pose_map.position.x == pytest.approx(2.005)


def test_deletion_during_proposal_does_not_recreate_landmark(rig):
    original = rig.repo.refine_landmark

    def delete_then_commit(*args, **kwargs):
        rig.repo.delete_landmark("map_a", "Tag_7")
        return original(*args, **kwargs)

    rig.repo.refine_landmark = Mock(side_effect=delete_then_commit)
    observe(rig)
    assert saved(rig) is None
    assert rig.repo.refine_landmark.call_count == 1
    rig.events.publish.assert_not_called()


def test_incomplete_refinement_metadata_disables_anchor_without_crashing_monitor(rig):
    path = rig.repo.get_map_path("map_a")
    data = json.loads(path.read_text())
    data["localization_landmarks"][0]["refinement"] = {"update_count": 1}
    path.write_text(json.dumps(data))
    rig.tick()
    assert rig.behavior.landmark_map == {}
    assert "Failed to load landmarks" in rig.behavior.feedback_message


@pytest.mark.parametrize("frame,unavailable", [("map", False), ("body", True)])
def test_nonrelative_tag_or_missing_capture_transform_cannot_refine(rig, frame, unavailable):
    rig.buffer.unavailable = unavailable
    for _ in range(9):
        rig.advance()
        stamp = rig.clock[0]
        rig.bb["visible_tags"] = {7: tag(stamp, frame=frame)}
        rig.bb["last_pose_estimation"] = localization(stamp, x=0.05, variance=0.001)
        rig.bb["last_localization_info"] = lidar_registration(stamp)
        rig.tick()
    assert saved(rig).refinement is None
