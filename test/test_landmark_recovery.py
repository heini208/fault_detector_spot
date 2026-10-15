"""Offline recovery supervision with timestamped TF and real map metadata."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import py_trees
import pytest
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from tf2_ros import TransformException

from fault_detector_spot.inspection.model.models import ReferenceTag
from fault_detector_spot.mapping.model.models import LocalizationLandmark
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.navigation.behaviours.landmark_relocalizer import (
    LandmarkRecoveryConfig, LandmarkRelocalizer,
)
from fault_detector_spot.navigation.localization_quality import LocalizationQualityConfig
from fault_detector_spot.navigation.landmark_refinement import LandmarkRefinementConfig
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.shared.geometry.transforms import pose_to_matrix


class Blackboard:
    def __init__(self):
        self.values = {
            "active_map_name": "map_a", "visible_tags": {},
            "command_tree_status": py_trees.common.Status.FAILURE,
        }

    def register_key(self, *_args, **_kwargs):
        pass

    def exists(self, key):
        return key in self.values

    def get(self, key):
        return self.values[key]

    def set(self, key, value):
        self.values[key] = value


class TimestampedTF:
    """Explicit capture-time and current odometry; no ROS graph or nodes."""

    def __init__(self, clock):
        self.clock = clock
        self.position = 0.0
        self.frame_heights = {"feet_center": 0.0, "base_link": 0.0, "body": 0.0, "camera": 0.0}
        self.history = {}
        self.latest_stamp_delta = 0.0
        self.history_stamp_delta = 0.0
        self.invalid_frame = False
        self.unavailable = False

    def lookup_transform(self, target, source, stamp, *, timeout):
        assert timeout.nanoseconds == 0
        if self.unavailable:
            raise TransformException("missing capture transform")
        assert target == "odom"
        assert source in self.frame_heights
        current = stamp.nanoseconds == 0
        when = self.clock[0] if current else stamp.nanoseconds / 1e9
        delta = self.latest_stamp_delta if current else self.history_stamp_delta
        result = TransformStamped()
        result.header.frame_id = "wrong" if self.invalid_frame else target
        result.child_frame_id = source
        result.header.stamp = Time(seconds=when + delta).to_msg()
        result.transform.translation.x = self.position if current else self.history.get(stamp.nanoseconds, self.position)
        result.transform.translation.z = self.frame_heights[source]
        result.transform.rotation.w = 1.0
        return result


def landmark(tag_id=7, x=2.0):
    pose = PoseData.identity()
    pose.position.x = x
    return LocalizationLandmark(
        landmark_id=f"Tag_{tag_id}", display_name=f"Tag_{tag_id}",
        reference_tag=ReferenceTag(tag_id=tag_id, tag_family="36h11"), pose_map=pose,
    )


def tag(stamp, *, x=2.0, z=0.0, frame="body", tag_id=7):
    result = TagElement()
    result.id = tag_id
    result.pose.header.frame_id = frame
    result.pose.header.stamp = Time(seconds=stamp).to_msg()
    result.pose.pose.position.x = x
    result.pose.pose.position.z = z
    result.pose.pose.orientation.w = 1.0
    return result


def localization(stamp, *, x=0.0, variance=0.01):
    result = PoseWithCovarianceStamped()
    result.header.frame_id = "map"
    result.header.stamp = Time(seconds=stamp).to_msg()
    result.pose.pose.position.x = x
    result.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        result.pose.covariance[index] = variance
    return result


def registration(stamp, *, accepted=True):
    result = Info()
    result.header.frame_id = "map"
    result.header.stamp = Time(seconds=stamp).to_msg()
    result.loop_closure_id = 42 if accepted else 0
    result.stats_keys = ["Loop/RejectedHypothesis/"]
    result.stats_values = [0.0 if accepted else 1.0]
    return result


@pytest.fixture
def rig(tmp_path, request):
    clock = [10.0]
    steady = [100.0]
    node = Mock()
    node.get_clock.return_value.now.side_effect = lambda: Time(seconds=clock[0])
    publisher = Mock()
    node.create_publisher.return_value = publisher
    runtime = SimpleNamespace(
        process=None, get_running_mode=lambda: "localization", maps_dir=tmp_path,
        localization_frame="feet_center",
    )
    repository = MapRepository(tmp_path)
    repository.create_empty("map_a")
    repository.add_landmark("map_a", landmark())
    buffer = TimestampedTF(clock)
    config = LandmarkRecoveryConfig(
        required_samples=3, maximum_sample_span_sec=4.0,
        maximum_position_spread_m=0.02, settle_time_sec=0.2,
        retry_interval_sec=1.0, maximum_attempts=2,
        recovery_position_tolerance_m=0.1, stationary_duration_sec=0.2,
    )
    behavior = LandmarkRelocalizer(
        runtime, buffer, repository, node, config=config,
        quality_config=LocalizationQualityConfig(), monotonic_clock=lambda: steady[0],
        refinement_config=getattr(request, "param", LandmarkRefinementConfig(enabled=False)),
        wall_clock=lambda: 1_700_000_000.0 + steady[0],
    )
    blackboard = Blackboard()
    behavior.blackboard = blackboard
    behavior.setup()
    runtime.process = object()
    assert behavior.update() == py_trees.common.Status.SUCCESS

    def advance(seconds=0.25, *, ros=True):
        steady[0] += seconds
        if ros:
            clock[0] += seconds

    def tick():
        assert behavior.update() == py_trees.common.Status.SUCCESS

    def sample(*, seconds=0.25, variance=0.5, mismatch=0.0, frame="body", info=None, move=0.0, tag_z=0.0):
        advance(seconds)
        buffer.position += move
        stamp_ns = Time(seconds=clock[0]).nanoseconds
        buffer.history[stamp_ns] = buffer.position
        blackboard.values["visible_tags"] = {
            7: tag(clock[0], x=2.0 - buffer.position, z=tag_z, frame=frame),
        }
        blackboard.values["last_pose_estimation"] = localization(
            clock[0], x=buffer.position + mismatch, variance=variance,
        )
        if info is not None:
            blackboard.values["last_localization_info"] = registration(clock[0], accepted=info)
        tick()

    def trigger(**kwargs):
        before = publisher.publish.call_count
        for _ in range(12):
            sample(**kwargs)
            if publisher.publish.call_count > before:
                return deepcopy(publisher.publish.call_args.args[0])
        pytest.fail(f"No correction: {behavior.feedback_message}")

    yield SimpleNamespace(
        behavior=behavior, bb=blackboard.values, clock=clock, steady=steady,
        runtime=runtime, repo=repository, buffer=buffer, node=node,
        publisher=publisher, advance=advance, tick=tick, sample=sample, trigger=trigger,
    )
    behavior.shutdown()


@pytest.mark.parametrize("variance,mismatch", [(0.5, 0.0), (0.01, 1.0)])
def test_uncertainty_or_confident_landmark_disagreement_requests_correction(rig, variance, mismatch):
    message = rig.trigger(variance=variance, mismatch=mismatch)
    assert message.header.frame_id == "map"
    assert message.header.stamp == Time(seconds=rig.clock[0]).to_msg()
    assert message.pose.pose.position.x == pytest.approx(0.0)
    assert rig.behavior.attempts == 1
    assert "requested" in rig.behavior.feedback_message


@pytest.mark.parametrize("reference_frame,expected_seed_height", [("feet_center", 0.0), ("base_link", 0.6)])
def test_recovery_seed_uses_active_maps_robot_pose_reference(rig, reference_frame, expected_seed_height):
    map_name = "reference_map"
    if reference_frame == "base_link":
        # A legacy database has body-referenced scan geometry. Creating its
        # metadata must retain that convention rather than relabel its poses.
        rig.repo.get_map_path(map_name).with_suffix(".db").touch()
    definition = rig.repo.create_empty(map_name)
    assert definition.localization_frame == reference_frame
    anchor = landmark()
    anchor.pose_map.position.z = 1.0
    rig.repo.add_landmark(map_name, anchor)
    rig.buffer.frame_heights.update(body=0.6, base_link=0.6, feet_center=0.0)
    rig.runtime.localization_frame = reference_frame
    rig.bb["active_map_name"] = map_name
    rig.tick()

    seed = rig.trigger(tag_z=0.4)

    assert rig.behavior.base_frame == reference_frame
    assert seed.pose.pose.position.z == pytest.approx(expected_seed_height)
    # The same physical landmark yields the same map<-odom correction for
    # either pose convention, even though the published robot height differs.
    odom_to_reference = np.eye(4)
    odom_to_reference[2, 3] = rig.buffer.frame_heights[reference_frame]
    map_to_odom = pose_to_matrix(seed.pose.pose) @ np.linalg.inv(odom_to_reference)
    np.testing.assert_allclose(map_to_odom, np.eye(4), atol=1e-12)
    rig.publisher.publish.assert_called_once()


def test_runtime_pose_reference_mismatch_with_map_metadata_prevents_correction(rig):
    rig.repo.create_empty("feet_map")
    rig.repo.add_landmark("feet_map", landmark())
    rig.runtime.localization_frame = "base_link"
    rig.bb["active_map_name"] = "feet_map"

    for _ in range(8):
        rig.sample()

    rig.publisher.publish.assert_not_called()
    assert not rig.behavior.landmark_map
    assert "pose reference differs" in rig.behavior.feedback_message


def test_repeated_tag_stamp_never_supplies_distinct_observations(rig):
    rig.sample()
    for _ in range(20):
        rig.advance(0.01)
        rig.tick()
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._windows[7].sample_count == 1


def test_runtime_mode_transition_never_fails_the_command_tree(rig):
    rig.trigger()
    rig.runtime.get_running_mode = Mock(side_effect=RuntimeError("mode not yet assigned"))
    rig.tick()
    assert "Waiting for localization runtime" in rig.behavior.feedback_message
    assert rig.behavior.attempts == 0
    assert rig.behavior._context is None
    rig.publisher.publish.assert_called_once()


def test_published_seed_holds_dispatch_even_if_recovery_context_changes(rig):
    rig.trigger()
    deadline = rig.steady[0] + rig.behavior.config.settle_time_sec
    assert rig.bb["localization_reset_hold_until"] == deadline
    rig.behavior._reset_recovery(Time(seconds=rig.clock[0]).nanoseconds)
    assert rig.bb["localization_reset_hold_until"] == deadline
    rig.behavior.shutdown()
    assert rig.bb["localization_reset_hold_until"] == 0.0


def test_source_frame_switch_restarts_observation_window(rig):
    rig.sample()
    rig.sample()
    rig.sample(frame="camera")
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._windows[7].sample_count == 1
    rig.trigger(frame="camera")


def test_healthy_localization_with_landmark_agreement_does_not_reset(rig):
    for _ in range(12):
        rig.sample(variance=0.01, info=True)
    rig.publisher.publish.assert_not_called()
    assert "healthy" in rig.behavior.feedback_message.lower()


def test_retries_require_new_post_request_tag_captures_and_stop_at_limit(rig):
    rig.trigger()
    old_tag = deepcopy(rig.bb["visible_tags"])
    for _ in range(8):
        rig.advance()
        rig.bb["visible_tags"] = deepcopy(old_tag)
        rig.bb["last_pose_estimation"] = localization(rig.clock[0], variance=0.5)
        rig.tick()
    assert rig.publisher.publish.call_count == 1
    rig.trigger()
    assert rig.behavior.attempts == 2
    for _ in range(12):
        rig.sample()
    assert rig.publisher.publish.call_count == 2
    assert "failed after 2 attempts" in rig.behavior.feedback_message
    assert rig.node.get_logger.return_value.warning.call_count == 1


def test_only_matching_accepted_post_correction_estimates_confirm_recovery(rig):
    rig.trigger()
    # Rebuild the observation window using fresh images after the correction.
    for _ in range(3):
        rig.sample(variance=0.01, info=False)
    assert rig.behavior.attempts == 1
    assert rig.behavior._healthy_count == 0
    for expected in (1, 2, 3):
        rig.sample(variance=0.01, info=True)
        assert rig.behavior._healthy_count == expected
        for _ in range(5):
            rig.tick()
        assert rig.behavior._healthy_count == expected
        assert rig.behavior.attempts == (0 if expected == 3 else 1)
    assert "healthy" in rig.behavior.feedback_message.lower()


def test_interleaved_pose_then_matching_info_preserves_confirmation_progress(rig):
    rig.trigger()
    for _ in range(3):
        rig.sample(variance=0.01, info=True)
    assert rig.behavior._healthy_count == 1
    # Pose and Info use separate subscriptions; the old Info is still present.
    rig.sample(variance=0.01)
    assert rig.behavior._healthy_count == 1
    rig.bb["last_localization_info"] = registration(rig.clock[0])
    rig.tick()
    assert rig.behavior._healthy_count == 2
    rig.sample(variance=0.01)
    rig.bb["last_localization_info"] = registration(rig.clock[0])
    rig.tick()
    assert rig.behavior.attempts == 0


def test_missing_registration_evidence_cannot_postpone_retries_forever(rig):
    rig.trigger()
    for _ in range(16):
        rig.sample(variance=0.01)
    assert rig.publisher.publish.call_count == 2
    assert rig.behavior.attempts == 2
    assert "failed after 2 attempts" in rig.behavior.feedback_message


def test_rejected_registration_breaks_consecutive_confirmation(rig):
    rig.trigger()
    for _ in range(3):
        rig.sample(variance=0.01, info=True)
    assert rig.behavior._healthy_count == 1
    rig.sample(variance=0.01, info=False)
    assert rig.behavior._healthy_count == 0


def test_landmark_addition_and_deletion_refresh_same_active_map(rig):
    rig.repo.delete_landmark("map_a", "Tag_7")
    rig.sample()
    assert not rig.behavior.landmark_map
    rig.repo.add_landmark("map_a", landmark())
    rig.sample()
    assert 7 in rig.behavior.landmark_map
    rig.publisher.publish.assert_not_called()
    rig.trigger()
    rig.repo.delete_landmark("map_a", "Tag_7")
    for _ in range(8):
        rig.sample()
    assert not rig.behavior.landmark_map
    assert rig.publisher.publish.call_count == 1


def test_conflicting_visible_landmarks_withhold_recovery(rig):
    rig.repo.add_landmark("map_a", landmark(tag_id=8, x=3.0))
    rig.tick()
    for _ in range(8):
        rig.advance()
        rig.bb["visible_tags"] = {
            7: tag(rig.clock[0]), 8: tag(rig.clock[0], tag_id=8),
        }
        rig.bb["last_pose_estimation"] = localization(rig.clock[0], variance=0.5)
        rig.tick()
    rig.publisher.publish.assert_not_called()
    assert "Conflicting landmarks" in rig.behavior.feedback_message


def test_landmark_deleted_during_tick_is_not_used_for_correction(rig):
    rig.sample()
    rig.sample()
    original_lookup = rig.buffer.lookup_transform

    def removing_lookup(target, source, stamp, **kwargs):
        if stamp.nanoseconds == 0 and rig.repo.get_landmark("map_a", "Tag_7") is not None:
            rig.repo.delete_landmark("map_a", "Tag_7")
        return original_lookup(target, source, stamp, **kwargs)

    rig.buffer.lookup_transform = removing_lookup
    rig.sample()
    rig.publisher.publish.assert_not_called()
    assert "Landmarks changed" in rig.behavior.feedback_message


def test_ros_clock_pause_does_not_keep_tag_and_localization_producers_live(rig):
    rig.sample()
    rig.sample()
    for _ in range(8):
        rig.advance(0.25, ros=False)
        rig.tick()
    rig.publisher.publish.assert_not_called()
    assert not rig.behavior._windows


@pytest.mark.parametrize("fault", ["stale", "future", "zero", "wrong_frame", "nan", "far", "missing_tf", "wrong_tf_frame", "wrong_tf_time"])
def test_invalid_observation_or_capture_tf_never_requests_correction(rig, fault):
    for _ in range(8):
        rig.advance()
        rig.bb["last_pose_estimation"] = localization(rig.clock[0], variance=0.5)
        observation = tag(rig.clock[0])
        if fault in ("stale", "future", "zero"):
            when = {"stale": rig.clock[0] - 2.0, "future": rig.clock[0] + 0.1, "zero": 0.0}[fault]
            observation.pose.header.stamp = Time(seconds=when).to_msg()
        elif fault == "wrong_frame":
            observation.pose.header.frame_id = "map"
        elif fault == "nan":
            observation.pose.pose.position.x = float("nan")
        elif fault == "far":
            observation.pose.pose.position.x = 30.0
        elif fault == "missing_tf":
            rig.buffer.unavailable = True
        elif fault == "wrong_tf_frame":
            rig.buffer.invalid_frame = True
        else:
            rig.buffer.history_stamp_delta = -0.5
        rig.bb["visible_tags"] = {7: observation}
        rig.tick()
    rig.publisher.publish.assert_not_called()


@pytest.mark.parametrize("stamp_delta", [-1.0, 0.1])
def test_stale_or_future_current_odometry_withholds_correction(rig, stamp_delta):
    rig.buffer.latest_stamp_delta = stamp_delta
    for _ in range(8):
        rig.sample()
    rig.publisher.publish.assert_not_called()


def test_mapping_mode_never_seeds_localization(rig):
    rig.runtime.get_running_mode = lambda: "mapping"
    for _ in range(8):
        rig.sample()
    rig.publisher.publish.assert_not_called()
    assert rig.behavior.attempts == 0


@pytest.mark.parametrize("change", ["clock", "map", "process"])
def test_map_clock_or_runtime_change_discards_attempts_and_old_observations(rig, change):
    rig.trigger()
    if change == "clock":
        rig.clock[0] = 5.0
    elif change == "map":
        rig.repo.create_empty("map_b")
        rig.repo.add_landmark("map_b", landmark())
        rig.bb["active_map_name"] = "map_b"
    else:
        rig.runtime.process = object()
    rig.tick()
    assert rig.behavior.attempts == 0
    assert not rig.behavior._windows
    assert rig.publisher.publish.call_count == 1
    rig.trigger()
    assert rig.publisher.publish.call_count == 2


def test_stationary_gate_defers_correction_until_motion_stops(rig):
    for _ in range(10):
        rig.sample(move=0.1)
    rig.publisher.publish.assert_not_called()
    message = rig.trigger()
    # Stable map<-odom observations preserve the current robot pose after motion.
    assert message.pose.pose.position.x == pytest.approx(rig.buffer.position)


def test_running_command_defers_correction_while_monitoring_continues(rig):
    rig.bb["command_tree_status"] = py_trees.common.Status.RUNNING
    for _ in range(8):
        rig.sample()
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._windows[7].sample_count == rig.behavior.config.required_samples
    rig.bb["command_tree_status"] = py_trees.common.Status.FAILURE
    rig.trigger()
