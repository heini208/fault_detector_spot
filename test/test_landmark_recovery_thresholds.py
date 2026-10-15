"""Conservative recovery gates with real covariance and matched RTAB-Map reports."""

import math

import pytest
from rclpy.time import Time

from fault_detector_spot.navigation.behaviours.landmark_relocalizer import LandmarkRecoveryConfig
from test_landmark_recovery import localization, registration, rig, tag  # noqa: F401


def lidar_result(stamp, outcome="failed"):
    message = registration(stamp, accepted=outcome in ("accepted", "non_lidar"))
    if outcome == "accepted":
        message.proximity_detection_id = 42
    message.stats_keys.extend([
        "Proximity/Space_detections_added_icp_global/",
        "Proximity/Space_detections_added_icp_multi/",
    ])
    message.stats_values.extend([1.0 if outcome == "accepted" else 0.0, 0.0])
    if outcome == "missing":
        return None
    if outcome in ("old", "future"):
        message.header.stamp = Time(seconds=stamp + (-1.0 if outcome == "old" else 1.0)).to_msg()
    elif outcome == "wrong_frame":
        message.header.frame_id = "odom"
    elif outcome == "malformed":
        message.stats_values[-1] = float("nan")
    return message


def tick_camera(rig, seconds):
    # Camera/odometry continue while RTAB-Map processes a slower lidar stream.
    while seconds > 1e-8:
        step = min(seconds, 0.25)
        rig.advance(step)
        rig.bb["visible_tags"] = {7: tag(rig.clock[0])}
        rig.tick()
        seconds -= step


def finite_sample(rig, *, position_std=0.8, yaw_deg=3.0, outcome="failed",
                  seconds=2.0, delay=0.0, interleaved=False):
    tick_camera(rig, seconds)
    stamp = rig.clock[0] - delay
    pose = localization(stamp, variance=position_std ** 2)
    pose.pose.covariance[35] = math.radians(yaw_deg) ** 2
    rig.bb["last_pose_estimation"] = pose
    if interleaved:
        before = rig.publisher.publish.call_count
        rig.tick()
        assert rig.publisher.publish.call_count == before
        tick_camera(rig, 0.25)
    rig.bb["last_localization_info"] = lidar_result(stamp, outcome)
    rig.tick()


@pytest.mark.parametrize("position_std,yaw_deg", [(0.4, 3.0), (0.1, 15.0), (0.75, 25.0)])
def test_moderate_uncertainty_never_seeds_or_relaxes_save_quality(rig, position_std, yaw_deg):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(8):
        finite_sample(rig, position_std=position_std, yaw_deg=yaw_deg)
    rig.publisher.publish.assert_not_called()
    assert "below landmark recovery thresholds" in rig.behavior.feedback_message
    assert rig.behavior._quality_error(
        rig.bb["last_pose_estimation"], Time(seconds=rig.clock[0]).nanoseconds, rig.steady[0],
    )  # Saving/confirmation still reject this uncertainty.
    assert rig.behavior.quality_config.maximum_position_std_m == 0.30
    assert rig.behavior.quality_config.maximum_yaw_std_rad == math.radians(10.0)


@pytest.mark.parametrize("position_std,yaw_deg", [(0.751, 3.0), (0.1, 25.1)])
@pytest.mark.parametrize("delay", [0.0, 3.2])
def test_finite_uncertainty_requires_ten_seconds_of_unsuccessful_results(rig, position_std, yaw_deg, delay):
    rig.behavior.config = LandmarkRecoveryConfig()
    rig.advance(delay)
    for _ in range(5):
        finite_sample(rig, position_std=position_std, yaw_deg=yaw_deg, delay=delay)
        rig.publisher.publish.assert_not_called()
    finite_sample(rig, position_std=position_std, yaw_deg=yaw_deg, delay=delay)
    rig.publisher.publish.assert_called_once()
    assert rig.behavior.attempts == 1


def test_successful_lidar_matching_with_high_covariance_prevents_resets(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(8):
        finite_sample(rig, outcome="accepted")
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._uncertain_count == 0
    assert "Lidar map matching succeeded" in rig.behavior.feedback_message


def test_successful_lidar_result_restarts_the_full_grace_period(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(5):
        finite_sample(rig)
    finite_sample(rig, outcome="accepted")
    assert rig.behavior._uncertain_count == 0
    for _ in range(5):
        finite_sample(rig)
        rig.publisher.publish.assert_not_called()
    finite_sample(rig)
    rig.publisher.publish.assert_called_once()


@pytest.mark.parametrize("outcome", ["missing", "old", "future", "wrong_frame", "malformed", "non_lidar"])
def test_unavailable_or_inconclusive_registration_cannot_count_as_lidar_failure(rig, outcome):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(2):
        finite_sample(rig)
    for _ in range(8):
        finite_sample(rig, outcome=outcome)
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._uncertain_count == 0


def test_interleaved_pose_and_info_can_complete_the_finite_uncertainty_run(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(5):
        finite_sample(rig, interleaved=True)
        rig.publisher.publish.assert_not_called()
    finite_sample(rig, interleaved=True)
    rig.publisher.publish.assert_called_once()


def test_replayed_finite_results_cannot_finish_the_grace_period(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(3):
        finite_sample(rig)
    tick_camera(rig, 12.0)
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._uncertain_count == 0


@pytest.mark.parametrize("gap_clock", ["both", "ros", "steady"])
def test_finite_uncertainty_streak_expires_across_a_long_result_gap(rig, gap_clock):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(3):
        finite_sample(rig)
    if gap_clock == "ros":
        rig.clock[0] += 5.1
    else:
        rig.advance(5.1, ros=gap_clock == "both")
    finite_sample(rig)
    rig.publisher.publish.assert_not_called()
    assert rig.behavior._uncertain_count == 1


def test_switch_to_explicit_not_localized_requires_its_own_fast_debounce(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(5):
        finite_sample(rig)
    for expected in (1, 2):
        rig.sample(seconds=1.0, variance=9999.0)
        assert rig.behavior._uncertain_count == expected
        rig.publisher.publish.assert_not_called()
    rig.sample(seconds=1.0, variance=9999.0)
    rig.publisher.publish.assert_called_once()


def test_switch_from_not_localized_requires_the_full_finite_grace_period(rig):
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(2):
        rig.sample(seconds=0.75, variance=9999.0)
    for _ in range(5):
        finite_sample(rig)
        rig.publisher.publish.assert_not_called()
    finite_sample(rig)
    rig.publisher.publish.assert_called_once()


@pytest.mark.parametrize("position_std,outcome", [(0.4, "failed"), (0.8, "accepted")])
def test_moderate_uncertainty_or_successful_lidar_withholds_retries_without_confirming(rig, position_std, outcome):
    rig.trigger(variance=9999.0)
    rig.behavior.config = LandmarkRecoveryConfig()
    for _ in range(10):
        finite_sample(rig, position_std=position_std, outcome=outcome)
    rig.publisher.publish.assert_called_once()
    assert rig.behavior.attempts == 1
    assert rig.behavior._healthy_count == 0
