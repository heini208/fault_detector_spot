"""Offline checks for trustworthy localization and RTAB-Map evidence."""

import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info

from fault_detector_spot.navigation import localization_quality
from fault_detector_spot.navigation.localization_quality import (
    LocalizationQualityConfig,
    accepted_localization,
    assess_localization_quality,
    lidar_localization_result,
    localization_quality_error,
)


def localization_pose():
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp.sec = 10
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.01
    message.pose.covariance[7] = 0.01
    message.pose.covariance[35] = math.radians(3.0) ** 2
    return message


def quality_error(message, now_ns=10_000_000_000):
    return localization_quality_error(message, now_ns, LocalizationQualityConfig())


def test_valid_fresh_pose_with_planar_covariance_is_healthy():
    message = localization_pose()
    message.pose.covariance[1] = 0.004
    message.pose.covariance[6] = 0.004

    assert quality_error(message) == ""
    assert quality_error(message, now_ns=11_500_000_000) == ""
    assert "stale" in quality_error(message, now_ns=11_500_000_001)


@pytest.mark.parametrize("stamp,reason", [(0, "invalid"), (11, "future"), (8, "stale")])
def test_unknown_future_and_old_timestamps_are_unusable(stamp, reason):
    message = localization_pose()
    message.header.stamp.sec = stamp
    assert reason in quality_error(message)


def test_map_frame_and_finite_valid_pose_are_required():
    message = localization_pose()
    message.header.frame_id = "odom"
    assert "map frame" in quality_error(message)
    message.header.frame_id = "map"
    message.pose.pose.position.x = float("nan")
    assert quality_error(message)
    message.pose.pose.position.x = 0.0
    message.pose.pose.orientation.w = 0.0
    assert quality_error(message)
    assert quality_error(None)
    assert quality_error(SimpleNamespace())


@pytest.mark.parametrize("value", [0.0, -0.01, 9999.0, float("inf"), float("nan")])
@pytest.mark.parametrize("index", [0, 7, 35])
def test_unknown_and_invalid_planar_variances_are_unusable(index, value):
    message = localization_pose()
    message.pose.covariance[index] = value
    assert quality_error(message)


def test_covariance_must_be_symmetric_and_positive_semidefinite():
    message = localization_pose()
    message.pose.covariance[1] = 0.02
    assert "symmetric" in quality_error(message)
    message.pose.covariance[6] = 0.02
    assert "positive semidefinite" in quality_error(message)


def test_position_uncertainty_uses_major_axis_including_cross_covariance():
    message = localization_pose()
    message.pose.covariance[0] = 0.08
    message.pose.covariance[7] = 0.08
    assert quality_error(message) == ""
    message.pose.covariance[1] = 0.04
    message.pose.covariance[6] = 0.04
    assert "position uncertainty" in quality_error(message)


def test_planar_psd_check_includes_position_yaw_cross_terms():
    message = localization_pose()
    message.pose.covariance[5] = 0.1
    message.pose.covariance[30] = 0.1
    assert "positive semidefinite" in quality_error(message)


def test_yaw_uncertainty_is_checked_separately():
    message = localization_pose()
    message.pose.covariance[35] = math.radians(11.0) ** 2
    assert "yaw uncertainty" in quality_error(message)


def test_only_measured_uncertainty_or_exact_not_localized_report_qualifies_for_recovery():
    def assess(message):
        return assess_localization_quality(message, 10_000_000_000, LocalizationQualityConfig())

    message = localization_pose()
    assert assess(message).healthy
    assert not assess(message).not_localized
    assert not assess(None).uncertain
    assert not assess(None).not_localized
    message.pose.covariance[0] = 0.5
    assert assess(message).uncertain
    assert not assess(message).not_localized
    message.pose.covariance[0] = 9999.0
    assert not assess(message).uncertain  # A mixed sentinel is not a valid report.
    assert not assess(message).not_localized
    for index in (0, 7, 35):
        message.pose.covariance[index] = 9999.0
    assert assess(message).uncertain
    assert assess(message).not_localized
    assert not assess(message).healthy
    assert quality_error(message)  # It still cannot qualify for saving/refining.
    message.pose.covariance[1] = 0.01
    assert not assess(message).uncertain
    assert not assess(message).not_localized


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf")])
@pytest.mark.parametrize("field", ["maximum_age_sec", "maximum_position_std_m", "maximum_yaw_std_rad"])
def test_quality_config_rejects_non_positive_and_non_finite(field, value):
    with pytest.raises(ValueError, match="positive and finite"):
        LocalizationQualityConfig(**{field: value})


def test_quality_config_respects_ros_parameters_and_saved_overrides(monkeypatch):
    values = {"navigation.localization_quality.maximum_age_sec": 2.0}
    node = SimpleNamespace(
        has_parameter=lambda key: key in values,
        declare_parameter=lambda key, default: values.setdefault(key, default),
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
    )
    monkeypatch.setattr(
        localization_quality,
        "parameter_override",
        lambda key, default: 0.2 if key.endswith("maximum_position_std_m") else default,
    )
    config = LocalizationQualityConfig.from_node(node)

    assert config.maximum_age_sec == 2.0
    assert config.maximum_position_std_m == 0.2
    assert config.maximum_yaw_std_rad == pytest.approx(math.radians(10.0))


def registration_info():
    info = Info()
    info.loop_closure_id = 123
    info.stats_keys = ["Loop/RejectedHypothesis/"]
    info.stats_values = [0.0]
    return info


def test_registration_evidence_requires_map_match_and_unrejected_hypothesis():
    info = registration_info()
    assert accepted_localization(info)
    info.loop_closure_id = 0
    assert not accepted_localization(info)
    info.proximity_detection_id = 456
    assert accepted_localization(info)
    info.stats_values = [1.0]
    assert not accepted_localization(info)


@pytest.mark.parametrize("keys,values", [
    ([], []),
    (["Loop/RejectedHypothesis"], [0.0]),
    (["Loop/RejectedHypothesis/"], []),
    (["Loop/RejectedHypothesis/"], [float("nan")]),
    (["Loop/RejectedHypothesis/"], [-1.0]),
    (["Loop/RejectedHypothesis/", "Loop/RejectedHypothesis/"], [0.0, 1.0]),
])
def test_missing_or_malformed_rejection_evidence_is_not_accepted(keys, values):
    info = registration_info()
    info.stats_keys = keys
    info.stats_values = values
    assert not accepted_localization(info)


def test_missing_or_invalid_registration_identifiers_are_not_accepted():
    assert not accepted_localization(None)
    assert not accepted_localization(SimpleNamespace())
    info = registration_info()
    info.loop_closure_id = -1
    assert not accepted_localization(info)


def lidar_result(*, loop_id=0, proximity_id=0, rejection=0.0, global_icp=0.0, multi_icp=0.0):
    return SimpleNamespace(
        loop_closure_id=loop_id,
        proximity_detection_id=proximity_id,
        stats_keys=[
            "Loop/RejectedHypothesis/",
            "Proximity/Space_detections_added_icp_global/",
            "Proximity/Space_detections_added_icp_multi/",
        ],
        stats_values=[rejection, global_icp, multi_icp],
    )


@pytest.mark.parametrize("counters", [{"global_icp": 1.0}, {"multi_icp": 1.0}])
def test_lidar_result_distinguishes_accepted_rejected_and_missing_registrations(counters):
    assert lidar_localization_result(lidar_result(proximity_id=42, **counters)) is True
    assert lidar_localization_result(lidar_result(proximity_id=42, rejection=1.0, **counters)) is False
    assert lidar_localization_result(lidar_result()) is False
    assert lidar_localization_result(lidar_result(rejection=1.0)) is False


@pytest.mark.parametrize("identifiers", [{"loop_id": 42}, {"proximity_id": 42}])
def test_accepted_registration_without_lidar_evidence_is_not_a_failed_result(identifiers):
    assert lidar_localization_result(lidar_result(**identifiers)) is None


@pytest.mark.parametrize("missing_index", [0, 1, 2])
def test_missing_or_duplicated_lidar_result_statistics_are_not_failure(missing_index):
    info = lidar_result()
    del info.stats_keys[missing_index]
    del info.stats_values[missing_index]
    assert lidar_localization_result(info) is None
    info = lidar_result()
    info.stats_keys.append(info.stats_keys[missing_index])
    info.stats_values.append(0.0)
    assert lidar_localization_result(info) is None


@pytest.mark.parametrize("index", [0, 1, 2])
@pytest.mark.parametrize("value", [-1.0, float("nan"), float("inf"), True, None])
def test_invalid_lidar_result_statistics_are_not_failure(index, value):
    info = lidar_result()
    info.stats_values[index] = value
    assert lidar_localization_result(info) is None


@pytest.mark.parametrize("field", ["loop_closure_id", "proximity_detection_id"])
@pytest.mark.parametrize("value", [-1, 1.5, True, None])
def test_invalid_lidar_result_identifiers_are_not_failure(field, value):
    info = lidar_result()
    setattr(info, field, value)
    assert lidar_localization_result(info) is None


def test_missing_or_incomplete_lidar_result_is_not_failure():
    assert lidar_localization_result(None) is None
    assert lidar_localization_result(SimpleNamespace()) is None
    info = lidar_result()
    info.stats_values.pop()
    assert lidar_localization_result(info) is None
    assert lidar_localization_result(lidar_result(rejection=2.0)) is None
