"""Offline checks for the stricter registration evidence used by refinement."""

from types import SimpleNamespace

import pytest
from rtabmap_msgs.msg import Info

from fault_detector_spot.navigation.localization_quality import (
    accepted_lidar_localization,
    accepted_localization,
)


REJECTION = "Loop/RejectedHypothesis/"
GLOBAL_ICP = "Proximity/Space_detections_added_icp_global/"
MULTI_ICP = "Proximity/Space_detections_added_icp_multi/"


def lidar_info(global_count=1.0, multi_count=0.0):
    info = Info()
    info.proximity_detection_id = 456
    info.stats_keys = [REJECTION, GLOBAL_ICP, MULTI_ICP]
    info.stats_values = [0.0, global_count, multi_count]
    return info


@pytest.mark.parametrize("global_count,multi_count", [
    (1.0, 0.0),
    (0.0, 1.0),
    (2.0, 3.0),
])
def test_accepts_both_supported_map_icp_paths(global_count, multi_count):
    assert accepted_lidar_localization(lidar_info(global_count, multi_count))


def test_visual_map_match_remains_valid_for_existing_localization_check():
    info = lidar_info(0.0, 0.0)
    info.loop_closure_id = 123
    assert accepted_localization(info)
    assert not accepted_lidar_localization(info)


def test_scan_to_scan_odometry_refinement_is_not_map_registration():
    info = lidar_info(0.0, 0.0)
    info.stats_keys.append("NeighborLinkRefining/ICP_inliers_ratio/")
    info.stats_values.append(1.0)
    assert not accepted_lidar_localization(info)


def test_positive_icp_count_requires_proximity_id_even_with_a_loop_closure():
    info = lidar_info()
    info.proximity_detection_id = 0
    info.loop_closure_id = 123
    assert accepted_localization(info)
    assert not accepted_lidar_localization(info)


@pytest.mark.parametrize("proximity_id", [0, 456])
def test_later_rejection_invalidates_positive_icp_counts(proximity_id):
    # RTAB-Map does not undo its counters when graph optimization rejects links.
    info = lidar_info()
    info.proximity_detection_id = proximity_id
    info.stats_values[0] = 1.0
    assert not accepted_lidar_localization(info)


@pytest.mark.parametrize("key", [GLOBAL_ICP, MULTI_ICP])
@pytest.mark.parametrize("malformation", ["missing", "duplicate", "wrong_key"])
def test_requires_each_exact_counter_once(key, malformation):
    info = lidar_info(1.0, 1.0)
    keys = list(info.stats_keys)
    values = list(info.stats_values)
    index = keys.index(key)
    if malformation == "missing":
        keys.pop(index)
        values.pop(index)
    elif malformation == "duplicate":
        keys.append(key)
        values.append(1.0)
    else:
        keys[index] = key.rstrip("/")
    info.stats_keys = keys
    info.stats_values = values
    assert not accepted_lidar_localization(info)


@pytest.mark.parametrize("counter_index", [1, 2])
@pytest.mark.parametrize("value", [
    -1.0, float("nan"), float("inf"), float("-inf"),
    None, "invalid", "1.0", True,
])
def test_invalid_counter_cannot_be_masked_by_other_positive_counter(
    counter_index, value,
):
    info = lidar_info(1.0, 1.0)
    values = list(info.stats_values)
    values[counter_index] = value
    malformed = SimpleNamespace(
        loop_closure_id=info.loop_closure_id,
        proximity_detection_id=info.proximity_detection_id,
        stats_keys=list(info.stats_keys),
        stats_values=values,
    )
    assert not accepted_lidar_localization(malformed)


@pytest.mark.parametrize("malformation", [
    "no_message", "missing_attributes", "mismatched_values", "missing_rejection",
])
def test_requires_existing_localization_evidence_contract(malformation):
    info = lidar_info()
    if malformation == "no_message":
        info = None
    elif malformation == "missing_attributes":
        info = SimpleNamespace()
    elif malformation == "mismatched_values":
        info.stats_values = [0.0, 1.0]
    else:
        info.stats_keys = [GLOBAL_ICP, MULTI_ICP]
        info.stats_values = [1.0, 0.0]
    assert not accepted_lidar_localization(info)
