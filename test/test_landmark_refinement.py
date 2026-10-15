"""Conservative landmark refinement keeps its manually saved reference bounded."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.navigation.landmark_refinement import (
    LandmarkRefinementConfig, LandmarkRefiner,
)
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler, rotation_distance_rad,
)


def pose(x=0.0, y=0.0, z=0.0, *, yaw=0.0):
    result = PoseData.identity()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation = quaternion_from_euler("z", math.radians(yaw))
    return result


def burst(refiner, observed=None, current=None, original=None, *, tag_id=7, start=100.0):
    observed = observed if observed is not None else pose(0.05)
    current = current if current is not None else pose()
    original = original if original is not None else pose()
    proposal = None
    for index in range(refiner.config.required_samples):
        now = start + index * refiner.config.sample_interval_sec
        stamp = int(now * 1e9)
        refiner.observe_visibility({tag_id}, now)
        proposal = refiner.observe(tag_id, stamp, stamp, observed, current, original, now)
    return proposal


def test_refinement_uses_mean_of_distinct_stable_captures_and_preserves_inputs():
    refiner = LandmarkRefiner()
    current, original = pose(z=1.0), pose(z=1.0)
    original_copy = deepcopy(original)
    for index, x in enumerate((0.03, 0.04, 0.05, 0.04, 0.05)):
        now = 100.0 + index * 0.5
        refiner.observe_visibility({7}, now)
        result = refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(x, z=1.02), current, original, now)
        if index < 4:
            assert result is None

    assert result.position.x == pytest.approx(0.0042)
    assert result.position.z == pytest.approx(1.002)
    assert original == original_copy
    assert current == pose(z=1.0)
    result.position.x = 50.0
    assert refiner._tags[7].proposal.position.x == pytest.approx(0.0042)


def test_camera_and_registration_must_both_advance_for_each_sample():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    assert refiner.observe(7, 10, 20, pose(0.05), pose(), pose(), 100.0) is None
    for camera, registration in ((10, 21), (11, 20), (10, 20)):
        assert refiner.observe(7, camera, registration, pose(0.05), pose(), pose(), 100.0) is None
    assert refiner._tags[7].window.sample_count == 1
    for index in range(1, 5):
        proposal = refiner.observe(7, 10 + index, 20 + index, pose(0.05), pose(), pose(), 100.0 + index * 0.5)
    assert proposal is not None


def test_slow_distinct_registrations_fit_within_the_bounded_attempt():
    refiner = LandmarkRefiner()
    for index in range(5):
        now = 100.0 + index * 5.0
        stamp = int(now * 1e9)
        refiner.observe_visibility({7}, now)
        proposal = refiner.observe(7, stamp, stamp, pose(0.05), pose(), pose(), now)
        if index < 4:
            assert proposal is None
    assert proposal.position.x == pytest.approx(0.005)


def test_each_adjustment_obeys_full_translation_and_rotation_step_caps():
    refiner = LandmarkRefiner()
    component = 0.14 / math.sqrt(3)
    result = burst(refiner, pose(component, component, component, yaw=8.0))
    assert math.sqrt(sum(value ** 2 for value in result.position.to_dict().values())) == pytest.approx(0.01)
    assert result.position.x == pytest.approx(0.01 / math.sqrt(3))
    assert math.degrees(rotation_distance_rad(pose().orientation, result.orientation)) == pytest.approx(0.5)


def test_rotation_blend_takes_short_so3_path_across_wrap_and_antipodal_quaternions():
    refiner = LandmarkRefiner()
    current = original = pose(yaw=179.0)
    observed = pose(yaw=-179.0)
    for index in range(5):
        observation = deepcopy(observed)
        if index % 2:
            for axis in ("x", "y", "z", "w"):
                setattr(observation.orientation, axis, -getattr(observation.orientation, axis))
        now = 100.0 + index * 0.5
        refiner.observe_visibility({7}, now)
        result = refiner.observe(7, int(now * 1e9), int(now * 1e9), observation, current, original, now)
    assert math.degrees(rotation_distance_rad(current.orientation, result.orientation)) == pytest.approx(0.2)
    assert math.degrees(rotation_distance_rad(result.orientation, observed.orientation)) == pytest.approx(1.8)


@pytest.mark.parametrize("observed", [pose(0.151), pose(yaw=10.1)])
def test_large_discrepancies_require_explicit_resaving(observed):
    refiner = LandmarkRefiner()
    assert burst(refiner, observed) is None
    assert "resaving" in refiner.detail
    assert refiner._tags[7].window.sample_count == 0


@pytest.mark.parametrize("current,observed", [
    (pose(0.045), pose(0.12)),
    (pose(yaw=2.8), pose(yaw=8.0)),
])
def test_original_anchor_bounds_cannot_be_evaded_by_small_individual_updates(current, observed):
    refiner = LandmarkRefiner()
    original = pose()
    assert burst(refiner, observed, current, original) is None
    assert "original-anchor limit" in refiner.detail
    assert original == pose()


def test_standing_in_view_cannot_refine_repeatedly_even_after_cooldown():
    refiner = LandmarkRefiner()
    result = burst(refiner)
    refiner.commit(7, 102.0)
    assert burst(refiner, current=result, start=500.0) is None
    assert "encounter" in refiner.detail
    refiner.reset_observations()
    assert burst(refiner, current=result, start=1000.0) is None
    assert "encounter" in refiner.detail


def test_brief_occlusion_does_not_rearm_but_sustained_absence_does():
    refiner = LandmarkRefiner()
    result = burst(refiner)
    refiner.commit(7, 102.0)
    refiner.observe_visibility(set(), 500.0)
    assert burst(refiner, current=result, start=504.0) is None
    refiner.observe_visibility(set(), 510.0)
    assert burst(refiner, current=result, start=515.0) is not None


def test_new_encounter_must_still_wait_for_minimum_interval():
    refiner = LandmarkRefiner()
    result = burst(refiner)
    refiner.commit(7, 102.0)
    refiner.observe_visibility(set(), 103.0)
    assert burst(refiner, current=result, start=108.0) is None
    assert "interval" in refiner.detail
    assert burst(refiner, current=result, start=402.0) is not None


def test_failed_persistence_does_not_consume_encounter_until_commit():
    refiner = LandmarkRefiner()
    assert burst(refiner) is not None
    # The caller failed to persist, so subsequent independent evidence may retry.
    assert burst(refiner, start=103.0) is not None
    refiner.commit(7, 105.0)
    assert burst(refiner, start=106.0) is None


def test_lost_quality_discards_partial_burst_and_does_not_recount_old_capture():
    refiner = LandmarkRefiner()
    for index in range(4):
        now = 100.0 + index * 0.5
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    refiner.reset_observations()
    assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 0
    assert burst(refiner, start=102.0) is not None


def test_anchor_change_during_burst_requires_new_stable_observations():
    refiner = LandmarkRefiner()
    for index in range(4):
        now = 100.0 + index * 0.5
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    now = 102.0
    refiner.observe_visibility({7}, now)
    assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(0.001), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 1


def test_unstable_or_expired_observations_never_supply_a_complete_burst():
    refiner = LandmarkRefiner()
    for index in range(10):
        now = 100.0 + index * 0.5
        refiner.observe_visibility({7}, now)
        x = 0.01 if index % 2 else 0.10
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(x), pose(), pose(), now) is None
    assert "unstable" in refiner.detail
    for index in range(5):
        now = 400.0 + index * 300.0
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 1


def test_absence_and_disabled_policy_do_not_generate_proposals():
    refiner = LandmarkRefiner()
    for index in range(5):
        assert refiner.observe(7, index + 1, index + 1, pose(0.05), pose(), pose(), 100.0) is None
    disabled = LandmarkRefiner(replace(LandmarkRefinementConfig(), enabled=False))
    assert burst(disabled) is None


def test_early_reservation_is_consumed_by_observe_without_scheduling_twice():
    refiner = LandmarkRefiner()
    for index in range(5):
        now = 100.0 + index * 0.5
        stamp = int(now * 1e9)
        refiner.observe_visibility({7}, now)
        assert refiner.begin_samples({7: (stamp, stamp)}, now) == {7}
        result = refiner.observe(7, stamp, stamp, pose(0.05), pose(), pose(), now)
        assert refiner._tags[7].window.sample_count == index + 1
    assert result is not None


def test_failed_downstream_quality_or_tf_does_not_repeat_stamps_or_sampling_work():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    assert refiner.begin_samples({7: (10, 20)}, 100.0) == {7}
    # No observe call: the caller could not validate quality or obtain TF.
    refiner.reset_observations()
    assert refiner.begin_samples({7: (11, 21)}, 100.49) == set()
    for pair in ((10, 20), (11, 20), (10, 21)):
        assert refiner.begin_samples({7: pair}, 100.5) == set()
    assert refiner.begin_samples({7: (11, 21)}, 100.5) == {7}
    assert refiner._tags[7].window.sample_count == 0


def test_one_global_thirty_second_attempt_samples_at_most_twice_a_second():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    scheduled = []
    for tick in range(601):
        now = 100.0 + tick * 0.05
        if refiner.begin_samples({7: (tick + 1, tick + 1)}, now):
            scheduled.append(now)
        # Repeated failed checks cannot extend the attempt or its budget.
        refiner.reset_observations()
    assert scheduled == pytest.approx([100.0 + index * 0.5 for index in range(60)])
    assert refiner.begin_samples({7: (1000, 1000)}, 399.99) == set()
    assert refiner.begin_samples({7: (1000, 1000)}, 400.0) == {7}


def test_only_one_landmark_uses_an_attempt_and_failed_tags_do_not_starve_others():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7, 8}, 100.0)
    assert refiner.begin_samples({7: (1, 1), 8: (1, 1)}, 100.0) == {7}
    assert refiner.begin_samples({8: (2, 2)}, 100.5) == set()
    assert refiner.begin_samples({7: (3, 3), 8: (3, 3)}, 130.0) == set()
    assert refiner.begin_samples({7: (4, 4), 8: (4, 4)}, 400.0) == {8}


def test_attempt_timeout_discards_uncommitted_proposal_and_preserves_retry_interval():
    refiner = LandmarkRefiner()
    assert burst(refiner) is not None
    assert refiner.begin_samples({7: (130_000_000_000, 130_000_000_000)}, 130.0) == set()
    assert refiner._tags[7].window.sample_count == 0
    with pytest.raises(ValueError, match="No landmark refinement proposal"):
        refiner.commit(7, 130.0)
    assert burst(refiner, start=400.0) is not None


def test_completed_agreement_consumes_encounter_without_persisting_an_update():
    refiner = LandmarkRefiner()
    assert burst(refiner, observed=pose()) is None
    assert "already agrees" in refiner.detail
    # The cheap gate prevents a second covariance/TF/pose calculation.
    assert refiner.begin_samples({7: (103_000_000_000, 103_000_000_000)}, 103.0) == set()
    assert "encounter" in refiner.detail
    assert refiner._tags[7].last_commit_at is None
    assert burst(refiner, start=400.0) is None
    assert "encounter" in refiner.detail
    refiner.observe_visibility(set(), 500.0)
    assert burst(refiner, start=506.0) is not None


def test_stationarity_checks_continue_between_distinct_registration_samples():
    refiner = LandmarkRefiner()
    assert not refiner.stationarity_sample_due(99.0)
    refiner.observe_visibility({7}, 100.0)
    assert refiner.begin_samples({7: (10, 20)}, 100.0) == {7}
    assert refiner.attempt_tag_id == 7
    assert refiner.stationarity_sample_due(100.0)
    assert not refiner.stationarity_sample_due(100.0)
    for index in range(1, 4):
        now = 100.0 + index * 0.5
        assert refiner.begin_samples({7: (10 + index, 20)}, now) == set()
        assert refiner.stationarity_sample_due(now)
    # Odometry slots neither consume nor require a new registration stamp.
    assert refiner._tags[7].last_registration_stamp_ns == 20
    assert refiner.begin_samples({7: (15, 21)}, 102.0) == {7}
    assert refiner.stationarity_sample_due(102.0)


def test_new_registration_waits_for_next_odometry_slot_without_consuming_stamps():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    assert refiner.begin_samples({7: (10, 20)}, 100.0) == {7}
    assert refiner.stationarity_sample_due(100.0)
    assert refiner.stationarity_sample_due(100.5)
    assert refiner.begin_samples({7: (11, 21)}, 100.75) == set()
    assert refiner._tags[7].last_registration_stamp_ns == 20
    assert not refiner.stationarity_sample_due(100.75)
    assert refiner.begin_samples({7: (11, 21)}, 101.0) == {7}
    assert refiner.stationarity_sample_due(101.0)
    assert refiner._tags[7].last_registration_stamp_ns == 21


def test_stationarity_checks_keep_one_bounded_budget_despite_observation_resets():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    refiner.begin_samples({7: (1, 1)}, 100.0)
    scheduled = []
    for tick in range(601):
        now = 100.0 + tick * 0.05
        if refiner.stationarity_sample_due(now):
            scheduled.append(now)
        refiner.reset_observations()
    assert scheduled == pytest.approx([100.0 + index * 0.5 for index in range(60)])
    assert refiner.attempt_tag_id is None
    assert not refiner.stationarity_sample_due(399.99)
    assert refiner.begin_samples({7: (2, 2)}, 399.99) == set()
    assert refiner.begin_samples({7: (2, 2)}, 400.0) == {7}
    assert refiner.stationarity_sample_due(400.0)


def test_stationarity_check_requires_the_active_tag_and_absence_cannot_extend_attempt():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7, 8}, 100.0)
    refiner.begin_samples({7: (1, 1), 8: (1, 1)}, 100.0)
    assert refiner.stationarity_sample_due(100.0)
    refiner.observe_visibility({8}, 101.0)
    assert not refiner.stationarity_sample_due(101.0)
    refiner.observe_visibility({7, 8}, 129.0)
    assert refiner.stationarity_sample_due(129.0)
    assert not refiner.stationarity_sample_due(130.0)
    assert refiner.attempt_tag_id is None


def test_stationarity_slots_end_on_commit_and_context_reset():
    refiner = LandmarkRefiner()
    assert burst(refiner) is not None
    assert refiner.stationarity_sample_due(102.0)
    refiner.commit(7, 102.0)
    assert not refiner.stationarity_sample_due(102.5)
    refiner.reset_context()
    assert not refiner.stationarity_sample_due(103.0)
    refiner.observe_visibility({7}, 103.0)
    assert refiner.begin_samples({7: (1, 1)}, 103.0) == {7}
    assert refiner.stationarity_sample_due(103.0)


def test_stationarity_clock_regression_cannot_rearm_sampling_or_attempt():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    refiner.begin_samples({7: (1, 1)}, 100.0)
    assert refiner.stationarity_sample_due(100.0)
    assert not refiner.stationarity_sample_due(99.0)
    assert not refiner.stationarity_sample_due(100.0)
    assert refiner.stationarity_sample_due(100.5)
    assert not refiner.stationarity_sample_due(130.0)
    assert refiner.begin_samples({7: (2, 2)}, 130.0) == set()


def test_observation_reset_and_clock_regression_cannot_rearm_attempt_cooldown():
    refiner = LandmarkRefiner()
    refiner.observe_visibility({7}, 100.0)
    assert refiner.begin_samples({7: (1, 1)}, 100.0) == {7}
    refiner.reset_observations()
    assert refiner.begin_samples({7: (2, 2)}, 99.0) == set()
    assert refiner.begin_samples({7: (2, 2)}, 130.0) == set()
    refiner.reset_context()
    refiner.observe_visibility({7}, 130.0)
    assert refiner.begin_samples({7: (2, 2)}, 130.0) == {7}


def test_config_is_read_from_its_own_parameter_namespace(monkeypatch):
    from fault_detector_spot.navigation import landmark_refinement as module
    parameters = {"navigation.landmark_refinement.enabled": False}
    node = Mock()
    node.has_parameter.side_effect = lambda name: name in parameters
    node.declare_parameter.side_effect = lambda name, value: parameters.setdefault(name, value)
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=parameters[name])
    monkeypatch.setattr(module, "parameter_override", lambda _name, value: value)
    config = LandmarkRefinementConfig.from_node(node)
    assert config.enabled is False
    assert config.maximum_sample_span_sec == 30.0
    assert config.maximum_localization_position_std_m == 0.10
    assert config.maximum_localization_yaw_std_rad == pytest.approx(math.radians(3.0))
    assert config.minimum_interval_sec == 300.0
    assert config.attempt_interval_sec == 300.0
    assert config.sample_interval_sec == 0.5
    assert all(name.startswith("navigation.landmark_refinement.") for name in parameters)


@pytest.mark.parametrize("changes", [
    {"gain": 0.0}, {"gain": 1.1}, {"required_samples": True},
    {"required_samples": 1}, {"minimum_interval_sec": float("inf")},
    {"maximum_step_position_m": 0.06}, {"enabled": 1},
    {"attempt_interval_sec": 0.0}, {"sample_interval_sec": float("nan")},
])
def test_invalid_refinement_config_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(LandmarkRefinementConfig(), **changes)
