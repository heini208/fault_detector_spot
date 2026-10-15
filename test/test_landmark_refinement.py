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
        now = start + index * 0.25
        stamp = int(now * 1e9)
        refiner.observe_visibility({tag_id}, now)
        proposal = refiner.observe(tag_id, stamp, stamp, observed, current, original, now)
    return proposal


def test_refinement_uses_mean_of_distinct_stable_captures_and_preserves_inputs():
    refiner = LandmarkRefiner()
    current, original = pose(z=1.0), pose(z=1.0)
    original_copy = deepcopy(original)
    for index, x in enumerate((0.03, 0.04, 0.05, 0.04, 0.05)):
        now = 100.0 + index * 0.25
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
        proposal = refiner.observe(7, 10 + index, 20 + index, pose(0.05), pose(), pose(), 100.0)
    assert proposal is not None


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
        now = 100.0 + index * 0.25
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
    refiner.commit(7, 101.0)
    assert burst(refiner, current=result, start=500.0) is None
    assert "encounter" in refiner.detail
    refiner.reset_observations()
    assert burst(refiner, current=result, start=1000.0) is None
    assert "encounter" in refiner.detail


def test_brief_occlusion_does_not_rearm_but_sustained_absence_does():
    refiner = LandmarkRefiner()
    result = burst(refiner)
    refiner.commit(7, 101.0)
    refiner.observe_visibility(set(), 200.0)
    assert burst(refiner, current=result, start=204.0) is None
    refiner.observe_visibility(set(), 210.0)
    assert burst(refiner, current=result, start=215.0) is not None


def test_new_encounter_must_still_wait_for_minimum_interval():
    refiner = LandmarkRefiner()
    result = burst(refiner)
    refiner.commit(7, 101.0)
    refiner.observe_visibility(set(), 102.0)
    assert burst(refiner, current=result, start=107.0) is None
    assert "interval" in refiner.detail
    assert burst(refiner, current=result, start=161.0) is not None


def test_failed_persistence_does_not_consume_encounter_until_commit():
    refiner = LandmarkRefiner()
    assert burst(refiner) is not None
    # The caller failed to persist, so subsequent independent evidence may retry.
    assert burst(refiner, start=102.0) is not None
    refiner.commit(7, 103.0)
    assert burst(refiner, start=104.0) is None


def test_lost_quality_discards_partial_burst_and_does_not_recount_old_capture():
    refiner = LandmarkRefiner()
    for index in range(4):
        now = 100.0 + index * 0.25
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    refiner.reset_observations()
    assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 0
    assert burst(refiner, start=102.0) is not None


def test_anchor_change_during_burst_requires_new_stable_observations():
    refiner = LandmarkRefiner()
    for index in range(4):
        now = 100.0 + index * 0.25
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    now = 101.0
    refiner.observe_visibility({7}, now)
    assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(0.001), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 1


def test_unstable_or_expired_observations_never_supply_a_complete_burst():
    refiner = LandmarkRefiner()
    for index in range(15):
        now = 100.0 + index * 0.25
        refiner.observe_visibility({7}, now)
        x = 0.01 if index % 2 else 0.10
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(x), pose(), pose(), now) is None
    assert "unstable" in refiner.detail
    for index in range(5):
        now = 200.0 + index * 7.0
        refiner.observe_visibility({7}, now)
        assert refiner.observe(7, int(now * 1e9), int(now * 1e9), pose(0.05), pose(), pose(), now) is None
    assert refiner._tags[7].window.sample_count == 1


def test_absence_and_disabled_policy_do_not_generate_proposals():
    refiner = LandmarkRefiner()
    for index in range(5):
        assert refiner.observe(7, index + 1, index + 1, pose(0.05), pose(), pose(), 100.0) is None
    disabled = LandmarkRefiner(replace(LandmarkRefinementConfig(), enabled=False))
    assert burst(disabled) is None


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
    assert config.maximum_localization_position_std_m == 0.10
    assert config.maximum_localization_yaw_std_rad == pytest.approx(math.radians(3.0))
    assert all(name.startswith("navigation.landmark_refinement.") for name in parameters)


@pytest.mark.parametrize("changes", [
    {"gain": 0.0}, {"gain": 1.1}, {"required_samples": True},
    {"required_samples": 1}, {"minimum_interval_sec": float("inf")},
    {"maximum_step_position_m": 0.06}, {"enabled": 1},
])
def test_invalid_refinement_config_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(LandmarkRefinementConfig(), **changes)
