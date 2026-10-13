"""Shared stability and correction bounds are independent of execution owners."""

import math

import pytest

from fault_detector_spot.manipulation.pose_precision import PoseCorrectionBudget, PoseStabilityWindow
from fault_detector_spot.shared.geometry.models import PoseData


def pose(x=0.0, yaw=0.0):
    value = PoseData.identity()
    value.position.x = x
    value.orientation.z = math.sin(yaw / 2)
    value.orientation.w = math.cos(yaw / 2)
    return value


def test_stability_requires_distinct_stamps_and_full_stable_span():
    window = PoseStabilityWindow()
    assert not window.add(1.0, (pose(),))
    with pytest.raises(ValueError, match="distinct"):
        window.add(1.0, (pose(),))
    assert not window.add(1.1, (pose(),))
    assert not window.add(1.2, (pose(),))
    assert window.add(1.3, (pose(),))
    assert window.sample_count == 4


@pytest.mark.parametrize("moved", [pose(0.0011), pose(yaw=math.radians(0.21))])
def test_movement_of_any_pose_restarts_stability_window(moved):
    window = PoseStabilityWindow()
    window.add(1.0, (pose(), pose()))
    window.add(1.15, (pose(), pose()))
    assert not window.add(1.3, (pose(), moved))
    assert window.sample_count == 1
    assert not window.add(1.45, (pose(), moved))
    assert window.add(1.6, (pose(), moved))


def test_stale_gap_and_invalid_pose_invalidate_window():
    window = PoseStabilityWindow()
    window.add(1.0, (pose(),))
    assert not window.add(1.3, (pose(),))
    assert window.sample_count == 1
    with pytest.raises(ValueError, match="non-finite"):
        window.add(1.4, (pose(float("nan")),))
    assert window.sample_count == 0


def test_reset_diagnostics_report_named_pose_motion_and_limits():
    window = PoseStabilityWindow(pose_names=("hand", "sensor tip", "body"))
    window.add(1.0, (pose(), pose(), pose()))
    window.add(1.15, (pose(), pose(), pose()))
    assert not window.add(1.3, (pose(), pose(0.0015, math.radians(0.3)), pose()))
    assert window.last_reset_reason == (
        "sensor tip changed 1.50 mm / 0.300 deg (limits 1.00 mm / 0.200 deg)")
    assert window.sample_count == 1
    assert window.last_stamp_seconds == pytest.approx(1.3)
    with pytest.raises(ValueError, match="distinct"):
        window.add(1.3, (pose(), pose(), pose()))
    assert window.sample_count == 1
    assert "sensor tip changed" in window.last_reset_reason


def test_gap_diagnostic_and_invalidation_preserve_last_accepted_stamp():
    window = PoseStabilityWindow()
    window.add(1.0, (pose(),))
    window.add(1.4, (pose(),))
    assert window.last_reset_reason == "Pose acquisition gap 0.400 s exceeded 0.250 s limit"
    window.invalidate("TF unavailable")
    assert window.sample_count == 0
    assert window.last_stamp_seconds == pytest.approx(1.4)
    assert window.last_reset_reason == "TF unavailable"
    window.reset()
    assert window.last_stamp_seconds == 0.0
    assert window.last_reset_reason == ""


def test_unnamed_pose_reset_identifies_its_index():
    window = PoseStabilityWindow()
    window.add(1.0, (pose(), pose()))
    window.add(1.1, (pose(), pose(0.002)))
    assert window.last_reset_reason.startswith("pose 2 changed 2.00 mm")


def test_correction_reserves_up_to_three_improving_attempts():
    budget = PoseCorrectionBudget()
    for error in (0.012, 0.010, 0.008):
        budget.reserve(error, 0.0, 0.005, math.radians(2))
    assert budget.corrections == 3
    assert budget.total_translation_m == pytest.approx(0.03)
    with pytest.raises(ValueError, match="limit reached"):
        budget.reserve(0.006, 0.0, 0.005, math.radians(2))


@pytest.mark.parametrize("position, angle", [(0.0201, 0.0), (0.0, math.radians(5.01))])
def test_single_correction_bounds_reject_without_reserving(position, angle):
    budget = PoseCorrectionBudget()
    with pytest.raises(ValueError, match="bounded correction range"):
        budget.reserve(position, angle, 0.005, math.radians(2))
    assert budget.corrections == 0


def test_normalized_orientation_regression_blocks_position_improvement():
    budget = PoseCorrectionBudget()
    budget.reserve(0.010, math.radians(1), 0.005, math.radians(2))
    with pytest.raises(ValueError, match="did not improve"):
        budget.reserve(0.005, math.radians(4), 0.005, math.radians(2))
    assert budget.corrections == 1


@pytest.mark.parametrize("position, angle", [(True, False), (False, True)])
def test_total_travel_budget_rejects_even_improving_attempts(position, angle):
    budget = PoseCorrectionBudget()
    for error in (1.0, 0.85):
        budget.reserve(0.02 * error if position else 0.0,
                       math.radians(5) * error if angle else 0.0,
                       0.005, math.radians(2))
    with pytest.raises(ValueError, match="travel budget"):
        budget.reserve(0.014 if position else 0.0, math.radians(3.5) if angle else 0.0,
                       0.005, math.radians(2))


def test_exact_required_improvement_is_accepted():
    budget = PoseCorrectionBudget()
    budget.reserve(0.01, 0.0, 0.005, math.radians(2))
    budget.reserve(0.009, 0.0, 0.005, math.radians(2))
    assert budget.corrections == 2
