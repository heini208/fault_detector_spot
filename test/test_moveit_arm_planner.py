"""Offline checks for MoveIt planning failure diagnostics."""

from types import SimpleNamespace

import pytest
from moveit_msgs.msg import MoveItErrorCodes

from fault_detector_spot.manipulation.moveit_arm_planner import (
    DEFAULT_POSITION_TOLERANCE_M,
    MoveItArmPlanner,
    MoveItPlanOutcome,
)


def test_default_position_tolerance_is_two_millimeters():
    assert DEFAULT_POSITION_TOLERANCE_M == pytest.approx(0.002)


@pytest.mark.parametrize("code, name", [
    (-27, "GOAL_STATE_INVALID"),
    (-26, "START_STATE_INVALID"),
    (-31, "NO_IK_SOLUTION"),
    (-12345, "UNKNOWN"),
])
def test_failed_plan_reports_symbolic_code_and_releases_future(code, name):
    planner = object.__new__(MoveItArmPlanner)
    planner._started_at = 0.0
    planner._future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(motion_plan_response=SimpleNamespace(
            error_code=MoveItErrorCodes(val=code),
        )),
    )

    update = planner.poll()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert name in update.detail
    assert str(code) in update.detail
    assert not planner.active
    if code == -27:
        assert "check move_group logs" in update.detail
