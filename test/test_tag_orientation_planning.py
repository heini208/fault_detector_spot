"""Move-to-tag and its corrections retain Orientation-constrained planning."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome, MoveItPlanUpdate
from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget
from test_arm_precision_execution import finish_motion, stable_feedback
from test_guarded_planning_frame_freshness import rig


def tag_rig(monkeypatch, precision, yaw=0.0):
    value = rig(monkeypatch)
    value.target.pose.orientation.z = math.sin(yaw / 2)
    value.target.pose.orientation.w = math.cos(yaw / 2)
    value.state.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round((10 + value.clock.now) * 1e9)),
    ))
    monkeypatch.setattr(value.executor.probe_motion_planner, "resolve_tag",
                        lambda *_: ResolvedProbeTarget(deepcopy(value.target), "hand"))
    planner = value.executor.moveit_arm_planner
    original_start = planner.start
    value.modes = []

    def start(target, **kwargs):
        value.modes.append("orientation")
        value.path_options = kwargs.copy()
        assert "path_orientation" in kwargs
        kwargs.pop("path_orientation")
        kwargs.pop("path_orientation_tolerance_rad")
        return original_start(target, **kwargs)

    def unexpected_motion(*_args, **_kwargs):
        pytest.fail("Move-to-tag must not fall back to unrestricted planning")

    planner.start = start
    planner.start_cartesian = unexpected_motion
    command = SimpleNamespace(
        precision_mode=precision, tag_position_tolerance_m=0.005,
        ignore_environment_collisions=False,
    )
    assert value.executor.tag_probe(command).outcome is ArmMovementOutcome.RUNNING
    return value


@pytest.mark.parametrize("precision", [False, True])
def test_tag_initial_motion_and_correction_use_orientation(monkeypatch, precision):
    value = tag_rig(monkeypatch, precision)
    assert value.modes == ["orientation"]
    assert value.starts[0].pose == value.target.pose
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    finish_motion(value)
    assert stable_feedback(value, position=0.988).outcome is ArmMovementOutcome.RUNNING
    assert value.modes == ["orientation", "orientation"]


@pytest.mark.parametrize("precision", [False, True])
def test_tag_orientation_failure_does_not_dispatch_or_fall_back(monkeypatch, precision):
    value = tag_rig(monkeypatch, precision)
    value.executor.moveit_arm_planner.poll = lambda: MoveItPlanUpdate(
        MoveItPlanOutcome.FAILURE, "Incomplete Orientation-constrained path",
    )
    update = value.executor.poll()
    assert update.outcome not in (ArmMovementOutcome.RUNNING, ArmMovementOutcome.SUCCESS)
    assert value.modes == ["orientation"]
    assert value.client.sent_goals == []


def test_tag_requested_rotation_is_included_in_orientation_envelope(monkeypatch):
    value = tag_rig(monkeypatch, False, yaw=0.4)
    assert value.path_options["path_orientation"].w == pytest.approx(1.0)
    assert value.path_options["path_orientation_tolerance_rad"] == pytest.approx(
        0.4 + math.radians(5.0),
    )
    actual = value.starts[0].pose.orientation
    expected = value.target.pose.orientation
    assert [actual.x, actual.y, actual.z, actual.w] == pytest.approx(
        [expected.x, expected.y, expected.z, expected.w],
    )
