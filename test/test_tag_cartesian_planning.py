"""Move-to-tag and its corrections retain Cartesian planning."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome, MoveItPlanUpdate
from fault_detector_spot.manipulation.probe_motion_planner import ResolvedProbeTarget
from test_arm_precision_execution import finish_motion, stable_feedback
from test_guarded_planning_frame_freshness import rig


def tag_rig(monkeypatch, precision):
    value = rig(monkeypatch)
    value.state.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round((10 + value.clock.now) * 1e9)),
    ))
    monkeypatch.setattr(value.executor.probe_motion_planner, "resolve_tag",
                        lambda *_: ResolvedProbeTarget(deepcopy(value.target), "hand"))
    planner = value.executor.moveit_arm_planner
    original_start = planner.start_cartesian
    value.modes = []

    def start(target, **kwargs):
        value.modes.append("cartesian")
        return original_start(target, **kwargs)

    def unexpected_motion(*_args, **_kwargs):
        pytest.fail("Move-to-tag must not fall back to unrestricted planning")

    planner.start_cartesian = start
    planner.start = unexpected_motion
    command = SimpleNamespace(
        precision_mode=precision, tag_position_tolerance_m=0.005,
        ignore_environment_collisions=False,
    )
    assert value.executor.tag_probe(command).outcome is ArmMovementOutcome.RUNNING
    return value


@pytest.mark.parametrize("precision", [False, True])
def test_tag_initial_motion_and_correction_use_cartesian(monkeypatch, precision):
    value = tag_rig(monkeypatch, precision)
    assert value.modes == ["cartesian"]
    assert value.starts[0].pose == value.target.pose
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    finish_motion(value)
    assert stable_feedback(value, position=0.988).outcome is ArmMovementOutcome.RUNNING
    assert value.modes == ["cartesian", "cartesian"]


@pytest.mark.parametrize("precision", [False, True])
def test_tag_cartesian_failure_does_not_dispatch_or_fall_back(monkeypatch, precision):
    value = tag_rig(monkeypatch, precision)
    value.executor.moveit_arm_planner.poll = lambda: MoveItPlanUpdate(
        MoveItPlanOutcome.FAILURE, "Incomplete Cartesian path",
    )
    update = value.executor.poll()
    assert update.outcome not in (ArmMovementOutcome.RUNNING, ArmMovementOutcome.SUCCESS)
    assert value.modes == ["cartesian"]
    assert value.client.sent_goals == []
