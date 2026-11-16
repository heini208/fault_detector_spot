"""Directional translations select Cartesian planning without a free-path fallback."""

import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome, MoveItPlanUpdate
from test_arm_movement_executor import FakeRelativeCommand, transform
from test_arm_precision_execution import finish_motion, stable_feedback
from test_guarded_planning_frame_freshness import rig


def relative_rig(monkeypatch, *, axis="x", distance=0.1, yaw=0.0, precision=False):
    value = rig(monkeypatch)
    value.state.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round((10 + value.clock.now) * 1e9)),
    ))
    value.tf.transforms[("odom", "flat_body")] = transform("odom", "flat_body")
    offset = PoseStamped()
    offset.header.frame_id = "hand"
    setattr(offset.pose.position, axis, distance)
    offset.pose.orientation.z = math.sin(yaw / 2)
    offset.pose.orientation.w = math.cos(yaw / 2)
    command = FakeRelativeCommand(offset)
    command.offset = offset
    command.precision_mode = precision
    command.tag_position_tolerance_m = 0.005
    value.modes = []
    planner = value.executor.moveit_arm_planner
    original_start = planner.start

    def start(mode, target, **kwargs):
        value.modes.append(mode)
        return original_start(target, **kwargs)

    planner.start = lambda target, **kwargs: start("motion", target, **kwargs)
    planner.start_cartesian = lambda target, **kwargs: start("cartesian", target, **kwargs)
    value.update = value.executor.relative(command)
    return value


@pytest.mark.parametrize("axis", ["x", "y", "z"])
@pytest.mark.parametrize("distance", [-0.1, 0.1])
def test_directional_translation_uses_cartesian_planning(monkeypatch, axis, distance):
    value = relative_rig(monkeypatch, axis=axis, distance=distance)
    assert value.update.outcome is ArmMovementOutcome.RUNNING
    assert value.modes == ["cartesian"]
    assert getattr(value.starts[0].pose.position, axis) == pytest.approx(distance)
    assert value.starts[0].pose.orientation.w == pytest.approx(1.0)


@pytest.mark.parametrize("distance", [0.0, 0.1])
def test_rotation_and_combined_moves_keep_motion_planning(monkeypatch, distance):
    value = relative_rig(monkeypatch, distance=distance, yaw=0.2)
    assert value.update.outcome is ArmMovementOutcome.RUNNING
    assert value.modes == ["motion"]


def test_cartesian_failure_does_not_fall_back_or_dispatch(monkeypatch):
    value = relative_rig(monkeypatch)
    value.executor.moveit_arm_planner.poll = lambda: MoveItPlanUpdate(
        MoveItPlanOutcome.FAILURE, "Incomplete Cartesian path",
    )
    update = value.executor.poll()
    assert update.outcome is not ArmMovementOutcome.RUNNING
    assert update.outcome is not ArmMovementOutcome.SUCCESS
    assert value.modes == ["cartesian"]
    assert value.client.sent_goals == []


def test_precision_correction_keeps_cartesian_planning(monkeypatch):
    value = relative_rig(monkeypatch, precision=True)
    assert value.executor.poll().outcome is ArmMovementOutcome.RUNNING
    finish_motion(value)
    assert stable_feedback(value, position=0.088).outcome is ArmMovementOutcome.RUNNING
    assert value.modes == ["cartesian", "cartesian"]
