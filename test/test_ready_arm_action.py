"""Focused tests for the custom ready-arm movement."""

from types import SimpleNamespace

from py_trees.common import Status

import fault_detector_spot.manipulation.behaviours.ready_arm_action as ready_module
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.manipulation.behaviours.ready_arm_action import (
    ReadyArmActionSimple,
)


class _Parameter:

    def __init__(self, value):
        self.value = value


class _Node:

    def __init__(self):
        self.parameters = {
            ready_module.READY_LIFT_DISTANCE_PARAMETER: 0.10,
            ready_module.READY_DURATION_PARAMETER: 2.0,
            ready_module.READY_DEPLOYED_TIMEOUT_PARAMETER: 2.0,
        }

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _TransformSource:

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        assert target == ready_module.GRAV_ALIGNED_BODY_FRAME_NAME
        assert source == ready_module.HAND_FRAME_NAME
        assert timeout_sec == 0.0
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.2, y=-0.1, z=0.4),
                rotation=SimpleNamespace(w=0.9, x=0.1, y=0.2, z=0.3),
            )
        )


class _ArmStateSource:

    def __init__(self, state):
        self.state = state

    def stow_state(self):
        return self.state


def test_ready_arm_targets_only_positive_body_z(monkeypatch):
    captured = {}

    def build_arm_pose(*args):
        captured["args"] = args
        return object()

    monkeypatch.setattr(
        ready_module.RobotCommandBuilder,
        "arm_pose_command",
        build_arm_pose,
    )
    monkeypatch.setattr(ready_module, "convert", lambda _src, _dst: None)

    action = ReadyArmActionSimple(robot_command_resources=object())
    action.node = _Node()
    action.tf_listener = _TransformSource()
    action._build_goal()

    args = captured["args"]
    assert args[0] == 0.2
    assert args[1] == -0.1
    assert args[2] == 0.5
    assert args[3:7] == (0.9, 0.1, 0.2, 0.3)
    assert args[8] == 2.0


def test_ready_arm_noops_when_already_deployed():
    action = ReadyArmActionSimple(robot_command_resources=object())
    action.arm_state_source = _ArmStateSource(ArmStowState.DEPLOYED)
    action.send_goal_future = None

    assert action._phase_send_goal() is Status.SUCCESS
    assert action.feedback_message == "Arm is already deployed"


def test_ready_arm_rejects_missing_or_unknown_state():
    action = ReadyArmActionSimple(robot_command_resources=object())
    action.send_goal_future = None

    action.arm_state_source = _ArmStateSource(None)
    assert action._phase_send_goal() is Status.FAILURE

    action.arm_state_source = _ArmStateSource(ArmStowState.UNKNOWN)
    assert action._phase_send_goal() is Status.FAILURE
