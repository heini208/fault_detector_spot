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
            ready_module.READY_STATE_TIMEOUT_PARAMETER: 2.0,
            ready_module.READY_TF_TIMEOUT_PARAMETER: 2.0,
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


class _Clock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class _Blackboard(SimpleNamespace):

    def exists(self, key):
        return hasattr(self, key)


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
    action._hand_transform = _TransformSource().lookup_a_tform_b(
        ready_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        ready_module.HAND_FRAME_NAME,
        timeout_sec=0.0,
    )
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


def test_ready_arm_waits_for_initial_stow_state():
    clock = _Clock()
    action = ReadyArmActionSimple(
        robot_command_resources=object(),
        monotonic_clock=clock,
    )
    action.node = _Node()
    action.arm_state_source = _ArmStateSource(None)
    action.send_goal_future = None

    assert action._phase_send_goal() is Status.RUNNING
    assert action.feedback_message == "Waiting for manipulator stow state"

    clock.now = 1.0
    action.arm_state_source.state = ArmStowState.UNKNOWN
    assert action._phase_send_goal() is Status.RUNNING

    action.arm_state_source.state = ArmStowState.DEPLOYED
    assert action._phase_send_goal() is Status.SUCCESS


def test_ready_arm_reports_state_timeout_with_correlated_detail():
    clock = _Clock()
    action = ReadyArmActionSimple(
        robot_command_resources=object(),
        monotonic_clock=clock,
    )
    action.node = _Node()
    action.blackboard = _Blackboard(
        last_command=SimpleNamespace(request_id="ready-request"),
        command_failure_request_id="",
        command_failure_detail="",
    )
    action.arm_state_source = _ArmStateSource(None)
    action.send_goal_future = None

    assert action._phase_send_goal() is Status.RUNNING
    clock.now = 2.0
    assert action._phase_send_goal() is Status.FAILURE
    assert action.blackboard.command_failure_request_id == "ready-request"
    assert action.blackboard.command_failure_detail == (
        "Fresh manipulator stow state was unavailable for 2.0 s"
    )


class _MissingTransformSource:

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        raise RuntimeError(
            f'"{target}" passed to lookupTransform argument target_frame '
            "does not exist"
        )


def test_ready_arm_waits_for_startup_tf_with_correlated_timeout_detail():
    clock = _Clock()
    action = ReadyArmActionSimple(
        robot_command_resources=object(),
        monotonic_clock=clock,
    )
    action.node = _Node()
    action.blackboard = _Blackboard(
        last_command=SimpleNamespace(request_id="ready-tf-request"),
        command_failure_request_id="",
        command_failure_detail="",
    )
    action.arm_state_source = _ArmStateSource(ArmStowState.STOWED)
    action.tf_listener = _MissingTransformSource()
    action.send_goal_future = None

    assert action._phase_send_goal() is Status.RUNNING
    assert "Waiting for ready-arm hand pose transform" in action.feedback_message

    clock.now = 2.0
    assert action._phase_send_goal() is Status.FAILURE
    assert action.blackboard.command_failure_request_id == "ready-tf-request"
    assert "flat_body -> hand" in action.blackboard.command_failure_detail
    assert "was unavailable for 2.0 s" in action.blackboard.command_failure_detail
