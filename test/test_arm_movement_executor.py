"""Focused tests for shared Cartesian arm movement construction."""

from copy import deepcopy

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)


class FakeTransformer:
    def __init__(self, transform=None):
        self.transform = transform
        self.calls = []

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        self.calls.append((target, source, timeout_sec))
        if self.transform is None:
            raise AssertionError("Unexpected TF lookup")
        return self.transform


class FakeRelativeCommand:
    def __init__(self, target):
        self.target = target
        self.calls = []

    def compute_goal_pose(self, transformer):
        self.calls.append(transformer)
        return self.target


class FakeTag:
    def __init__(self, tag_id, pose):
        self.id = tag_id
        self.pose = pose


class FakeTagStateSource:
    def __init__(self, tags):
        self.tags = tags
        self.requests = []

    def reachable_tag(self, tag_id):
        self.requests.append(tag_id)
        tag = self.tags.get(tag_id)
        return None if tag is None else deepcopy(tag)


class FakeTagCommand:
    def __init__(self, tag_id):
        self.tag_id = tag_id
        self.tag_pose = PoseStamped()
        self.calls = []

    def compute_goal_pose(self, transformer):
        self.calls.append(transformer)
        return deepcopy(self.tag_pose)


def _capture_builder(monkeypatch):
    captured = {}

    def build(*args):
        captured["args"] = args
        return object()

    monkeypatch.setattr(
        executor_module.RobotCommandBuilder,
        "arm_pose_command",
        build,
    )
    monkeypatch.setattr(
        executor_module,
        "convert",
        lambda source, target: None,
    )
    return captured


def test_relative_motion_is_normalized_to_gravity_aligned_body(
    monkeypatch,
):
    target = PoseStamped()
    target.header.frame_id = "hand"
    target.pose.position.x = 0.10
    target.pose.orientation.w = 1.0

    transform = TransformStamped()
    transform.header.frame_id = (
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    )
    transform.child_frame_id = "hand"
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0

    transformer = FakeTransformer(transform)
    command = FakeRelativeCommand(target)
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.relative(command, 2.0)

    assert command.calls == [transformer]
    assert transformer.calls == [
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
            0.0,
        )
    ]
    args = captured["args"]
    assert args[0] == pytest.approx(1.10)
    assert args[1] == pytest.approx(0.0)
    assert args[2] == pytest.approx(0.0)
    assert args[7] == executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    assert args[8] == pytest.approx(2.0)


def test_tag_pose_uses_latest_reachable_tag_snapshot(monkeypatch):
    pose = PoseStamped()
    pose.header.frame_id = "body"
    pose.pose.position.x = 0.7
    pose.pose.orientation.w = 1.0

    source = FakeTagStateSource({7: FakeTag(7, pose)})
    transformer = FakeTransformer()
    command = FakeTagCommand(7)
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(
        transformer,
        tag_state_source=source,
    )

    executor.tag_pose(command, 3.0)

    assert source.requests == [7]
    assert command.calls == [transformer]
    assert command.tag_pose is not pose
    assert command.tag_pose.pose.position.x == pytest.approx(0.7)
    assert captured["args"][0] == pytest.approx(0.7)
    assert captured["args"][7] == "body"
    assert captured["args"][8] == pytest.approx(3.0)


def test_tag_pose_rejects_tag_that_is_not_reachable():
    executor = ArmMovementExecutor(
        FakeTransformer(),
        tag_state_source=FakeTagStateSource({}),
    )

    with pytest.raises(RuntimeError, match="not currently reachable"):
        executor.tag_pose(FakeTagCommand(7), 3.0)


def test_pose_motion_preserves_supplied_target_frame(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.4
    target.pose.position.y = -0.2
    target.pose.position.z = 0.8
    target.pose.orientation.w = 1.0

    transformer = FakeTransformer()
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.pose(target, 3.0)

    assert transformer.calls == []
    args = captured["args"]
    assert args[0] == pytest.approx(0.4)
    assert args[1] == pytest.approx(-0.2)
    assert args[2] == pytest.approx(0.8)
    assert args[7] == "body"
    assert args[8] == pytest.approx(3.0)


def test_pose_motion_rejects_invalid_duration():
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    executor = ArmMovementExecutor(FakeTransformer())

    with pytest.raises(ValueError, match="duration"):
        executor.pose(target, 0.0)
