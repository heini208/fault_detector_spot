"""Focused tests for shared Cartesian arm movement construction."""

from copy import deepcopy
import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)


class FakeTransformer:
    def __init__(self, transforms=None):
        self.transforms = transforms or {}
        self.calls = []

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        self.calls.append((target, source, timeout_sec))
        key = (target, source)
        if key not in self.transforms:
            raise AssertionError(
                f"Unexpected TF lookup: {target} <- {source}"
            )
        return self.transforms[key]


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
    def __init__(self, tag_id, sensor_id, probe_target):
        self.tag_id = tag_id
        self.motion_sensor_id = sensor_id
        self.tag_pose = PoseStamped()
        self.probe_target = probe_target
        self.calls = []

    def compute_goal_pose(self, transformer):
        self.calls.append(transformer)
        return deepcopy(self.probe_target)


def transform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    result.transform.translation.x = x
    result.transform.translation.y = y
    result.transform.translation.z = z
    result.transform.rotation.z = math.sin(yaw * 0.5)
    result.transform.rotation.w = math.cos(yaw * 0.5)
    return result


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

    hand_to_execution = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=1.0,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): hand_to_execution,
    })
    command = FakeRelativeCommand(target)
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.relative(command, 2.0)

    assert command.calls == [transformer]
    args = captured["args"]
    assert args[0] == pytest.approx(1.10)
    assert args[7] == executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    assert args[8] == pytest.approx(2.0)


def test_probe_pose_converts_probe_target_to_hand_target(monkeypatch):
    hand_to_probe = transform(
        "hand",
        "hall_probe_probe",
        x=0.2,
        yaw=math.radians(90.0),
    )
    transformer = FakeTransformer({
        ("hand", "hall_probe_probe"): hand_to_probe,
    })
    probe_target = PoseStamped()
    probe_target.header.frame_id = "body"
    probe_target.pose.position.x = 1.0
    probe_target.pose.orientation.w = 1.0
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_pose(
        probe_target,
        "hall_probe",
        3.0,
    )

    args = captured["args"]
    assert args[0] == pytest.approx(1.0)
    assert args[1] == pytest.approx(0.2)
    assert args[2] == pytest.approx(0.0)
    assert args[3] == pytest.approx(math.sqrt(0.5))
    assert args[6] == pytest.approx(-math.sqrt(0.5))
    assert args[7] == "body"


def test_bare_hand_probe_pose_needs_no_attachment_tf(monkeypatch):
    probe_target = PoseStamped()
    probe_target.header.frame_id = "body"
    probe_target.pose.position.x = 0.4
    probe_target.pose.orientation.w = 1.0
    transformer = FakeTransformer()
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_pose(probe_target, "hand", 2.0)

    assert transformer.calls == []
    assert captured["args"][0] == pytest.approx(0.4)


def test_tag_probe_uses_latest_tag_then_probe_conversion(monkeypatch):
    tag_pose = PoseStamped()
    tag_pose.header.frame_id = "body"
    tag_pose.pose.orientation.w = 1.0

    probe_target = PoseStamped()
    probe_target.header.frame_id = "body"
    probe_target.pose.position.x = 0.8
    probe_target.pose.orientation.w = 1.0

    hand_to_probe = transform(
        "hand",
        "hall_probe_probe",
        x=0.2,
    )
    transformer = FakeTransformer({
        ("hand", "hall_probe_probe"): hand_to_probe,
    })
    source = FakeTagStateSource({
        7: FakeTag(7, tag_pose),
    })
    command = FakeTagCommand(
        7,
        "hall_probe",
        probe_target,
    )
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(
        transformer,
        tag_state_source=source,
    )

    executor.tag_probe(command, 3.0)

    assert source.requests == [7]
    assert command.calls == [transformer]
    assert command.tag_pose is not tag_pose
    assert captured["args"][0] == pytest.approx(0.6)
    assert captured["args"][7] == "body"


def test_tag_probe_rejects_tag_that_is_not_reachable():
    executor = ArmMovementExecutor(
        FakeTransformer(),
        tag_state_source=FakeTagStateSource({}),
    )
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    with pytest.raises(RuntimeError, match="not currently reachable"):
        executor.tag_probe(
            FakeTagCommand(7, "hand", target),
            3.0,
        )


def test_probe_relative_rotates_about_probe_tip(monkeypatch):
    probe_frame = "hall_probe_probe"
    current_probe = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        probe_frame,
        x=1.0,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
        ): current_probe,
        ("hand", probe_frame): hand_to_probe,
    })

    offset = PoseStamped()
    offset.header.frame_id = probe_frame
    offset.pose.orientation.z = math.sqrt(0.5)
    offset.pose.orientation.w = math.sqrt(0.5)

    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_relative(
        offset,
        "hall_probe",
        2.0,
    )

    args = captured["args"]
    assert args[0] == pytest.approx(1.0)
    assert args[1] == pytest.approx(-0.2)
    assert args[3] == pytest.approx(math.sqrt(0.5))
    assert args[6] == pytest.approx(math.sqrt(0.5))
    assert args[7] == executor_module.GRAV_ALIGNED_BODY_FRAME_NAME


def test_probe_relative_requires_active_probe_frame():
    offset = PoseStamped()
    offset.header.frame_id = "body"
    offset.pose.orientation.w = 1.0
    executor = ArmMovementExecutor(FakeTransformer())

    with pytest.raises(ValueError, match="active probe frame"):
        executor.probe_relative(
            offset,
            "hall_probe",
            2.0,
        )


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
