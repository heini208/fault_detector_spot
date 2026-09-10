"""Focused tests for centralized Cartesian arm movement."""

from copy import deepcopy
import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
)
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
        return deepcopy(self.target)


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


def test_relative_uses_current_and_absolute_goal_for_speed(monkeypatch):
    relative = PoseStamped()
    relative.header.frame_id = "hand"
    relative.pose.position.x = 0.10
    relative.pose.orientation.w = 1.0

    current_transform = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=1.0,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): current_transform,
    })
    command = FakeRelativeCommand(relative)
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.relative(command)

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
    assert args[7] == executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    assert args[8] == pytest.approx(1.0)


def test_relative_speed_override_uses_same_current_to_goal_path(
    monkeypatch,
):
    relative = PoseStamped()
    relative.header.frame_id = "hand"
    relative.pose.position.x = 0.10
    relative.pose.orientation.w = 1.0

    current_transform = transform(
        executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
        "hand",
        x=1.0,
    )
    transformer = FakeTransformer({
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            "hand",
        ): current_transform,
    })
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.relative(
        FakeRelativeCommand(relative),
        speed=ArmMotionSpeed(
            linear_speed_mps=0.05,
            angular_speed_rad_s=0.25,
        ),
    )

    assert captured["args"][8] == pytest.approx(2.0)


def test_absolute_hand_pose_uses_current_to_goal_for_speed(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.4
    target.pose.orientation.w = 1.0

    current = transform(
        "body",
        "hand",
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", "hand"): current,
    })
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.pose(target)

    assert captured["args"][0] == pytest.approx(0.4)
    assert captured["args"][8] == pytest.approx(2.0)


def test_probe_pose_uses_probe_current_to_goal_for_speed(monkeypatch):
    probe_frame = "hall_probe_probe"
    current_probe = transform(
        "body",
        probe_frame,
        x=0.5,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", probe_frame): current_probe,
        ("hand", probe_frame): hand_to_probe,
    })

    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.7
    target.pose.orientation.w = 1.0

    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_pose(target, "hall_probe")

    assert captured["args"][0] == pytest.approx(0.5)
    assert captured["args"][8] == pytest.approx(2.0)


def test_tag_probe_forwards_speed_to_probe_motion(monkeypatch):
    tag_pose = PoseStamped()
    tag_pose.header.frame_id = "body"
    tag_pose.pose.orientation.w = 1.0

    probe_target = PoseStamped()
    probe_target.header.frame_id = "body"
    probe_target.pose.position.x = 0.8
    probe_target.pose.orientation.w = 1.0

    probe_frame = "hall_probe_probe"
    current_probe = transform(
        "body",
        probe_frame,
        x=0.6,
    )
    hand_to_probe = transform(
        "hand",
        probe_frame,
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", probe_frame): current_probe,
        ("hand", probe_frame): hand_to_probe,
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

    executor.tag_probe(
        command,
        speed=ArmMotionSpeed(
            linear_speed_mps=0.05,
            angular_speed_rad_s=0.25,
        ),
    )

    assert source.requests == [7]
    assert command.calls == [transformer]
    assert captured["args"][0] == pytest.approx(0.6)
    assert captured["args"][8] == pytest.approx(4.0)


def test_probe_relative_reuses_current_probe_transform(monkeypatch):
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
    offset.pose.position.x = 0.10
    offset.pose.orientation.w = 1.0

    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_relative(
        offset,
        "hall_probe",
    )

    assert transformer.calls == [
        (
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            probe_frame,
            0.0,
        ),
        ("hand", probe_frame, 0.0),
    ]
    assert captured["args"][0] == pytest.approx(0.9)
    assert captured["args"][8] == pytest.approx(1.0)


def test_bare_hand_probe_pose_uses_hand_speed_path(monkeypatch):
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.4
    target.pose.orientation.w = 1.0

    current = transform(
        "body",
        "hand",
        x=0.2,
    )
    transformer = FakeTransformer({
        ("body", "hand"): current,
    })
    captured = _capture_builder(monkeypatch)
    executor = ArmMovementExecutor(transformer)

    executor.probe_pose(target, "hand")

    assert transformer.calls == [
        ("body", "hand", 0.0),
    ]
    assert captured["args"][8] == pytest.approx(2.0)


def test_tag_probe_rejects_unreachable_tag():
    executor = ArmMovementExecutor(
        FakeTransformer(),
        tag_state_source=FakeTagStateSource({}),
    )
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    with pytest.raises(RuntimeError, match="not currently reachable"):
        executor.tag_probe(
            FakeTagCommand(7, "hand", target)
        )


def test_public_movement_methods_do_not_accept_duration():
    executor = ArmMovementExecutor(FakeTransformer())

    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0

    with pytest.raises(TypeError):
        executor.pose(target, duration_sec=2.0)
