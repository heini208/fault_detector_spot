"""Regression tests for SciPy-backed movement-command rotations."""

import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.application.behaviour_tree.commands.move_command import (
    MoveCommand,
)
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)


class FakeTransformer:
    def __init__(self, transform):
        self.transform = transform
        self.calls = []

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        self.calls.append((target, source, timeout_sec))
        return self.transform


def transform_with_quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    transform = TransformStamped()
    transform.transform.rotation.x = x
    transform.transform.rotation.y = y
    transform.transform.rotation.z = z
    transform.transform.rotation.w = w
    return transform


def command():
    return MoveCommand("test", Time())


def test_vector_rotation_uses_full_scipy_frame_rotation():
    half = math.radians(90.0) * 0.5
    transformer = FakeTransformer(
        transform_with_quaternion(z=math.sin(half), w=math.cos(half))
    )

    rotated = command()._rotate_vector_into_frame(
        np.array([1.0, 0.0, 0.0]),
        "source",
        "target",
        transformer,
    )

    assert rotated == pytest.approx(np.array([0.0, 1.0, 0.0]))


def test_quaternion_rotation_preserves_composition_order():
    half = math.radians(90.0) * 0.5
    transformer = FakeTransformer(
        transform_with_quaternion(z=math.sin(half), w=math.cos(half))
    )

    rotated = command()._rotate_quaternion_into_frame(
        [0.0, 0.0, 0.0, 1.0],
        "source",
        "target",
        transformer,
    )

    assert rotated == pytest.approx(
        [0.0, 0.0, math.sin(half), math.cos(half)]
    )


def test_tag_heading_remains_based_on_transformed_negative_z_normal():
    half = math.radians(90.0) * 0.5
    transformer = FakeTransformer(
        transform_with_quaternion(x=math.sin(half), w=math.cos(half))
    )

    rotated = command()._rotate_only_yaw_into_frame(
        [0.0, 0.0, 0.0, 1.0],
        "tag",
        "body",
        transformer,
    )

    expected = math.sqrt(0.5)
    assert rotated == pytest.approx([0.0, 0.0, expected, expected])


def test_tag_yaw_only_vector_rotation_preserves_vertical_component():
    half = math.radians(90.0) * 0.5
    transformer = FakeTransformer(
        transform_with_quaternion(x=math.sin(half), w=math.cos(half))
    )

    rotated = command()._rotate_vector_into_frame_yaw_only(
        np.array([1.0, 0.0, 0.4]),
        "tag",
        "body",
        transformer,
    )

    assert rotated == pytest.approx(np.array([0.0, 1.0, 0.4]))


def test_different_frame_rotation_requires_transformer():
    with pytest.raises(RuntimeError, match="requires TF"):
        command()._rotate_vector_into_frame(
            np.array([1.0, 0.0, 0.0]),
            "source",
            "target",
            None,
        )


def test_same_frame_rotation_does_not_require_transformer():
    rotated = command()._rotate_vector_into_frame(
        np.array([1.0, 2.0, 3.0]),
        "body",
        "body",
        None,
    )

    assert rotated == pytest.approx(np.array([1.0, 2.0, 3.0]))


def test_tag_transform_without_tf_is_only_allowed_in_same_frame():
    tag = PoseStamped()
    tag.header.frame_id = "body"
    tag.pose.orientation.w = 1.0
    move = MoveToTagCommand(
        "test",
        Time(),
        tag,
        1,
        target_frame="body",
    )

    resolved = move.transform_tag_to_target_frame(None)

    assert resolved.header.frame_id == "body"

    move.target_frame = "odom"
    with pytest.raises(RuntimeError, match="requires TF"):
        move.transform_tag_to_target_frame(None)
