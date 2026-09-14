"""Tests for six-axis arm joint telemetry cached from joint_states."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_joint_state_source import (
    ARM_JOINT_NAMES,
    ARM_JOINT_STATE_TOPIC,
    ArmJointStateSource,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeNode:

    def __init__(self):
        self.callback = None
        self.subscription = object()
        self.destroyed = []
        self.subscription_calls = []

    def create_subscription(
        self,
        message_type,
        topic,
        callback,
        depth,
    ):
        self.callback = callback
        self.subscription_calls.append(
            (message_type, topic, depth)
        )
        return self.subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def joint_state(
    *,
    names=ARM_JOINT_NAMES,
    positions=None,
    velocities=None,
    efforts=None,
):
    names = tuple(names)
    size = len(names)
    if positions is None:
        positions = tuple(float(index) for index in range(size))
    if velocities is None:
        velocities = tuple(
            float(index) / 10.0 for index in range(size)
        )
    if efforts is None:
        efforts = tuple(
            float(index) + 10.0 for index in range(size)
        )
    return SimpleNamespace(
        name=list(names),
        position=list(positions),
        velocity=list(velocities),
        effort=list(efforts),
    )


def test_source_subscribes_once_to_joint_states():
    node = FakeNode()

    source = ArmJointStateSource(node)

    assert len(node.subscription_calls) == 1
    assert node.subscription_calls[0][1] == ARM_JOINT_STATE_TOPIC
    source.destroy()


def test_source_exposes_complete_arm_state_by_canonical_name():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )

    clock.now = 1.25
    node.callback(joint_state())

    sample = source.sample()

    assert sample.received_at == pytest.approx(1.25)
    assert tuple(sample.joints) == ARM_JOINT_NAMES
    assert sample.joints["arm_sh0"].position_rad == pytest.approx(0.0)
    assert sample.joints["arm_el1"].velocity_rad_s == pytest.approx(0.3)
    assert sample.joints["arm_wr1"].effort_nm == pytest.approx(15.0)


def test_joint_order_and_robot_prefix_do_not_change_mapping():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )
    names = tuple(
        f"spot/{name}"
        for name in reversed(ARM_JOINT_NAMES)
    )
    positions = tuple(float(index + 20) for index in range(6))
    velocities = tuple(float(index + 30) for index in range(6))
    efforts = tuple(float(index + 40) for index in range(6))

    node.callback(
        joint_state(
            names=names,
            positions=positions,
            velocities=velocities,
            efforts=efforts,
        )
    )

    sample = source.sample()

    assert sample.joints["arm_wr1"].position_rad == pytest.approx(20.0)
    assert sample.joints["arm_sh0"].position_rad == pytest.approx(25.0)
    assert sample.joints["arm_wr1"].effort_nm == pytest.approx(40.0)
    assert sample.joints["arm_sh0"].effort_nm == pytest.approx(45.0)


def test_extra_non_controlled_arm_and_body_joints_are_ignored():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )
    names = (
        "front_left_hip_x",
        "arm_hr0",
        *ARM_JOINT_NAMES,
        "arm_f1x",
    )

    node.callback(joint_state(names=names))

    sample = source.sample()

    assert tuple(sample.joints) == ARM_JOINT_NAMES


@pytest.mark.parametrize(
    "message",
    (
        joint_state(names=ARM_JOINT_NAMES[:-1]),
        joint_state(efforts=()),
        joint_state(positions=(0.0,) * 5),
        joint_state(velocities=(0.0,) * 5),
        joint_state(efforts=(0.0,) * 5),
        joint_state(
            efforts=(0.0, 0.0, 0.0, float("nan"), 0.0, 0.0)
        ),
    ),
)
def test_incomplete_or_invalid_latest_message_makes_sample_unavailable(
    message,
):
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )

    node.callback(joint_state())
    assert source.sample() is not None

    clock.now = 0.1
    node.callback(message)

    assert source.sample() is None
    assert not source.is_stale()


def test_duplicate_required_joint_name_is_rejected():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )
    names = (*ARM_JOINT_NAMES, "spot/arm_sh0")

    node.callback(joint_state(names=names))

    assert source.sample() is None


def test_sample_becomes_unavailable_when_joint_state_is_stale():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        stale_after_sec=1.0,
        monotonic_clock=clock,
    )

    node.callback(joint_state())
    assert source.sample() is not None

    clock.now = 1.01

    assert source.sample() is None
    assert source.is_stale()


def test_new_valid_message_replaces_invalid_state():
    clock = ManualClock()
    node = FakeNode()
    source = ArmJointStateSource(
        node,
        monotonic_clock=clock,
    )

    node.callback(joint_state(efforts=()))
    assert source.sample() is None

    clock.now = 0.2
    node.callback(joint_state(efforts=(1.0,) * 6))

    sample = source.sample()
    assert sample is not None
    assert sample.received_at == pytest.approx(0.2)
    assert all(
        joint.effort_nm == pytest.approx(1.0)
        for joint in sample.joints.values()
    )


def test_destroy_releases_subscription_once():
    node = FakeNode()
    source = ArmJointStateSource(node)

    source.destroy()
    source.destroy()

    assert node.destroyed == [node.subscription]
