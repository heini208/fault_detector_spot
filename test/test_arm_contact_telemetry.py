"""Tests for shadow contact telemetry collection."""

import json
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.manipulation.arm_contact_telemetry import (
    ArmContactTelemetry,
)
from fault_detector_spot.manipulation.arm_joint_state_source import (
    ARM_JOINT_NAMES,
)


class FakeArmStateSource:

    def __init__(self, sample):
        self._sample = sample

    def hand_velocity_sample(self):
        return self._sample


class FakeJointStateSource:

    def __init__(self, sample):
        self._sample = sample

    def sample(self):
        return self._sample


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    value = PoseStamped()
    value.header.frame_id = "body"
    value.pose.position.x = x
    value.pose.position.y = y
    value.pose.position.z = z
    value.pose.orientation.z = math.sin(yaw * 0.5)
    value.pose.orientation.w = math.cos(yaw * 0.5)
    return value


def plan():
    return SimpleNamespace(
        linear_speed_mps=0.05,
        direction_frame="body",
        direction_x=1.0,
        direction_y=0.0,
        direction_z=0.0,
        current_hand=pose(0.0),
        target_hand=pose(0.10, yaw=0.4),
    )


def force_sample():
    return SimpleNamespace(
        received_at=4.9,
        x_n=-3.0,
        y_n=2.0,
        z_n=1.0,
    )


def force_baseline():
    return SimpleNamespace(
        x_n=1.0,
        y_n=2.0,
        z_n=1.0,
    )


def force_delta():
    return SimpleNamespace(
        opposing_n=4.0,
        total_n=4.0,
        delta_x_n=-4.0,
        delta_y_n=0.0,
        delta_z_n=0.0,
    )


def joint_sample():
    joints = {
        name: SimpleNamespace(
            position_rad=index + 0.1,
            velocity_rad_s=index + 0.2,
            effort_nm=index + 0.3,
        )
        for index, name in enumerate(ARM_JOINT_NAMES)
    }
    return SimpleNamespace(
        received_at=4.8,
        joints=joints,
    )


def velocity_sample():
    return SimpleNamespace(
        received_at=4.85,
        linear_x_mps=0.01,
        linear_y_mps=0.02,
        linear_z_mps=0.03,
        angular_x_rad_s=0.1,
        angular_y_rad_s=0.2,
        angular_z_rad_s=0.3,
    )


def test_observation_captures_force_motion_and_joint_evidence():
    telemetry = ArmContactTelemetry(
        FakeArmStateSource(velocity_sample()),
        FakeJointStateSource(joint_sample()),
    )

    sequence = telemetry.begin_movement()
    observation = telemetry.observe(
        movement_sequence=sequence,
        observed_at=5.0,
        elapsed_sec=0.4,
        phase="moving",
        plan=plan(),
        force_sample=force_sample(),
        force_baseline=force_baseline(),
        force_delta=force_delta(),
        current_hand=pose(0.04, y=0.03, yaw=0.1),
    )

    assert observation.movement_sequence == 1
    assert observation.elapsed_sec == pytest.approx(0.4)
    assert observation.planned_linear_speed_mps == pytest.approx(0.05)
    assert observation.movement_direction == (1.0, 0.0, 0.0)
    assert observation.initial_translation_distance_m == pytest.approx(0.10)
    assert observation.current_hand_position_m == pytest.approx((0.04, 0.03, 0.0))
    assert observation.target_hand_position_m == pytest.approx((0.10, 0.0, 0.0))
    assert observation.position_error_m == pytest.approx((0.06 ** 2 + 0.03 ** 2) ** 0.5)
    assert observation.rotation_error_rad == pytest.approx(0.3)
    assert observation.forward_progress_m == pytest.approx(0.04)
    assert observation.off_axis_displacement_m == pytest.approx(0.03)
    assert observation.translation_progress_fraction == pytest.approx(
        1.0 - ((0.06 ** 2 + 0.03 ** 2) ** 0.5 / 0.10)
    )
    assert observation.opposing_force_delta_n == pytest.approx(4.0)
    assert observation.total_force_delta_n == pytest.approx(4.0)
    assert observation.hand_linear_velocity_mps == (0.01, 0.02, 0.03)
    assert observation.joint_names == tuple(ARM_JOINT_NAMES)
    assert observation.joint_positions_rad[0] == pytest.approx(0.1)
    assert observation.joint_velocities_rad_s[5] == pytest.approx(5.2)
    assert observation.joint_efforts_nm[2] == pytest.approx(2.3)
    assert telemetry.latest_observation() is observation
    assert telemetry.observation_count == 1


def test_missing_optional_motion_samples_do_not_block_force_observation():
    telemetry = ArmContactTelemetry(
        FakeArmStateSource(None),
        FakeJointStateSource(None),
    )

    observation = telemetry.observe(
        movement_sequence=telemetry.begin_movement(),
        observed_at=5.0,
        elapsed_sec=0.2,
        phase="moving",
        plan=plan(),
        force_sample=force_sample(),
        force_baseline=force_baseline(),
        force_delta=force_delta(),
    )

    assert observation.current_hand_position_m is None
    assert observation.current_hand_orientation_xyzw is None
    assert observation.position_error_m is None
    assert observation.rotation_error_rad is None
    assert observation.forward_progress_m is None
    assert observation.off_axis_displacement_m is None
    assert observation.translation_progress_fraction is None
    assert observation.target_hand_position_m == pytest.approx((0.10, 0.0, 0.0))
    assert observation.hand_velocity_received_at is None
    assert observation.hand_linear_velocity_mps is None
    assert observation.joint_state_received_at is None
    assert observation.joint_positions_rad is None
    assert observation.joint_velocities_rad_s is None
    assert observation.joint_efforts_nm is None
    assert telemetry.observation_count == 1


def test_raw_logging_is_disabled_by_default(tmp_path):
    telemetry = ArmContactTelemetry(
        FakeArmStateSource(None),
        FakeJointStateSource(None),
        raw_log_root=tmp_path,
    )

    telemetry.observe(
        movement_sequence=telemetry.begin_movement(),
        observed_at=5.0,
        elapsed_sec=0.1,
        phase="moving",
        plan=plan(),
        force_sample=force_sample(),
        force_baseline=force_baseline(),
        force_delta=force_delta(),
    )

    assert telemetry.raw_log_path is None
    assert not list(tmp_path.iterdir())


def test_raw_logging_writes_self_describing_jsonl(tmp_path):
    telemetry = ArmContactTelemetry(
        FakeArmStateSource(velocity_sample()),
        FakeJointStateSource(joint_sample()),
        raw_logging_enabled=True,
        raw_log_root=tmp_path,
    )

    telemetry.observe(
        movement_sequence=telemetry.begin_movement(),
        observed_at=5.0,
        elapsed_sec=0.1,
        phase="moving",
        plan=plan(),
        force_sample=force_sample(),
        force_baseline=force_baseline(),
        force_delta=force_delta(),
    )
    telemetry.close()

    log_path = telemetry.raw_log_path
    assert log_path is not None
    payload = json.loads(log_path.read_text(encoding="utf-8").strip())
    assert payload["joint_names"] == list(ARM_JOINT_NAMES)
    assert payload["opposing_force_delta_n"] == pytest.approx(4.0)
    assert payload["target_hand_position_m"] == pytest.approx([0.10, 0.0, 0.0])
    assert "translation_progress_fraction" in payload
    assert payload["joint_efforts_nm"][0] == pytest.approx(0.3)


def test_raw_logging_failure_is_fail_open(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("not a directory", encoding="utf-8")
    telemetry = ArmContactTelemetry(
        FakeArmStateSource(None),
        FakeJointStateSource(None),
        raw_logging_enabled=True,
        raw_log_root=blocker / "contact_telemetry",
    )

    observation = telemetry.observe(
        movement_sequence=telemetry.begin_movement(),
        observed_at=5.0,
        elapsed_sec=0.1,
        phase="moving",
        plan=plan(),
        force_sample=force_sample(),
        force_baseline=force_baseline(),
        force_delta=force_delta(),
    )

    assert observation is telemetry.latest_observation()
    assert telemetry.observation_count == 1
    assert telemetry.last_error
    assert not telemetry.raw_logging_enabled
