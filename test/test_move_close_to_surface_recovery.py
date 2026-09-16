"""Safety regression tests for close-surface recovery."""

import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
    rotation_distance_rad,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MoveCloseToSurfaceBehaviour,
    MoveCloseToSurfaceConfig,
)
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def stamped(data):
    value = PoseStamped()
    value.header.frame_id = "body"
    value.pose = pose_data_to_pose(data)
    return value


class FakePlanner:
    def __init__(self, hand_pose=None):
        self.hand_pose = hand_pose or pose()

    def current_hand_pose(self, _frame):
        return stamped(self.hand_pose)


class FakeExecutor:
    def __init__(self, hand_pose=None):
        self.active = False
        self.speed_policy = SimpleNamespace(
            default_speed=SimpleNamespace(angular_speed_rad_s=0.5)
        )
        self.probe_motion_planner = FakePlanner(hand_pose)
        self.guarded_calls = []
        self.probe_calls = []
        self.cancel_count = 0

    def guarded_probe(self, *args, **kwargs):
        self.guarded_calls.append((args, kwargs))
        self.active = True
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "running")

    def probe(self, *args, **kwargs):
        self.probe_calls.append((args, kwargs))
        self.active = True
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "recovering")

    def cancel(self):
        self.cancel_count += 1
        self.active = False


class FrozenPlan:
    def inward_direction(self):
        return Vector3Data(x=1.0, y=0.0, z=0.0)


def behaviour(executor=None, **changes):
    action = MoveCloseToSurfaceBehaviour(
        surface_source=object(),
        config=MoveCloseToSurfaceConfig(**changes),
    )
    action.executor = executor or FakeExecutor()
    action._command = SimpleNamespace(target_surface_distance_m=0.03)
    action._phase = "approach"
    action._started = True
    return action


def test_contact_mode_returns_success_after_shared_snap_retreat():
    action = behaviour()
    action._command = SimpleNamespace(target_surface_distance_m=0.0)

    result = action._handle_approach_update(
        ArmMovementUpdate(
            ArmMovementOutcome.CONTACT,
            "contact; retreated 0.0100 m",
        )
    )

    assert result is Status.SUCCESS
    assert "snap retreat" in action.feedback_message


def test_nonzero_contact_starts_recovery_to_original_start():
    executor = FakeExecutor(hand_pose=pose(x=0.06))
    action = behaviour(executor=executor)
    action._recovery_hand_pose = pose(x=0.0)
    action._approach_steps = 1

    result = action._handle_approach_update(
        ArmMovementUpdate(
            ArmMovementOutcome.CONTACT,
            "contact; retreated 0.0100 m",
        )
    )

    assert result is Status.RUNNING
    assert len(executor.probe_calls) == 1
    target = executor.probe_calls[0][0][0]
    assert target.pose.position.x == pytest.approx(0.02)
    assert "original pre-approach" in action.feedback_message


def test_diagonal_recovery_step_is_bounded_by_euclidean_distance():
    executor = FakeExecutor(hand_pose=pose())
    action = behaviour(executor=executor, recovery_step_m=0.040)
    action._recovery_hand_pose = pose(x=0.040, y=0.040)
    action._phase = "recovery_prepare"

    result = action._update_recovery_prepare()

    assert result is Status.RUNNING
    target = executor.probe_calls[0][0][0]
    distance = math.sqrt(
        target.pose.position.x ** 2
        + target.pose.position.y ** 2
        + target.pose.position.z ** 2
    )
    assert distance == pytest.approx(0.040)
    assert target.pose.position.x == pytest.approx(
        0.040 / math.sqrt(2.0)
    )
    assert target.pose.position.y == pytest.approx(
        0.040 / math.sqrt(2.0)
    )


def test_recovery_configuration_rejects_more_than_40_mm():
    with pytest.raises(ValueError, match="0.040 m"):
        behaviour(recovery_step_m=0.0401)


def test_rotation_distance_is_sign_invariant():
    first = QuaternionData(x=0.0, y=0.0, z=0.0, w=1.0)
    second = QuaternionData(x=-0.0, y=-0.0, z=-0.0, w=-1.0)

    assert rotation_distance_rad(first, second) == pytest.approx(0.0)


def test_rotation_distance_matches_known_angle():
    first = QuaternionData.identity()
    second = quaternion_from_euler("z", math.radians(7.5))

    assert rotation_distance_rad(first, second) == pytest.approx(
        math.radians(7.5)
    )


def test_endpoint_validation_measures_settled_lateral_error():
    action = behaviour(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose(x=0.050, y=0.0100)
    action._requested_step_m = 0.010

    achieved, lateral = action._validate_step_motion(
        pose(x=0.060, y=0.0102)
    )

    assert achieved == pytest.approx(0.010)
    assert lateral == pytest.approx(0.0002)


def test_endpoint_validation_rejects_excessive_settled_lateral_error():
    action = behaviour(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose()
    action._requested_step_m = 0.010

    with pytest.raises(RuntimeError, match="settled endpoint lateral error"):
        action._validate_step_motion(
            pose(x=0.010, y=0.0102)
        )
