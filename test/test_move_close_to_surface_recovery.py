"""Safety regression tests for close-surface recovery."""

import math

import pytest

import fault_detector_spot.inspection.execution.move_close_to_surface_operation as move_close_module
from fault_detector_spot.inspection.execution.move_close_to_surface_operation import (
    MoveCloseToSurfaceOperation,
    MoveCloseToSurfaceStatus,
)
from fault_detector_spot.inspection.execution.probe_surface_approach import (
    ProbeSurfaceApproachEvaluation,
)
from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
    rotation_distance_rad,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


def operation(**kwargs):
    return MoveCloseToSurfaceOperation(
        node=object(),
        state_source=object(),
        robot_command_client=object(),
        **kwargs,
    )


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


class RecoveryStateSource:
    def __init__(self, current_pose):
        self.current_pose = current_pose

    def current_hand_pose_execution(self):
        return self.current_pose


class ApproachStateSource:
    def __init__(self, probe_pose, hand_pose):
        self.probe_pose = probe_pose
        self.hand_pose = hand_pose

    def current_probe_pose_execution(self, _sensor_id):
        return self.probe_pose

    def current_hand_pose_execution(self):
        return self.hand_pose


class FrozenPlan:
    def inward_direction(self):
        return Vector3Data(x=1.0, y=0.0, z=0.0)


def test_diagonal_recovery_step_is_bounded_by_euclidean_distance():
    action = operation(recovery_step_m=0.040)
    action._state_source = RecoveryStateSource(pose())
    action._recovery_hand_pose = pose(x=0.040, y=0.040)
    action._phase = "recovery_prepare"
    action._recovery_steps = 0

    sent = []
    action._send_pose_goal = lambda target, duration: sent.append(target)

    result = action._update_recovery_prepare()

    assert result is MoveCloseToSurfaceStatus.RUNNING
    assert len(sent) == 1
    target = sent[0]
    distance = math.sqrt(
        target.position.x ** 2
        + target.position.y ** 2
        + target.position.z ** 2
    )
    assert distance == pytest.approx(0.040)
    assert target.position.x == pytest.approx(
        0.040 / math.sqrt(2.0)
    )
    assert target.position.y == pytest.approx(
        0.040 / math.sqrt(2.0)
    )


def test_recovery_configuration_rejects_more_than_40_mm():
    with pytest.raises(ValueError, match="0.040 m"):
        operation(recovery_step_m=0.0401)


def test_rotation_distance_is_sign_invariant():
    first = QuaternionData(
        x=0.0,
        y=0.0,
        z=0.0,
        w=1.0,
    )
    second = QuaternionData(
        x=-0.0,
        y=-0.0,
        z=-0.0,
        w=-1.0,
    )

    assert rotation_distance_rad(first, second) == pytest.approx(0.0)


def test_rotation_distance_matches_known_angle():
    first = QuaternionData.identity()
    second = quaternion_from_euler("z", math.radians(7.5))

    assert rotation_distance_rad(first, second) == pytest.approx(
        math.radians(7.5)
    )


def test_trajectory_guard_measures_lateral_drift_per_step():
    action = operation(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose(x=0.050, y=0.0100)
    action._requested_step_m = 0.010

    achieved, lateral = action._validate_step_motion(
        pose(x=0.060, y=0.0102)
    )

    assert achieved == pytest.approx(0.010)
    assert lateral == pytest.approx(0.0002)


def test_trajectory_guard_rejects_actual_per_step_lateral_drift():
    action = operation(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose()
    action._requested_step_m = 0.010

    with pytest.raises(RuntimeError, match="per-step lateral drift"):
        action._validate_step_motion(
            pose(x=0.010, y=0.0102)
        )


def test_approach_step_commands_original_aligned_orientation(monkeypatch):
    aligned_orientation = quaternion_from_euler("z", math.radians(2.0))
    drifted_orientation = quaternion_from_euler("z", math.radians(4.0))
    action = operation()
    action._sensor_id = "sensor"
    action._plan = FrozenPlan()
    action._recovery_hand_pose = pose(orientation=aligned_orientation)
    action._state_source = ApproachStateSource(
        probe_pose=pose(orientation=drifted_orientation),
        hand_pose=pose(x=0.100, orientation=drifted_orientation),
    )
    action._approach_steps = 0

    evaluation = ProbeSurfaceApproachEvaluation(
        estimated_distance_m=0.100,
        remaining_inward_travel_m=0.010,
        traveled_inward_m=0.0,
        lateral_offset_m=0.0,
        axis_error_rad=0.0,
        requested_step_m=0.010,
        reached=False,
    )
    monkeypatch.setattr(
        move_close_module,
        "evaluate_probe_surface_approach",
        lambda *args, **kwargs: evaluation,
    )

    sent = []
    action._send_pose_goal = lambda target, duration: sent.append(target)

    result = action._prepare_next_approach_step()

    assert result is MoveCloseToSurfaceStatus.RUNNING
    assert len(sent) == 1
    assert sent[0].position.x == pytest.approx(0.110)
    assert rotation_distance_rad(
        sent[0].orientation,
        aligned_orientation,
    ) == pytest.approx(0.0)
    assert rotation_distance_rad(
        sent[0].orientation,
        drifted_orientation,
    ) > 0.0
