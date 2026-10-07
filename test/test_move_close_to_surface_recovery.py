"""Safety regression tests for close-surface recovery."""

import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler,
    rotation_distance_rad,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MoveCloseToSurfaceConfig,
    MoveCloseToSurfaceExecution,
    MoveCloseToSurfaceOutcome,
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


def execution(executor=None, **changes):
    action = MoveCloseToSurfaceExecution(
        executor or FakeExecutor(),
        object(),
        MoveCloseToSurfaceConfig(**changes),
    )
    action._command = SimpleNamespace(target_surface_distance_m=0.03)
    action._phase = "approach"
    action._started = True
    return action


def test_execution_start_owns_lifecycle_and_cancel():
    executor = FakeExecutor()
    source = SimpleNamespace(active_attachment=lambda: ("probe", 1))
    action = MoveCloseToSurfaceExecution(
        executor,
        source,
        MoveCloseToSurfaceConfig(),
    )
    command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=object(),
        target_surface_distance_m=0.03,
    )

    result = action.start(command)

    assert result is MoveCloseToSurfaceOutcome.RUNNING
    assert action.active
    assert action._phase == "sampling"
    assert action.feedback_message == "Collecting initial live surface distance"

    executor.active = True
    action.cancel()

    assert executor.cancel_count == 1
    assert not action.active


@pytest.mark.parametrize("angle", [0.0, math.pi / 2])
def test_zero_distance_search_needs_no_reading_and_uses_sensitive_guard(angle):
    executor = FakeExecutor()
    # No surface_distance_samples method: contact search must never request it.
    source = SimpleNamespace(active_attachment=lambda: ("probe", 1))
    action = MoveCloseToSurfaceExecution(
        executor, source, MoveCloseToSurfaceConfig(),
    )
    start = pose(x=0.1, orientation=quaternion_from_euler("z", angle))
    action._current_probe_pose = lambda: start
    command = MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, stamp=object(),
        target_surface_distance_m=0.0,
    )

    assert action.start(command) is MoveCloseToSurfaceOutcome.RUNNING
    assert action._phase == "approach"
    args, options = executor.guarded_calls[0]
    target = args[0].pose
    assert target.position.x == pytest.approx(0.1 + 0.4 * math.cos(angle))
    assert target.position.y == pytest.approx(0.4 * math.sin(angle))
    assert target.orientation == pose_data_to_pose(start).orientation
    assert options["speed"].linear_speed_mps == pytest.approx(0.001)
    assert options["force_threshold_n"] == pytest.approx(2.0)
    assert options["cartesian_path"] is True
    assert action._plan is None

    result = action._handle_approach_update(
        ArmMovementUpdate(ArmMovementOutcome.CONTACT, "contact; snap retreat")
    )
    assert result is MoveCloseToSurfaceOutcome.SUCCESS
    assert executor.probe_calls == []


def test_contact_search_without_contact_recovers_at_planned_endpoint():
    executor = FakeExecutor()
    action = MoveCloseToSurfaceExecution(
        executor, SimpleNamespace(active_attachment=lambda: ("probe", 1)),
        MoveCloseToSurfaceConfig(maximum_travel_m=0.1),
    )
    action._current_probe_pose = lambda: executor.probe_motion_planner.hand_pose
    action.start(MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, stamp=object(),
        target_surface_distance_m=0.0,
    ))
    executor.probe_motion_planner.hand_pose = pose(x=0.1)
    action._handle_approach_update(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "endpoint reached")
    )
    action._clock = lambda: action._settle_deadline + 1.0

    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
    assert "planned endpoint" in action._recovery_detail
    assert len(executor.guarded_calls) == 1
    assert len(executor.probe_calls) == 1
    assert executor.probe_calls[0][0][0].pose.position.x == pytest.approx(0.0)


def test_incomplete_contact_plan_shortens_endpoint_before_single_motion():
    executor = FakeExecutor()
    action = MoveCloseToSurfaceExecution(
        executor, SimpleNamespace(active_attachment=lambda: ("probe", 1)),
        MoveCloseToSurfaceConfig(),
    )
    action._current_probe_pose = lambda: pose()
    action.start(MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, stamp=object(),
        target_surface_distance_m=0.0,
    ))
    # Planning failed before any trajectory was submitted to the robot.
    executor.active = False
    result = action._handle_approach_update(ArmMovementUpdate(
        ArmMovementOutcome.PLANNING_FAILED,
        "MoveIt Cartesian path is incomplete: fraction 0.925743 < 0.999000",
    ))
    assert result is MoveCloseToSurfaceOutcome.RUNNING
    assert action._phase == "contact_replan"
    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
    target, options = executor.guarded_calls[-1]
    assert target[0].pose.position.x == pytest.approx(0.32)
    assert options["speed"].linear_speed_mps == pytest.approx(0.001)
    assert options["force_threshold_n"] == pytest.approx(2.0)
    assert action._approach_steps == 1
    assert executor.probe_calls == []
    assert action._handle_approach_update(ArmMovementUpdate(
        ArmMovementOutcome.CONTACT, "contact; snap retreat",
    )) is MoveCloseToSurfaceOutcome.SUCCESS


@pytest.mark.parametrize("detail,retries", [
    ("MoveIt Cartesian path is incomplete: fraction 0.9", 8),
    ("MoveIt Cartesian planning error", 0),
])
def test_contact_replanning_is_bounded_and_only_for_incomplete_paths(detail, retries):
    action = execution()
    action._command = SimpleNamespace(target_surface_distance_m=0.0)
    action._requested_step_m = 0.4
    action._contact_plan_retries = retries
    result = action._handle_approach_update(ArmMovementUpdate(
        ArmMovementOutcome.PLANNING_FAILED, detail,
    ))
    assert result is MoveCloseToSurfaceOutcome.FAILURE
    assert detail in action.feedback_message


@pytest.mark.parametrize("verified", [False, True])
def test_sampling_rejects_misaligned_measured_plane_even_at_standoff(monkeypatch, verified):
    action = execution()
    action._clock = lambda: 4.0
    action._phase_started = 0.0
    action._sample_receipt_not_before = 0.0
    action._surface_samples = {}
    action._sensor_id = "probe"
    action._require_attachment_unchanged = lambda: None
    action._current_probe_pose = lambda: pose()
    action.surface_source = SimpleNamespace(surface_distance_samples=lambda *a, **k: ())
    monkeypatch.setattr(
        "fault_detector_spot.manipulation.move_close_to_surface_execution.aggregate_surface_distance_samples",
        lambda *a, **k: SimpleNamespace(
            verified=verified, distance_m=0.03,
            surface_plane_probe=SimpleNamespace(normal=Vector3Data(
                x=-math.cos(math.radians(20)), y=math.sin(math.radians(20)), z=0.0,
            )),
        ),
    )
    result = action._update_sampling()
    assert result is MoveCloseToSurfaceOutcome.FAILURE
    assert "20.00 deg" in action.feedback_message
    assert action.executor.guarded_calls == []


def test_contact_mode_returns_success_after_shared_snap_retreat():
    action = execution()
    action._command = SimpleNamespace(target_surface_distance_m=0.0)

    result = action._handle_approach_update(
        ArmMovementUpdate(
            ArmMovementOutcome.CONTACT,
            "contact; retreated 0.0100 m",
        )
    )

    assert result is MoveCloseToSurfaceOutcome.SUCCESS
    assert "snap retreat" in action.feedback_message


def test_nonzero_contact_starts_recovery_to_original_start():
    executor = FakeExecutor(hand_pose=pose(x=0.06))
    action = execution(executor=executor)
    action._recovery_hand_pose = pose(x=0.0)
    action._approach_steps = 1

    result = action._handle_approach_update(
        ArmMovementUpdate(
            ArmMovementOutcome.CONTACT,
            "contact; retreated 0.0100 m",
        )
    )

    assert result is MoveCloseToSurfaceOutcome.RUNNING
    assert len(executor.probe_calls) == 1
    target = executor.probe_calls[0][0][0]
    assert target.pose.position.x == pytest.approx(0.0)
    assert "original pre-approach" in action.feedback_message


def test_diagonal_recovery_targets_full_original_pose():
    executor = FakeExecutor(hand_pose=pose())
    action = execution(executor=executor)
    original = pose(x=0.12, y=0.08, orientation=quaternion_from_euler("z", 0.03))
    action._recovery_hand_pose = original

    result = action._update_recovery_prepare()

    assert result is MoveCloseToSurfaceOutcome.RUNNING
    assert len(executor.probe_calls) == 1
    target = executor.probe_calls[0][0][0]
    assert target.pose == pose_data_to_pose(original)
    assert executor.probe_calls[0][1]["speed"].linear_speed_mps == 0.020
    assert executor.probe_calls[0][1]["ignore_environment_collisions"] is True


@pytest.mark.parametrize("remaining", [0.0, 0.02])
def test_completed_recovery_never_starts_another_move(remaining):
    executor = FakeExecutor(hand_pose=pose(x=0.12))
    action = execution(executor=executor)
    action._recovery_hand_pose = pose()
    action._recovery_detail = "Unexpected contact"
    action._update_recovery_prepare()
    executor.probe_motion_planner.hand_pose = pose(x=remaining)
    executor.active = False  # A terminal successful movement releases the executor.
    action._handle_recovery_update(
        ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "completed")
    )

    result = action._update_recovery_prepare()

    assert result is MoveCloseToSurfaceOutcome.FAILURE
    assert len(executor.probe_calls) == 1
    assert "Unexpected contact" in action.feedback_message
    if remaining:
        assert "did not reach" in action.feedback_message


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
    action = execution(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose(x=0.050, y=0.0100)
    action._requested_step_m = 0.010

    achieved, lateral = action._validate_step_motion(
        pose(x=0.060, y=0.0102)
    )

    assert achieved == pytest.approx(0.010)
    assert lateral == pytest.approx(0.0002)


def test_endpoint_validation_rejects_excessive_settled_lateral_error():
    action = execution(maximum_lateral_drift_m=0.010)
    action._plan = FrozenPlan()
    action._previous_probe_pose = pose()
    action._requested_step_m = 0.010

    with pytest.raises(RuntimeError, match="settled endpoint lateral error"):
        action._validate_step_motion(
            pose(x=0.010, y=0.0102)
        )


@pytest.mark.parametrize("outcome", [
    ArmMovementOutcome.STOP_UNCONFIRMED, ArmMovementOutcome.RETREAT_FAILED,
])
@pytest.mark.parametrize("target", [0.0, 0.03])
def test_stop_failure_never_returns_to_start_in_either_mode(outcome, target):
    executor = FakeExecutor(hand_pose=pose(x=0.06))
    action = execution(executor=executor)
    action._command = SimpleNamespace(target_surface_distance_m=target)
    action._recovery_hand_pose = pose()
    action._approach_steps = 1
    result = action._handle_approach_update(ArmMovementUpdate(
        outcome, "Contact detected; stop or retreat failed",
    ))
    assert result is MoveCloseToSurfaceOutcome.FAILURE
    assert executor.probe_calls == []
    assert "no return" in action.feedback_message
    assert action.failure_outcome in {None, ArmMovementOutcome.STOP_UNCONFIRMED, ArmMovementOutcome.RETREAT_FAILED}


def test_contact_search_settings_are_independent_of_standoff_settings():
    executor = FakeExecutor()
    action = MoveCloseToSurfaceExecution(
        executor, SimpleNamespace(active_attachment=lambda: ("probe", 1)),
        MoveCloseToSurfaceConfig(
            approach_near_speed_mps=0.002, contact_search_retreat_distance_m=0.003,
        ),
    )
    action._current_probe_pose = lambda: pose()
    action.start(MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, stamp=object(),
        target_surface_distance_m=0.0,
    ))
    options = executor.guarded_calls[-1][1]
    assert options["speed"].linear_speed_mps == pytest.approx(0.001)
    assert options["force_threshold_n"] == pytest.approx(2.0)

    assert options["retreat_distance_m"] == pytest.approx(0.003)


def test_recovery_waits_for_cancelled_motion_to_stop_before_moving():
    executor = FakeExecutor(hand_pose=pose(x=0.06))
    executor.active = True
    executor.cancel = lambda: None  # Stop remains pending.
    executor.poll = lambda: ArmMovementUpdate(ArmMovementOutcome.RUNNING, "stopping")
    action = execution(executor=executor)
    action._approach_steps = 1
    action._recovery_hand_pose = pose()
    assert action._begin_recovery("planning failed") is MoveCloseToSurfaceOutcome.RUNNING
    assert not executor.probe_calls
    assert action.failure_outcome in {None, ArmMovementOutcome.STOP_UNCONFIRMED, ArmMovementOutcome.RETREAT_FAILED}
    executor.active = False  # Executor releases ownership only after confirmed stop.
    assert action._update_recovery_prepare() is MoveCloseToSurfaceOutcome.RUNNING
    assert len(executor.probe_calls) == 1
