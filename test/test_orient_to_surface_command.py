"""Focused regression tests for the orient-to-surface command."""

import inspect
import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.behaviours.orient_to_surface_behaviour import (
    OrientToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.commands.orient_to_surface_command import (
    OrientToSurfaceCommand,
)
from fault_detector_spot.ui.manipulation.controls import ManipulationControls


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=34))


def test_orientation_requests_image_time_tf_without_latest_fallback():
    calls = []
    def lookup(target, source, **kwargs):
        calls.append(kwargs)
        raise RuntimeError("historical transform unavailable")
    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._active = False
    executor.surface_source = SimpleNamespace(
        surface_normal=lambda **kwargs: SimpleNamespace(
            stamp_nanoseconds=12000000034,
            projected_point=SimpleNamespace(frame_id="camera"),
        )
    )
    executor.tf_listener = SimpleNamespace(lookup_a_tform_b=lookup)
    with pytest.raises(RuntimeError, match="historical transform unavailable"):
        executor._resolve_surface_orientation_target("hand")
    assert len(calls) == 1
    assert calls[0]["transform_time"].nanoseconds == 12000000034
    assert calls[0]["timeout_sec"] == 0.0


def test_orientation_allows_delayed_capture_time_transform():
    from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
    from fault_detector_spot.shared.geometry.models import (
        Vector3Data,
    )
    calls = []
    from tf2_ros import ExtrapolationException
    from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable
    now = [0.0]

    def lookup(target, source, **kwargs):
        calls.append(kwargs)
        # Model a capture-time transform arriving 604 ms after the image.
        assert kwargs["timeout_sec"] == 0.0
        if now[0] < 0.604:
            raise ExtrapolationException("extrapolation into the future")
        result = TransformStamped()
        result.transform.rotation.w = 1.0
        return result

    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor.surface_source = SimpleNamespace(
        surface_normal=lambda **kwargs: SimpleNamespace(
            stamp_nanoseconds=12000000034,
            projected_point=SimpleNamespace(frame_id="camera"),
            normal_camera=Vector3Data(x=-1.0, y=0.0, z=0.0),
        )
    )
    executor.tf_listener = SimpleNamespace(lookup_a_tform_b=lookup)
    executor._monotonic_clock = lambda: now[0]
    executor._active = False
    executor.guarded_probe = lambda builder, **kwargs: builder
    hand_to_probe = Pose()
    hand_to_probe.orientation.w = 1.0
    current = PoseStamped()
    current.pose.orientation.w = 1.0
    current.pose.position.x = 0.4
    executor.probe_motion_planner = SimpleNamespace(
        hand_to_probe_pose=lambda sensor: hand_to_probe,
        current_pose=lambda *args: current,
    )
    builder = executor.orient_to_surface("hand")
    with pytest.raises(MovementGeometryUnavailable):
        builder()
    # No refit or newer image may replace the retained observation on retry.
    executor.surface_source.surface_normal = (
        lambda **kwargs: pytest.fail("refitted image")
    )
    now[0] = 0.7
    target, sensor = builder()
    assert sensor == "hand"
    assert target.pose.position.x == 0.4
    assert target.pose.orientation.w == pytest.approx(1.0)
    assert len(calls) == 2
    assert all(call["transform_time"].nanoseconds == 12000000034 for call in calls)


def test_orientation_tf_retry_times_out_and_new_command_starts_fresh():
    from tf2_ros import ExtrapolationException
    from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable
    now = [0.0]
    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._monotonic_clock = lambda: now[0]
    executor._active = False
    executor.guarded_probe = lambda builder, **kwargs: builder
    executor.surface_source = SimpleNamespace(
        surface_normal=lambda **kwargs: SimpleNamespace(
            stamp_nanoseconds=12000000034,
            projected_point=SimpleNamespace(frame_id="camera"),
        )
    )
    def unavailable(*args, **kwargs):
        raise ExtrapolationException("TF still behind")
    executor.tf_listener = SimpleNamespace(lookup_a_tform_b=unavailable)
    builder = executor.orient_to_surface("hand")
    with pytest.raises(MovementGeometryUnavailable):
        builder()
    now[0] = 2.1
    with pytest.raises(RuntimeError, match="synchronization timed out"):
        builder()
    new_builder = executor.orient_to_surface("hand")
    with pytest.raises(MovementGeometryUnavailable):
        new_builder()


def test_surface_orientation_verification_starts_one_correction():
    from fault_detector_spot.shared.geometry.models import (
        Vector3Data,
    )
    from fault_detector_spot.manipulation.arm_movement_result import (
        ArmMovementOutcome,
        ArmMovementUpdate,
    )

    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._surface_orientation_sensor_id = "hand"
    executor._surface_orientation_verify_not_before = 4.0
    executor._surface_orientation_verify_deadline = 6.0
    executor._surface_orientation_corrections = 0
    executor._surface_orientation_force_threshold_n = 3.0
    executor._surface_orientation_speed = None
    executor._monotonic_clock = lambda: 5.0
    estimate = SimpleNamespace(normal_camera=Vector3Data(x=-1.0, y=0.0, z=0.0))
    executor.surface_source = SimpleNamespace(
        surface_normal=lambda **kwargs: estimate
    )
    executor._surface_orientation_error_rad = (
        lambda sensor_id, value: math.radians(7.0)
    )
    executor.probe_motion_planner = SimpleNamespace(
        build_plan=lambda builder, speed: (builder, speed)
    )
    marker = ArmMovementUpdate(ArmMovementOutcome.RUNNING, "correction")
    executor._begin_guarded_probe = lambda: marker

    update = executor._poll_surface_orientation_verification()

    assert update is marker
    assert executor._surface_orientation_corrections == 1
    assert executor._guarded_force_threshold_n == pytest.approx(3.0)
    target_builder, speed = executor._guarded_plan_builder()
    assert callable(target_builder)
    assert speed is None


def test_surface_orientation_full_frame_chain_preserves_probe_axis():
    from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
    from fault_detector_spot.shared.geometry.rotation import rotate_vector
    from fault_detector_spot.shared.geometry.models import (
        Vector3Data,
    )
    from fault_detector_spot.manipulation.probe_motion_planner import (
        ProbeMotionPlanner,
    )
    from fault_detector_spot.shared.geometry.transforms import compose_poses

    half_camera_yaw = math.radians(30.0) * 0.5
    camera_tf = TransformStamped()
    camera_tf.transform.rotation.z = math.sin(half_camera_yaw)
    camera_tf.transform.rotation.w = math.cos(half_camera_yaw)

    half_mount_roll = math.radians(20.0) * 0.5
    mounting = Pose()
    mounting.orientation.x = math.sin(half_mount_roll)
    mounting.orientation.w = math.cos(half_mount_roll)

    current = PoseStamped()
    current.header.frame_id = "body"
    current.pose.orientation.w = 1.0

    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor.surface_source = None
    executor.tf_listener = SimpleNamespace(
        lookup_a_tform_b=lambda *args, **kwargs: camera_tf
    )
    executor.probe_motion_planner = SimpleNamespace(
        hand_to_probe_pose=lambda sensor_id: mounting,
        current_pose=lambda *args: current,
    )
    estimate = SimpleNamespace(
        stamp_nanoseconds=12000000034,
        projected_point=SimpleNamespace(frame_id="camera"),
        normal_camera=Vector3Data(x=-1.0, y=0.0, z=0.0),
    )

    target_probe, _ = executor._resolve_surface_orientation_target(
        "hall_probe",
        estimate,
    )
    target_hand = ProbeMotionPlanner.probe_pose_to_hand_pose(
        target_probe,
        mounting,
    )
    reconstructed_probe = PoseStamped()
    reconstructed_probe.pose = compose_poses(
        target_hand.pose,
        mounting,
    )
    from fault_detector_spot.shared.geometry.models import (
        QuaternionData,
    )

    actual_axis = rotate_vector(
        QuaternionData(
            x=reconstructed_probe.pose.orientation.x,
            y=reconstructed_probe.pose.orientation.y,
            z=reconstructed_probe.pose.orientation.z,
            w=reconstructed_probe.pose.orientation.w,
        ),
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    expected_inward = Vector3Data(
        x=math.cos(math.radians(30.0)),
        y=math.sin(math.radians(30.0)),
        z=0.0,
    )
    assert actual_axis.x == pytest.approx(expected_inward.x, abs=1e-9)
    assert actual_axis.y == pytest.approx(expected_inward.y, abs=1e-9)
    assert actual_axis.z == pytest.approx(expected_inward.z, abs=1e-9)


def subscriber():
    result = CommandSubscriber()
    result.node = SimpleNamespace(get_clock=lambda: FakeClock())
    return result


def test_public_intent_maps_to_orient_to_surface_command():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_ORIENT_TO_SURFACE

    command = operational_intent_to_command(intent)

    assert command.command_id is CommandID.ORIENT_TO_SURFACE


def test_semantic_command_preserves_bound_motion_sensor():
    command = SemanticCommand(
        command_id=CommandID.ORIENT_TO_SURFACE,
        motion_sensor_id="hall_probe",
    )

    translated = subscriber().fire_command_sequence(command)

    assert len(translated) == 1
    assert isinstance(translated[0], OrientToSurfaceCommand)
    assert translated[0].motion_sensor_id == "hall_probe"


def test_orient_to_surface_requires_bound_motion_sensor():
    command = SemanticCommand(command_id=CommandID.ORIENT_TO_SURFACE)

    with pytest.raises(ValueError, match="active sensor geometry"):
        subscriber().fire_command_sequence(command)


def test_behaviour_only_dispatches_to_executor():
    behaviour = OrientToSurfaceBehaviour(name="OrientToSurfaceBehaviour")
    command = OrientToSurfaceCommand(
        CommandID.ORIENT_TO_SURFACE,
        Time(),
        "hall_probe",
    )
    marker = object()
    behaviour._last_command = lambda: command
    behaviour.executor = SimpleNamespace(
        orient_to_surface=lambda sensor_id, *, ignore_environment_collisions: (
            marker
            if sensor_id == "hall_probe" and not ignore_environment_collisions
            else None
        )
    )

    assert behaviour._start_operation() is marker


def test_surface_source_owns_live_depth_to_normal_resolution():
    source = inspect.getsource(ProbeSurfaceSource.surface_normal)

    assert "self.latest_hand_depth(" in source
    assert "project_reference_pixel(" in source
    assert "MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M" in source
    assert "estimate_surface_normal(" in source


def test_executor_owns_surface_orientation_calculation_and_guarded_move():
    start = inspect.getsource(ArmMovementExecutor.orient_to_surface)
    acquire = inspect.getsource(
        ArmMovementExecutor._surface_orientation_target_builder
    )
    resolve = inspect.getsource(
        ArmMovementExecutor._resolve_surface_orientation_target
    )
    verify = inspect.getsource(
        ArmMovementExecutor._poll_surface_orientation_verification
    )

    assert "self.guarded_probe(" in start
    assert "receipt_not_before=started_at" in start
    assert "SurfaceOrientationTarget(" in acquire
    assert "receipt_not_before=receipt_not_before" in acquire
    assert "surface_aligned_probe_orientation(" in resolve
    assert "sensor_probe_frame(sensor_id)" in resolve
    assert "receipt_not_before=receipt_not_before" in verify
    correction = inspect.getsource(ArmMovementExecutor._handle_surface_orientation_error)
    assert "SURFACE_ORIENTATION_MAX_ERROR_RAD" in correction
    assert "self._begin_guarded_probe()" in correction


def test_ui_button_dispatches_orient_to_surface_intent():
    source = inspect.getsource(
        ManipulationControls.handle_orient_to_surface
    )

    assert "INTENT_ORIENT_TO_SURFACE" in source
    assert "execute_basic_operation(intent)" in source
    assert "show_setup_unavailable" not in source


def test_orientation_sensing_budget_starts_after_readiness_and_tf_gets_own_budget():
    from tf2_ros import ExtrapolationException
    from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable
    now = [0.0]
    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._monotonic_clock = lambda: now[0]
    def sense(**kwargs):
        if now[0] < 15.0:
            raise ValueError("No fresh depth yet")
        return SimpleNamespace(stamp_nanoseconds=12000000034)
    executor.surface_source = SimpleNamespace(surface_normal=sense)
    def resolve(*args):
        if now[0] < 16.0:
            raise ExtrapolationException("TF behind")
        return "target"
    executor._resolve_surface_orientation_target = resolve
    builder = executor._surface_orientation_target_builder("hand", 0.0)
    now[0] = 10.0  # Arm readiness did not consume the sensing budget.
    with pytest.raises(MovementGeometryUnavailable):
        builder()
    now[0] = 15.0  # Delayed image arrives, then starts its own TF budget.
    with pytest.raises(MovementGeometryUnavailable):
        builder()
    now[0] = 16.1
    assert builder() == "target"


def test_verification_retains_post_motion_frame_while_waiting_for_tf():
    from tf2_ros import ExtrapolationException
    from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._surface_orientation_sensor_id = "hand"
    executor._surface_orientation_verify_not_before = 4.0
    executor._surface_orientation_verify_deadline = 10.0
    executor._surface_orientation_verify_estimate = None
    executor._surface_orientation_corrections = 0
    executor._surface_orientation_force_threshold_n = None
    executor._surface_orientation_speed = None
    now = [5.0]
    executor._monotonic_clock = lambda: now[0]
    estimate = object()
    executor.surface_source = SimpleNamespace(surface_normal=lambda **kwargs: estimate)
    def verify(sensor, selected):
        assert selected is estimate
        if now[0] < 5.5:
            raise ExtrapolationException("TF behind")
        return math.radians(7)
    executor._surface_orientation_error_rad = verify
    executor.probe_motion_planner = SimpleNamespace(build_plan=lambda *a: None)
    executor._begin_guarded_probe = lambda: "correction"
    assert executor._poll_surface_orientation_verification().outcome is ArmMovementOutcome.RUNNING
    executor.surface_source.surface_normal = lambda **kwargs: pytest.fail("replaced retained frame")
    now[0] = 5.7
    assert executor._poll_surface_orientation_verification() == "correction"
