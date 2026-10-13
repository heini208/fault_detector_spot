"""Post-settle acquisition must recover from noise without admitting bad depth."""

from types import SimpleNamespace
from dataclasses import replace
import math

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome, ArmMovementUpdate,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MoveCloseToSurfaceConfig, MoveCloseToSurfaceExecution, MoveCloseToSurfaceOutcome,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import SensorTipPoseSample
from test_live_surface_distance import distance_sample, fitted_plane
from test_move_close_to_surface_recovery import FakeExecutor, pose, stamped
from fault_detector_spot.shared.geometry.models import Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler


def sampling_execution():
    state = SimpleNamespace(now=10.0, samples=(), requests=[])

    def read_samples(_reference_pose, _expected_distance, **options):
        state.requests.append(options)
        return state.samples

    executor = FakeExecutor()
    action = MoveCloseToSurfaceExecution(
        executor,
        SimpleNamespace(
            active_attachment=lambda: ("probe", 1),
            sensor_tip_distance_samples=read_samples,
            sensor_tip_pose_snapshot=lambda *args, **kwargs: (pose(), pose(), pose()),
            pose_acquisition_cutoff=lambda delay_sec=0.0: state.now + delay_sec,
            sensor_tip_pose_sample=lambda *args, **kwargs: SensorTipPoseSample(
                state.now, pose(), pose(), pose(),
            ),
            observation_view=lambda *args, **kwargs: SimpleNamespace(visible=True),
        ),
        MoveCloseToSurfaceConfig(), monotonic_clock=lambda: state.now,
    )
    action._current_probe_pose = lambda: pose()
    action.start(MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE, stamp=object(), target_surface_distance_m=0.03,
    ))
    return action, state, executor


def samples(start, distances):
    return tuple(
        distance_sample(value, start + index * 0.3, surface_plane_probe=fitted_plane(value))
        for index, value in enumerate(distances)
    )


def poll_at(action, state, *times):
    for instant in times:
        state.now = instant
        outcome = action.poll()
    return outcome


def settle_initial_pose(action, state):
    assert poll_at(action, state, 10.5, 10.7, 10.9) is MoveCloseToSurfaceOutcome.RUNNING


def test_initial_settle_excludes_earlier_frames_without_using_sampling_budget():
    action, state, executor = sampling_execution()
    state.now = 10.49

    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
    assert not state.requests
    assert not executor.guarded_calls

    state.now = 10.5
    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
    assert not state.requests
    assert not executor.guarded_calls
    assert poll_at(action, state, 10.7, 10.9) is MoveCloseToSurfaceOutcome.RUNNING
    assert state.requests[-1]["receipt_not_before"] == pytest.approx(10.5)
    assert state.requests[-1]["maximum_latest_age_sec"] == pytest.approx(0.5)
    assert action._measurement._phase_start == pytest.approx(10.9)


def test_later_stable_window_can_start_approach_after_old_three_second_budget():
    action, state, executor = sampling_execution()
    settle_initial_pose(action, state)
    noisy = samples(10.6, [0.08, 0.10, 0.08, 0.10, 0.08])
    state.samples = noisy
    state.now = 11.8

    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
    assert "unstable" in action.feedback_message
    assert not executor.guarded_calls

    state.samples = noisy + samples(13.6, [0.079, 0.080, 0.081, 0.080, 0.079])
    assert poll_at(action, state, 14.8, 15.0, 15.2) is MoveCloseToSurfaceOutcome.RUNNING
    assert action._phase == "approach"
    assert len(executor.guarded_calls) == 1
    target = executor.guarded_calls[0][0][0].pose
    assert target.position.x == pytest.approx(0.05)


def test_persistent_instability_still_fails_without_submitting_arm_motion():
    action, state, executor = sampling_execution()
    settle_initial_pose(action, state)
    state.samples = samples(15.3, [0.08, 0.10, 0.08, 0.10, 0.08])
    state.now = 16.89
    assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING

    state.now = 16.91
    assert action.poll() is MoveCloseToSurfaceOutcome.FAILURE
    assert "unstable" in action.feedback_message
    assert "20.00 mm" in action.feedback_message
    assert "5.00 mm" in action.feedback_message
    assert not executor.guarded_calls
    assert not executor.probe_calls


def test_stale_source_cannot_reuse_accumulated_samples_to_start_motion():
    action, state, executor = sampling_execution()
    settle_initial_pose(action, state)
    action._measurement._samples = {
        sample.stamp_seconds: sample
        for sample in samples(10.6, [0.08] * 5)
    }

    def stale(*_args, **_options):
        raise ValueError("Newest valid surface-distance frame is stale")

    action.surface_source.sensor_tip_distance_samples = stale
    state.now = 11.0
    action.poll()
    state.now = 16.91
    assert action.poll() is MoveCloseToSurfaceOutcome.FAILURE
    assert "stale" in action.feedback_message
    assert not executor.guarded_calls


@pytest.mark.parametrize("settled", [False, True])
@pytest.mark.parametrize("change", [
    "body_transform_unavailable", "body_position", "body_orientation",
    "attachment_revision", "attachment_sensor", "attachment_unavailable",
])
def test_invalid_measurement_reference_prohibits_approach_recovery(change, settled):
    action, state, executor = sampling_execution()
    settle_initial_pose(action, state)
    state.samples = samples(10.6, [0.08] * 5)
    assert poll_at(action, state, 11.8, 12.0, 12.2) is MoveCloseToSurfaceOutcome.RUNNING
    assert action._phase == "approach"
    assert len(executor.guarded_calls) == 1

    if settled:
        executor.active = False
        executor.poll = lambda: ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "completed")
        assert action.poll() is MoveCloseToSurfaceOutcome.RUNNING
        assert action._phase == "settling"

    def unavailable(*_args, **_kwargs):
        raise ValueError("measurement reference unavailable")

    if change == "body_transform_unavailable":
        executor.probe_motion_planner.current_pose = unavailable
    elif change in {"body_position", "body_orientation"}:
        body = (pose(x=0.011) if change == "body_position" else pose(
            orientation=quaternion_from_euler("z", math.radians(2.1)),
        ))
        executor.probe_motion_planner.current_pose = lambda *_args: stamped(body)
    elif change == "attachment_revision":
        action.surface_source.active_attachment = lambda: ("probe", 2)
    elif change == "attachment_sensor":
        action.surface_source.active_attachment = lambda: ("different_probe", 1)
    else:
        action.surface_source.active_attachment = unavailable

    assert action.poll() is MoveCloseToSurfaceOutcome.FAILURE
    assert action.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED
    assert not action.active
    assert len(executor.guarded_calls) == 1
    assert not executor.probe_calls
    assert executor.cancel_count == (0 if settled else 1)


def test_consistently_misaligned_measurement_cannot_start_surface_approach():
    action, state, executor = sampling_execution()
    settle_initial_pose(action, state)
    angle = math.radians(9.0)
    plane = replace(fitted_plane(0.08), normal=Vector3Data(
        -math.cos(angle), math.sin(angle), 0.0,
    ))
    distance = plane.signed_distance(Vector3Data.zero())
    state.samples = tuple(
        distance_sample(distance, 10.6 + index * 0.3, surface_plane_probe=plane)
        for index in range(5)
    )
    assert poll_at(action, state, 11.8, 12.0, 12.2) is MoveCloseToSurfaceOutcome.FAILURE
    assert action._measurement.result is not None
    assert "not aligned" in action.feedback_message
    assert "9.00 deg > 5.00 deg" in action.feedback_message
    assert not executor.guarded_calls
    assert not executor.probe_calls
