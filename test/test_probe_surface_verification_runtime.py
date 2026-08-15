"""Boundary tests for server-owned surface-verification execution."""

import inspect
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.api.probe_setup_motion_api import (
    ProbeSetupMotionApi,
)
from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.application.coordinators import (
    probe_surface_verification_runner as verification_runner,
)
from fault_detector_spot.inspection.execution.probe_surface_approach import (
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    SurfaceDistanceSample,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)


def _pose(x=0.0, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


def _sample(stamp):
    return SurfaceDistanceSample(
        distance_m=0.20,
        stamp_seconds=stamp,
        frame_id="hand_depth",
        sample_count=20,
        valid_pixel_ratio=0.8,
        spread_m=0.001,
        source_region=ImageRegion(x=0, y=0, width=2, height=2),
    )


class _Coordinator:
    def __init__(self):
        self.evaluated = []

    def add_motion_status_listener(self, listener):
        self.listener = listener

    def context(self, context_id, client_id):
        return object()

    def evaluate_surface_verification(
        self,
        context,
        request_id,
        samples,
        achieved,
    ):
        self.evaluated.append(tuple(sample.stamp_seconds for sample in samples))
        if len(samples) < 5:
            raise ValueError("Need five samples")
        return SimpleNamespace(resample_required=False), "snapshot"


class _StateSource:
    def __init__(self):
        self.calls = 0
        self.minimum_samples = []

    def surface_distance_samples(
        self,
        sensor_id,
        receipt_not_before=0.0,
        maximum_age_sec=0.5,
        minimum_samples=3,
    ):
        self.minimum_samples.append(minimum_samples)
        self.calls += 1
        return (_sample(10.0 + self.calls * 0.25),)

    def current_probe_pose_object(self, tag_id, sensor_id):
        return _pose()


def test_surface_distance_correction_is_internal_relative_motion():
    motion = ProbeMotionRequest(
        kind=ProbeMotionKind.ADJUST_PROBE_DISTANCE,
    )

    assert motion.relative
    assert (
        ProbeRefinementController.motion_stage(motion.kind)
        is RefinementStage.PROBE
    )


def test_public_primitive_action_cannot_request_surface_correction():
    source = inspect.getsource(ProbeSetupMotionApi._motion_request)

    assert "ADJUST_PROBE_DISTANCE" not in source


def test_runner_defaults_to_three_second_sampling():
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        _Coordinator(),
        _StateSource(),
    )

    assert runner.sample_timeout_sec == pytest.approx(3.0)


def test_runner_accumulates_individually_fresh_samples(monkeypatch):
    coordinator = _Coordinator()
    state_source = _StateSource()
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        coordinator,
        state_source,
        poll_sec=0.001,
    )
    snapshot = SimpleNamespace(
        selected_sensor_id="hall_probe",
        selected_reference_tag_id=2,
        context=SimpleNamespace(
            context_id="context",
            client_id="client",
        ),
    )

    result = runner._wait_for_initial_evaluation(
        snapshot,
        request_id="request",
        receipt_not_before=1.0,
        cancel_requested=lambda: False,
    )

    assert result[0].resample_required is False
    assert state_source.calls == 5
    assert state_source.minimum_samples == [1, 1, 1, 1, 1]
    assert coordinator.evaluated[-1] == (
        10.25,
        10.5,
        10.75,
        11.0,
        11.25,
    )


def test_progress_guard_rejects_probable_obstruction():
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        _Coordinator(),
        _StateSource(),
    )
    plan = freeze_probe_surface_approach(
        current_probe_pose_execution=_pose(),
        measured_initial_distance_m=0.20,
        target_distance_m=0.05,
        maximum_travel_m=0.16,
    )
    evaluation = SimpleNamespace(
        lateral_offset_m=0.0,
        axis_error_rad=0.0,
    )

    with pytest.raises(RuntimeError, match="Possible obstruction or contact"):
        runner._validate_progress(
            plan,
            previous_pose=_pose(),
            current_pose=_pose(x=0.001),
            requested_step_m=0.01,
            evaluation=evaluation,
        )
