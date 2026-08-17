"""Boundary tests for server-owned surface-verification execution."""

import inspect
import time
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
from fault_detector_spot.inspection.sensing.end_effector_force import (
    EndEffectorForceSample,
    ForceBaseline,
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


def _force_sample(force_x, receipt_time, stamp_seconds):
    return EndEffectorForceSample(
        force_hand=Vector3Data(x=force_x, y=0.0, z=0.0),
        stamp_seconds=stamp_seconds,
        receipt_time=receipt_time,
        frame_id="hand",
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
        self.evaluated.append(
            tuple(sample.stamp_seconds for sample in samples)
        )
        if len(samples) < 5:
            raise ValueError("Need five samples")
        return SimpleNamespace(resample_required=False), "snapshot"


class _StateSource:
    def __init__(self):
        self.calls = 0
        self.minimum_samples = []
        self.sensor_ids = []

    def surface_distance_samples(
        self,
        sensor_id,
        receipt_not_before=0.0,
        maximum_age_sec=0.5,
        minimum_samples=3,
    ):
        self.sensor_ids.append(sensor_id)
        self.minimum_samples.append(minimum_samples)
        self.calls += 1
        return (_sample(10.0 + self.calls * 0.25),)

    def current_probe_pose_object(self, tag_id, sensor_id):
        self.sensor_ids.append(sensor_id)
        return _pose()


class _ForceStateSource(_StateSource):
    def __init__(self, samples):
        super().__init__()
        self.force_samples = list(samples)

    def latest_end_effector_force(self):
        return self.force_samples.pop(0)


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


def test_runner_defaults_to_three_contact_retries():
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        _Coordinator(),
        _StateSource(),
    )

    assert runner.maximum_contact_retries == 3
    assert runner.force_contact_consecutive_samples == 2


def test_runner_accumulates_samples_for_reserved_active_sensor(monkeypatch):
    coordinator = _Coordinator()
    state_source = _StateSource()
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        coordinator,
        state_source,
        poll_sec=0.001,
    )
    snapshot = SimpleNamespace(
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
        sensor_id="active_probe",
    )

    assert result[0].resample_required is False
    assert state_source.calls == 5
    assert state_source.minimum_samples == [1, 1, 1, 1, 1]
    assert set(state_source.sensor_ids) == {"active_probe"}
    assert coordinator.evaluated[-1] == (
        10.25,
        10.5,
        10.75,
        11.0,
        11.25,
    )


def test_force_guard_requires_two_consecutive_contact_samples():
    now = time.monotonic()
    state_source = _ForceStateSource((
        _force_sample(6.0, now + 0.01, 10.0),
        _force_sample(6.5, now + 0.02, 10.1),
    ))
    runner = verification_runner.ProbeSurfaceVerificationRunner(
        _Coordinator(),
        state_source,
    )
    baseline = ForceBaseline(
        force_hand=Vector3Data.zero(),
        frame_id="hand",
        sample_count=10,
        sample_span_sec=0.6,
        maximum_component_span_n=0.1,
    )
    guard = verification_runner._ForceGuardState(
        last_receipt_time=now,
    )
    axis = Vector3Data(x=1.0, y=0.0, z=0.0)

    first = runner._check_force_guard(baseline, axis, guard)
    second = runner._check_force_guard(baseline, axis, guard)

    assert first is None
    assert isinstance(second, verification_runner._ProbableContact)
    assert "possible contact" in str(second).lower()


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
