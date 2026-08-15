"""Boundary tests for server-owned surface-verification execution."""

import inspect
from types import SimpleNamespace

from fault_detector_spot.application.api.probe_setup_motion_api import (
    ProbeSetupMotionApi,
)
from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.application.coordinators.probe_surface_verification_runner import (
    ProbeSurfaceVerificationRunner,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)


class _Coordinator:
    def __init__(self):
        self.evaluated_sample_counts = []

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
        self.evaluated_sample_counts.append(len(samples))
        if len(samples) < 5:
            raise ValueError("Need five accumulated surface samples")
        return "decision", "snapshot"


class _StateSource:
    def __init__(self):
        self.calls = 0
        self.maximum_ages = []

    def surface_distance_samples(
        self,
        sensor_id,
        receipt_not_before=0.0,
        maximum_age_sec=0.5,
    ):
        self.calls += 1
        self.maximum_ages.append(maximum_age_sec)
        return tuple(range(min(2 + self.calls, 5)))

    def current_probe_pose_object(self, tag_id, sensor_id):
        return object()


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


def test_sampling_uses_multi_second_window_until_enough_samples_arrive():
    coordinator = _Coordinator()
    state_source = _StateSource()
    runner = ProbeSurfaceVerificationRunner(
        coordinator,
        state_source,
        sample_timeout_sec=3.0,
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

    result = runner._wait_for_evaluation(
        snapshot,
        request_id="request",
        receipt_not_before=10.0,
        cancel_requested=lambda: False,
    )

    assert result == ("decision", "snapshot")
    assert state_source.calls == 3
    assert state_source.maximum_ages == [3.0, 3.0, 3.0]
    assert coordinator.evaluated_sample_counts == [3, 4, 5]
