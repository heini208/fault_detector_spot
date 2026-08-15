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
from fault_detector_spot.application.coordinators.probe_surface_verification_runner import (
    ProbeSurfaceVerificationRunner,
)
from fault_detector_spot.inspection.execution.probe_surface_approach import (
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)


def _pose(x=0.0, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


class _Coordinator:
    def add_motion_status_listener(self, listener):
        self.listener = listener


class _StateSource:
    pass


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


def test_runner_defaults_to_three_second_sampling_and_ten_mm_steps():
    runner = ProbeSurfaceVerificationRunner(
        _Coordinator(),
        _StateSource(),
    )

    assert runner.sample_timeout_sec == pytest.approx(3.0)


def test_progress_guard_rejects_probable_obstruction():
    runner = ProbeSurfaceVerificationRunner(
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
