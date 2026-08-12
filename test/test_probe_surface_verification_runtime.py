"""Boundary tests for server-owned surface-verification execution."""

import inspect

from fault_detector_spot.application.api.probe_setup_motion_api import (
    ProbeSetupMotionApi,
)
from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)


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
