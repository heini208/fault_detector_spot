"""Tests for authoritative probe surface-verification state."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupDraft,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    ProbeSurfaceVerificationSession,
    SurfaceVerificationPolicy,
    SurfaceVerificationState,
)


REQUEST_ID = "12345678-1234-5678-1234-567812345678"


def _session():
    return ProbeSurfaceVerificationSession(
        request_id=REQUEST_ID,
        target_distance_m=0.05,
        maximum_cumulative_correction_m=0.08,
        policy=SurfaceVerificationPolicy(),
    )


def test_clear_geometry_discards_surface_verification():
    draft = ProbeSetupDraft(context=object())
    draft.surface_verification = _session()

    draft.clear_geometry()

    assert draft.surface_verification is None


def test_state_adapter_publishes_surface_verification_measurement():
    session = _session()
    session.measured_distance_m = 0.061
    session.error_m = 0.011
    session.last_correction_m = 0.011
    session.cumulative_correction_m = 0.011
    session.iteration_count = 2
    session.state = SurfaceVerificationState.SETTLING
    session.recovery_required = True
    message = ProbeSetupState()
    snapshot = SimpleNamespace(surface_verification=session)

    ProbeSetupStateAdapter._write_surface_verification(message, snapshot)

    assert message.surface_verification_active
    assert (
        message.surface_verification_state
        == ProbeSetupState.SURFACE_VERIFICATION_SETTLING
    )
    assert message.surface_verification_request_id == REQUEST_ID
    assert message.has_surface_distance_measurement
    assert message.measured_surface_distance_m == pytest.approx(0.061)
    assert message.surface_distance_error_m == pytest.approx(0.011)
    assert message.surface_distance_tolerance_m == pytest.approx(0.005)
    assert message.has_surface_correction
    assert message.last_surface_correction_m == pytest.approx(0.011)
    assert message.cumulative_surface_correction_m == pytest.approx(0.011)
    assert message.surface_verification_iteration == 2
    assert message.surface_recovery_required


@pytest.mark.parametrize(
    ("internal", "public"),
    (
        (
            SurfaceVerificationState.SAMPLING,
            ProbeSetupState.SURFACE_VERIFICATION_SAMPLING,
        ),
        (
            SurfaceVerificationState.MOVING,
            ProbeSetupState.SURFACE_VERIFICATION_MOVING,
        ),
        (
            SurfaceVerificationState.CONVERGED,
            ProbeSetupState.SURFACE_VERIFICATION_CONVERGED,
        ),
        (
            SurfaceVerificationState.FAILED,
            ProbeSetupState.SURFACE_VERIFICATION_FAILED,
        ),
        (
            SurfaceVerificationState.CANCELLED,
            ProbeSetupState.SURFACE_VERIFICATION_CANCELLED,
        ),
        (
            SurfaceVerificationState.RECOVERY_REQUIRED,
            ProbeSetupState.SURFACE_VERIFICATION_RECOVERY_REQUIRED,
        ),
    ),
)
def test_state_adapter_maps_surface_verification_states(internal, public):
    session = _session()
    session.state = internal
    message = ProbeSetupState()
    snapshot = SimpleNamespace(surface_verification=session)

    ProbeSetupStateAdapter._write_surface_verification(message, snapshot)

    assert message.surface_verification_state == public

