"""Guards for removal of the obsolete setup surface-verification stack."""

import inspect
from pathlib import Path

from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.execution.probe_surface_runtime_state import (
    ProbeSurfaceRuntimeStateSource,
)
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupDraft,
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_legacy_surface_verification_modules_are_removed():
    obsolete = (
        ROOT / "application/coordinators/probe_surface_verification_controller.py",
        ROOT / "application/coordinators/probe_surface_verification_runner.py",
        ROOT / "inspection/setup/probe_surface_verification.py",
    )

    assert all(not path.exists() for path in obsolete)


def test_probe_setup_state_no_longer_contains_verification_session():
    assert "surface_verification" not in ProbeSetupDraft.__dataclass_fields__
    assert "surface_verification" not in ProbeSetupSnapshot.__dataclass_fields__


def test_probe_setup_coordinator_has_no_legacy_verification_api():
    source = inspect.getsource(ProbeSetupCoordinator)

    assert "surface_controller" not in source
    assert "surface_verification" not in source
    assert "ProbeSurfaceVerificationRunner" not in source


def test_probe_setup_state_adapter_does_not_publish_legacy_verification():
    source = inspect.getsource(ProbeSetupStateAdapter)

    assert "SurfaceVerificationState" not in source
    assert "_write_surface_verification" not in source


def test_close_surface_runtime_remains_independent_of_probe_setup():
    source = inspect.getsource(ProbeSurfaceRuntimeStateSource)

    assert "active_attachment" in source
    assert "surface_distance_samples" in source
    assert "probe_setup" not in source
