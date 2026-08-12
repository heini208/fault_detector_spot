"""Regression guard for shared probe setup bookkeeping."""

from pathlib import Path

from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)


def test_probe_setup_uses_shared_setup_primitives():
    package_root = Path(__file__).parents[1]
    coordinator_path = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    )
    refinement_path = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_refinement_controller.py"
    )
    finalization_path = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_finalization_controller.py"
    )

    coordinator_source = coordinator_path.read_text(
        encoding="utf-8"
    )
    refinement_source = refinement_path.read_text(
        encoding="utf-8"
    )
    finalization_source = finalization_path.read_text(
        encoding="utf-8"
    )

    assert "SetupContextAccess" not in coordinator_source
    assert "self.setup_coordinator.resolve_context" in coordinator_source
    assert "ProbeRefinementController" in coordinator_source
    assert "ProbeFinalizationController" in coordinator_source
    assert "SetupOperationRegistry" in refinement_source
    assert "self._pending" not in coordinator_source
    assert "self._motion_listeners" not in coordinator_source
    assert "self._drafts" in coordinator_source
    assert "self._context_locks" in coordinator_source
    assert "self._finalizations" not in coordinator_source
    assert "self._active" in finalization_source

    assert ProbeSetupCoordinator.__name__ == "ProbeSetupCoordinator"
