"""Regression guard for shared probe setup bookkeeping."""

from pathlib import Path

from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)


def test_probe_setup_uses_shared_setup_primitives():
    package_root = Path(__file__).parents[1]
    path = (
        package_root
        / "fault_detector_spot/application/coordinators/"
        "probe_setup_coordinator.py"
    )
    source = path.read_text(encoding="utf-8")

    assert "SetupContextAccess" in source
    assert "SetupOperationRegistry" in source
    assert "self._pending" not in source
    assert "self._motion_listeners" not in source
    assert "self._drafts" in source
    assert "self._context_locks" in source
    assert "self._finalizations" in source

    assert ProbeSetupCoordinator.__name__ == "ProbeSetupCoordinator"
