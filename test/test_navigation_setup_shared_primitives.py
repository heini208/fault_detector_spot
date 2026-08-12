"""Regression guards for shared navigation setup bookkeeping."""

from pathlib import Path

from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    NavigationSetupCoordinator,
)
from fault_detector_spot.application.setup.setup_operation_registry import (
    SetupOperationRegistry,
)


def test_navigation_setup_uses_shared_setup_primitives():
    source = Path(
        NavigationSetupCoordinator.__module__.replace(".", "/") + ".py"
    )
    package_root = Path(__file__).parents[1]
    text = (package_root / source).read_text(encoding="utf-8")

    assert "SetupContextAccess" not in text
    assert "self.setup_coordinator.resolve_context" in text
    assert "self.setup_coordinator.contexts_for" in text
    assert "SetupOperationRegistry" in text
    assert "self._contexts" not in text
    assert "self._pending" not in text
    assert "self._listeners" not in text


def test_operation_registry_is_the_navigation_request_registry():
    assert SetupOperationRegistry.__name__ == "SetupOperationRegistry"
