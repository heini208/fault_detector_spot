"""Regression guards for shared navigation setup bookkeeping."""

from pathlib import Path

from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    NavigationSetupCoordinator,
)
from fault_detector_spot.application.setup.setup_context_access import (
    SetupContextAccess,
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

    assert "SetupContextAccess" in text
    assert "SetupOperationRegistry" in text
    assert "self._contexts" not in text
    assert "self._pending" not in text
    assert "self._listeners" not in text


def test_shared_primitive_types_are_the_navigation_dependencies():
    assert SetupContextAccess.__name__ == "SetupContextAccess"
    assert SetupOperationRegistry.__name__ == "SetupOperationRegistry"
