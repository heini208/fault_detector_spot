"""Tests for setup context lookup owned directly by SetupCoordinator."""

import pytest

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)


class FakeCommandController:
    def __init__(self):
        self.listeners = []

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)


def test_setup_coordinator_resolves_context_by_domain_and_client():
    coordinator = SetupCoordinator(FakeCommandController())
    navigation = coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )
    probe = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )

    assert coordinator.contexts_for(
        CommandOrigin.NAVIGATION_SETUP
    ) == (navigation,)
    assert coordinator.contexts_for(
        CommandOrigin.PROBE_SETUP
    ) == (probe,)
    assert coordinator.resolve_context(
        navigation.context_id,
        "navigation-ui",
        CommandOrigin.NAVIGATION_SETUP,
    ) == navigation

    with pytest.raises(ValueError, match="does not own"):
        coordinator.resolve_context(
            navigation.context_id,
            "other-ui",
            CommandOrigin.NAVIGATION_SETUP,
        )

    with pytest.raises(LookupError, match="Unknown setup context"):
        coordinator.resolve_context(
            navigation.context_id,
            "navigation-ui",
            CommandOrigin.PROBE_SETUP,
        )


def test_setup_coordinator_resolves_latest_context_revision():
    coordinator = SetupCoordinator(FakeCommandController())
    initial = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )
    updated = coordinator.advance_context(initial)

    assert coordinator.resolve_context(
        initial.context_id,
        "probe-ui",
        CommandOrigin.PROBE_SETUP,
    ) == updated
