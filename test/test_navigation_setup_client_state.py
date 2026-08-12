"""Tests for navigation client rendering safety."""

import inspect

from fault_detector_spot.ui.ros.navigation_setup_client import (
    NavigationSetupClient,
)


def test_runtime_state_preserves_last_known_map_list():
    source = inspect.getsource(NavigationSetupClient._emit_state)

    assert "_last_map_names" in source
    assert "_RUNTIME_OPERATIONS" in source
    assert "state.map_names = list(self._last_map_names)" in source
