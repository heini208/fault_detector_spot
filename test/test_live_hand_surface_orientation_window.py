"""Regression guards for live hand surface fitting configuration."""

import inspect

import pytest

from fault_detector_spot.inspection.setup import (
    probe_setup_motion_state_source as source_module,
)
from fault_detector_spot.inspection.setup import reference_view_surface_normal


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, value):
        self.value = value

    def get_parameter(self, _name):
        return _Parameter(self.value)


def _source(value):
    source = source_module.ProbeSetupMotionStateSource.__new__(
        source_module.ProbeSetupMotionStateSource
    )
    source.node = _Node(value)
    return source


def test_hand_surface_window_radius_is_runtime_configurable():
    assert _source(24)._hand_surface_window_radius_px() == 24


@pytest.mark.parametrize("value", [3, 65, 4.0, True])
def test_hand_surface_window_radius_rejects_invalid_values(value):
    with pytest.raises(ValueError, match="window radius"):
        _source(value)._hand_surface_window_radius_px()


def test_live_surface_fit_uses_configured_window_for_seed_and_plane():
    source = inspect.getsource(
        source_module.ProbeSetupMotionStateSource.live_hand_surface_orientation
    )

    assert "search_radius_px=window_radius_px" in source
    assert "neighborhood_radius_px=window_radius_px" in source
    assert "maximum_neighborhood_radius_px=window_radius_px" in source


def test_surface_samples_remain_centered_on_requested_image_location():
    source = inspect.getsource(
        reference_view_surface_normal._collect_surface_samples
    )

    assert "center = projected_point.mapped_pixel" in source
