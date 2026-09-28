"""Focused tests for tag usability filtering."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.sensing.observations.tag_usability import (
    TagUsabilityFilter,
)


def tag(x, y, z):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        )
    )


def test_filter_keeps_tags_within_body_range_including_boundary():
    near = tag(1.4, 0.0, 0.0)
    boundary = tag(0.0, 0.0, 1.5)
    far = tag(1.5001, 0.0, 0.0)
    assert TagUsabilityFilter(1.5).filter(
        {1: near, 2: boundary, 3: far}
    ) == {1: near, 2: boundary}


def test_filter_uses_three_dimensional_distance():
    assert TagUsabilityFilter(1.5).filter({1: tag(1.0, 1.0, 1.0)}) == {}


@pytest.mark.parametrize("coordinate", [float("nan"), float("inf")])
def test_filter_rejects_nonfinite_positions(coordinate):
    assert TagUsabilityFilter(1.5).filter({1: tag(coordinate, 0, 0)}) == {}


@pytest.mark.parametrize("maximum_range", [0.0, -1.0, float("inf"), float("nan")])
def test_filter_rejects_invalid_range(maximum_range):
    with pytest.raises(ValueError):
        TagUsabilityFilter(maximum_range)
