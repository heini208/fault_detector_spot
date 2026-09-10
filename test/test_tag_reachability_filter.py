"""Focused tests for tag reachability filtering."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.sensing.observations.tag_reachability import (
    TagReachabilityFilter,
)


def tag(x, y, z):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        )
    )


def test_filter_keeps_tags_within_arm_reach():
    near = tag(1.0, 0.0, 0.0)
    far = tag(1.3, 0.0, 0.0)
    filter_ = TagReachabilityFilter(1.1)

    result = filter_.filter(
        {1: near, 2: far},
        (0.1, 0.0, 0.0),
    )

    assert result == {1: near}


def test_filter_returns_empty_until_arm_base_transform_is_available():
    filter_ = TagReachabilityFilter(1.1)

    assert filter_.filter({1: tag(0.5, 0.0, 0.0)}, None) == {}


@pytest.mark.parametrize("maximum_reach", [0.0, -1.0, float("inf")])
def test_filter_rejects_invalid_reach(maximum_reach):
    with pytest.raises(ValueError):
        TagReachabilityFilter(maximum_reach)
