"""Tests for per-tag observation caching."""

from fault_detector_msgs.msg import TagElement
from rclpy.time import Time

from fault_detector_spot.sensing.data import (
    tag_observation_cache,
)

TagObservationCache = tag_observation_cache.TagObservationCache


def make_tag(
    tag_id: int,
    stamp_sec: int,
    x: float,
) -> TagElement:
    """Create a timestamped body-frame tag observation."""
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = stamp_sec
    tag.pose.pose.position.x = x
    tag.pose.pose.orientation.w = 1.0
    return tag


def test_empty_update_does_not_clear_cache():
    """A missed source update does not immediately remove a tag."""
    cache = TagObservationCache(max_age_sec=1.5)
    cache.update({7: make_tag(7, 10, 1.0)})
    cache.update({})

    snapshot = cache.snapshot(Time(seconds=10.5))

    assert set(snapshot) == {7}


def test_expired_observation_is_removed():
    """An observation is removed after its configured lifetime."""
    cache = TagObservationCache(max_age_sec=1.0)
    cache.update({7: make_tag(7, 10, 1.0)})

    snapshot = cache.snapshot(Time(seconds=11.1))

    assert snapshot == {}


def test_older_observation_does_not_replace_newer_one():
    """Out-of-order TF resolution cannot move a cache backward."""
    cache = TagObservationCache(max_age_sec=2.0)
    cache.update({7: make_tag(7, 10, 2.0)})
    cache.update({7: make_tag(7, 9, 1.0)})

    snapshot = cache.snapshot(Time(seconds=10.5))

    assert snapshot[7].pose.pose.position.x == 2.0


def test_snapshot_is_a_deep_copy():
    """Consumers cannot mutate the cache through a snapshot."""
    cache = TagObservationCache(max_age_sec=2.0)
    cache.update({7: make_tag(7, 10, 1.0)})

    first = cache.snapshot(Time(seconds=10.5))
    first[7].pose.pose.position.x = 99.0
    second = cache.snapshot(Time(seconds=10.5))

    assert second[7].pose.pose.position.x == 1.0
