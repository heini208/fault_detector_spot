from builtin_interfaces.msg import Time as TimeMessage
from rclpy.time import Time

from fault_detector_spot.sensing.observations.tag_observation_time import (
    is_observation_fresh,
    observation_age_sec,
)


def make_stamp(seconds: float) -> TimeMessage:
    whole_seconds = int(seconds)
    nanoseconds = int(
        (seconds - whole_seconds) * 1_000_000_000
    )
    return TimeMessage(
        sec=whole_seconds,
        nanosec=nanoseconds,
    )


def test_observation_age():
    current = Time(seconds=10.0)
    stamp = make_stamp(9.8)

    age = observation_age_sec(current, stamp)

    assert abs(age - 0.2) < 1e-9


def test_fresh_observation_is_accepted():
    current = Time(seconds=10.0)
    stamp = make_stamp(9.8)

    assert is_observation_fresh(
        current,
        stamp,
        maximum_age_sec=0.5,
    )


def test_stale_observation_is_rejected():
    current = Time(seconds=10.0)
    stamp = make_stamp(9.0)

    assert not is_observation_fresh(
        current,
        stamp,
        maximum_age_sec=0.5,
    )


def test_large_future_timestamp_is_rejected():
    current = Time(seconds=10.0)
    stamp = make_stamp(10.2)

    assert not is_observation_fresh(
        current,
        stamp,
        maximum_age_sec=0.5,
    )


def test_zero_timestamp_is_rejected():
    current = Time(seconds=10.0)
    stamp = TimeMessage()

    assert not is_observation_fresh(
        current,
        stamp,
        maximum_age_sec=0.5,
    )