from builtin_interfaces.msg import Time as TimeMessage
from rclpy.time import Time


def observation_age_sec(
    current_time: Time,
    observation_stamp: TimeMessage,
) -> float:
    observation_time = Time.from_msg(observation_stamp)
    return (
        current_time - observation_time
    ).nanoseconds / 1_000_000_000.0


def is_observation_fresh(
    current_time: Time,
    observation_stamp: TimeMessage,
    maximum_age_sec: float,
    future_tolerance_sec: float = 0.1,
) -> bool:
    if (
        observation_stamp.sec == 0
        and observation_stamp.nanosec == 0
    ):
        return False

    age = observation_age_sec(
        current_time,
        observation_stamp,
    )

    return (
        age >= -future_tolerance_sec
        and age <= maximum_age_sec
    )