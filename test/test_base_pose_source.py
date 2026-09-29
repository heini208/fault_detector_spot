"""Regression tests for measured Spot base pose sampling."""

from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped

from fault_detector_spot.navigation.base_pose_source import BasePoseSource


class Listener:

    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error

    def lookup_a_tform_b(self, *_args, **_kwargs):
        if self.error is not None:
            raise self.error
        return self.result


def test_valid_transform_returns_planar_sample():
    transform = TransformStamped()
    transform.header.stamp.sec = 12
    transform.header.stamp.nanosec = 500_000_000
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.5
    transform.transform.rotation.w = 1.0

    sample = BasePoseSource(Listener(transform)).sample()

    assert sample is not None
    assert sample.planar_pose == (1.25, -0.5, 0.0)
    assert sample.stamp_sec == 12.5


def test_lookup_failure_returns_no_sample():
    source = BasePoseSource(
        Listener(error=RuntimeError("TF unavailable"))
    )

    assert source.sample() is None


def test_malformed_transform_returns_no_sample():
    source = BasePoseSource(
        Listener(SimpleNamespace(transform=SimpleNamespace()))
    )

    assert source.sample() is None


def test_invalid_quaternion_returns_no_sample():
    transform = TransformStamped()
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 0.0

    assert BasePoseSource(Listener(transform)).sample() is None
