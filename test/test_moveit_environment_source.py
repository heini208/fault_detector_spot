"""Freshness checks use real ROS messages without starting ROS nodes."""

from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
from fault_detector_spot.manipulation.moveit_environment_source import MoveItEnvironmentSource


def source_rig():
    clock = [10.0]
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))),
        create_subscription=lambda *args: object(),
        destroy_subscription=lambda subscription: None,
    )
    source = MoveItEnvironmentSource(node, ArmMotionParameters(), lambda: clock[0])
    return source, clock


def odometry(stamp, *, x=0.0, velocity=0.0):
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(stamp * 1e9), 10**9)
    message.header.frame_id = "odom"
    message.child_frame_id = "body"
    message.pose.pose.orientation.w = 1.0
    message.pose.pose.position.x = x
    message.twist.twist.linear.x = velocity
    return message


def cloud(stamp, value=0.6):
    message = PointCloud2(height=1, width=1, point_step=12, row_step=12)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(stamp * 1e9), 10**9)
    message.fields = [PointField(name=axis, offset=index * 4, datatype=7, count=1)
                      for index, axis in enumerate("xyz")]
    message.data = np.array([value, 0, 0], dtype="<f4").tobytes()
    return message


def test_cloud_acquisition_must_follow_clear_acknowledgment_and_remain_fresh():
    source, clock = source_rig()
    source._receive_odometry(odometry(10))
    assert not source.begin_refresh()
    clock[0] = 10.1
    source._receive_cloud(cloud(10.1))
    assert not source.has_fresh_cloud()
    clock[0] = 10.2
    source.map_cleared()
    assert not source.has_fresh_cloud()  # pre-clear cloud is still excluded
    source._receive_cloud(cloud(10.15))  # delayed old acquisition
    assert not source.has_fresh_cloud()
    source._receive_cloud(cloud(10.2, float("nan")))
    assert not source.has_fresh_cloud()
    source._receive_cloud(cloud(10.2, 0.0))
    assert not source.has_fresh_cloud()
    source._receive_cloud(cloud(10.2))
    assert source.has_fresh_cloud()
    clock[0] = 12
    assert not source.has_fresh_cloud()


def test_motion_and_missing_or_stale_odometry_invalidate_the_observation_interval():
    source, clock = source_rig()
    assert source.begin_refresh()
    source._receive_odometry(odometry(10))
    assert not source.begin_refresh()
    clock[0] = 10.1
    source._receive_odometry(odometry(10.1, velocity=0.2))
    assert source.motion_problem()
    clock[0] = 10.45
    source._receive_odometry(odometry(10.45))
    assert source.motion_problem()  # returning to the same pose doesn't repair a moved map
    assert not source.begin_refresh()
    clock[0] = 10.5
    source._receive_odometry(odometry(10.5, x=0.06))
    assert source.motion_problem()
    assert not source.begin_refresh()
    clock[0] = 11.1
    assert source.motion_problem()
