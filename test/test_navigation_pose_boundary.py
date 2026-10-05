"""ROS navigation observations are converted before coordinator use."""

from threading import RLock
from unittest.mock import Mock

from fault_detector_spot.navigation.setup import (
    navigation_setup_state_source as pose_source,
)
from fault_detector_spot.shared.geometry.models import PoseData

from geometry_msgs.msg import PoseWithCovarianceStamped

import pytest


@pytest.mark.parametrize('frame,age,accepted', [
    ('map', 0.5, True), ('odom', 0.5, False), ('map', 2.0, False),
])
def test_localization_pose_boundary_checks_frame_and_freshness(
    frame, age, accepted,
):
    """Accept only fresh map poses and return independent domain objects."""
    source = pose_source.NavigationSetupStateSource.__new__(
        pose_source.NavigationSetupStateSource,
    )
    source._lock = RLock()
    source.maximum_pose_age_sec = 1.5
    source.node = Mock()
    now = source.node.get_clock.return_value.now.return_value
    now.nanoseconds = int((10 + age) * 1e9)
    source._localization_pose = PoseWithCovarianceStamped()
    source._localization_pose.header.frame_id = frame
    source._localization_pose.header.stamp.sec = 10
    source._localization_pose.pose.pose.position.x = 2.0
    source._localization_pose.pose.pose.orientation.w = 1.0
    result = source.current_pose()
    if accepted:
        assert isinstance(result, PoseData)
        assert result.position.x == 2.0
        result.position.x = 5.0
        assert source.current_pose().position.x == 2.0
    else:
        assert result is None
