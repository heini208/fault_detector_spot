"""Observe localization evidence for existing-map admission without pausing mapping."""
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info

from fault_detector_spot.navigation.localization_state import LocalizationStateTracker
from fault_detector_spot.shared.ros.qos_profiles import (
    INITIALPOSE_QOS, LOCALIZATION_POSE_QOS,
)


class MappingSafetyMonitor:
    def __init__(self, node, runtime):
        self.node = node
        self.runtime = runtime
        self.tracker = LocalizationStateTracker(
            lambda: node.get_clock().now().nanoseconds,
            # The managed lidar launch enforces ICP-only registration.
            scan_only_registration=(runtime.launch_file == "lidar_rtab_mapping_launch.py"),
        )
        self._subscriptions = [
            node.create_subscription(
                PoseWithCovarianceStamped, "/localization_pose",
                self.tracker.observe_pose, LOCALIZATION_POSE_QOS,
            ),
            node.create_subscription(
                Info, "/info", self.tracker.observe_info, LOCALIZATION_POSE_QOS,
            ),
            node.create_subscription(
                PoseWithCovarianceStamped, "/initialpose",
                self.tracker.observe_seed, INITIALPOSE_QOS,
            ),
        ]

    def tick(self):
        """Return localization evidence without controlling the mapping process."""
        return self.tracker.snapshot()

    def close(self):
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        self._subscriptions.clear()
