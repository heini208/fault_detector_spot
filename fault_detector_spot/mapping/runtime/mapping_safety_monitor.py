"""Non-blocking RTAB-Map mapping watchdog.

Pauses map integration on missing, stale or invalid tracking observations.
A pause prevents further integration; it cannot undo scans already processed.
"""
from concurrent.futures import ThreadPoolExecutor
from threading import Lock

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
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mapping-safety",
        )
        self._lock = Lock()
        self._pause_future = None
        self._closed = False

    def tick(self):
        """Called by the status timer; never waits for a ROS service."""
        state = self.tracker.snapshot()
        with self._lock:
            if self._closed:
                return state
            if self._pause_future is not None:
                if not self._pause_future.done():
                    return state
                try:
                    self._pause_future.result()
                except Exception as error:
                    self.node.get_logger().error(
                        f"Mapping safety stop failed: {error}"
                    )
                self._pause_future = None
            process = self.runtime.process
            if process is not None and self.tracker.requires_pause(process):
                self._pause_future = self._executor.submit(self._pause, process)
        return state

    def _pause(self, process):
        # Serialize with runtime mode transitions and stop operations.
        with self.runtime._process_lock:
            if (self.runtime.process is not process
                    or not self.runtime.is_mapping_running()
                    or not self.tracker.requires_pause(process)):
                return
            if self.runtime._call_service("/rtabmap/pause", timeout_sec=3.0):
                self.tracker.pause_confirmed(process)
                self.node.get_logger().error(
                    f"RTAB-Map mapping paused: {self.tracker.snapshot()[1]}"
                )
                return
            self.node.get_logger().error(
                "RTAB-Map pause could not be confirmed; stopping the runtime"
            )
            # Best-effort fail closed if the pause service is unavailable.
            if not self.runtime.stop(save=False):
                raise RuntimeError(
                    "RTAB-Map could not be paused or stopped; map is unsafe"
                )

    def close(self):
        with self._lock:
            self._closed = True
        self._executor.shutdown(wait=True, cancel_futures=True)
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        self._subscriptions.clear()
