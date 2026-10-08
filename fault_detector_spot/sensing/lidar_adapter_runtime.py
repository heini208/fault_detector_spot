"""Share one lidar adapter between the application's existing consumers."""

import time
from threading import Event

from rclpy.clock import Clock, ClockType
from std_srvs.srv import Trigger

from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager


class LidarAdapterRuntime(RuntimeManager):
    PROCESS_KEY = "lidar_adapter_launch_process"
    RUNTIME_NAME = "lidar adapter"
    OUTPUT_TOPIC = "/velodyne/points_sensor"
    STOP_SERVICE = "/fault_detector/lidar_adapter/stop"

    def __init__(self, node, blackboard, mapping_runtime, collision_control):
        super().__init__(node, blackboard)
        self._mapping = mapping_runtime
        self._collision = collision_control
        self._stop_client = node.create_client(Trigger, self.STOP_SERVICE)
        # Allow discovery of an adapter that was started before the application.
        self._next_start = time.monotonic() + 2.0
        self._next_external_stop = 0.0
        self._last_warning = ""
        self._timer = node.create_timer(
            0.5, self._schedule_reconcile,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _required(self):
        # Localization/Nav2 still consume lidar when map building is disabled.
        return (self._mapping.is_running()
                or self._mapping.nav2_runtime.is_running()
                or self._collision.state().enabled)

    def _schedule_reconcile(self):
        with self._runtime_lock:
            if not self._closing:
                self.begin_runtime_operation("reconcile", self._reconcile)

    def _warn(self, message):
        if message != self._last_warning:
            self.node.get_logger().warning(message)
            self._last_warning = message

    def _reconcile(self):
        """Runs only on the runtime worker; never waits in a ROS callback."""
        try:
            self._reconcile_adapter()
        except Exception as exception:
            self._next_start = time.monotonic() + 5.0
            self._warn(f"Lidar adapter lifecycle failed: {exception}")

    def _reconcile_adapter(self):
        if self._closing:
            return
        if self.process is not None and self.process.poll() is not None:
            # Reap the launch parent and any remaining children before replacing it.
            if not self.stop():
                return
        publishers = self.node.get_publishers_info_by_topic(self.OUTPUT_TOPIC)
        if len(publishers) > 1:
            self._warn(
                f"Multiple publishers on {self.OUTPUT_TOPIC}; stop duplicate lidar "
                "sources. No additional adapter will be launched."
            )
            # We may still stop our own group below when consumers are disabled.
        if self._required():
            if self.is_running() or publishers or self._stop_client.service_is_ready():
                return
            if time.monotonic() < self._next_start:
                return
            # Recheck demand after discovery, immediately before launching.
            if self._closing or not self._required():
                return
            self._next_start = time.monotonic() + 5.0
            self._launch("lidar_frame_adapter_launch.py")
            self._last_warning = ""
            return

        # Keep the source through map switches and until stop/save has finished.
        if self._mapping.has_pending_operation():
            return
        if self.process is not None:
            if self.stop():
                self._next_start = time.monotonic() + 2.0
            return
        if len(publishers) != 1:
            return
        publisher = publishers[0]
        if (publisher.node_name != "lidar_frame_adapter"
                or publisher.node_namespace != "/"):
            # A rosbag or other publisher is not ours to shut down.
            return
        if time.monotonic() < self._next_external_stop:
            return
        if not self._stop_client.service_is_ready():
            self._warn(
                "Existing lidar adapter has no shutdown service. Stop the old "
                "manual adapter once after updating; automatic ownership then applies."
            )
            return
        self._next_external_stop = time.monotonic() + 5.0
        if self._required() or self._closing:
            return
        self._request_external_stop()

    def _request_external_stop(self):
        future = self._stop_client.call_async(Trigger.Request())
        done = Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(2.0):
            self._stop_client.remove_pending_request(future)
            self._warn("Existing lidar adapter did not acknowledge shutdown")
            return
        response = future.result()
        if not response.success:
            self._warn(f"Existing lidar adapter refused shutdown: {response.message}")
            return
        self._last_warning = ""
        self._next_start = time.monotonic() + 2.0
        self.node.get_logger().info("Requested shutdown of existing lidar adapter")

    def close(self):
        # Stop scheduling before draining the worker and stopping our process group.
        with self._runtime_lock:
            self._closing = True
            if self._timer is not None:
                self.node.destroy_timer(self._timer)
                self._timer = None
        result = super().close()
        if self._stop_client is not None:
            self.node.destroy_client(self._stop_client)
            self._stop_client = None
        return result
