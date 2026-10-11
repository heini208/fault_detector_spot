"""Refresh the confirmed head for RViz without requesting robot movement."""

import time

from rclpy.clock import Clock, ClockType

from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome


class MoveItSceneRefresh:
    """Schedule scene-only work through the same planner used by arm commands.

    The timer shares the BT tick's default mutually exclusive callback group.
    A motion request can take over polling a running refresh; the timer then
    leaves it alone until the planner is idle again.
    """

    def __init__(self, node, planner, sensor_head, arm_is_idle,
                 monotonic_clock=time.monotonic):
        self.node = node
        self.planner = planner
        self.sensor_head = sensor_head
        self._arm_is_idle = arm_is_idle
        self._clock = monotonic_clock
        self._retry_after = 0.0
        self._last_failure = ""
        self._closed = False
        self._timer = node.create_timer(
            0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _tick(self):
        if self._closed or not self._arm_is_idle():
            return
        if self.planner.scene_sync_active:
            update = self.planner.poll()
        elif (self.planner.active or not self.sensor_head.needs_scene_sync
              or self._clock() < self._retry_after):
            return
        else:
            update = self.planner.start_scene_sync()
        if update.outcome is MoveItPlanOutcome.RUNNING:
            return
        if update.outcome is MoveItPlanOutcome.SUCCESS:
            self._last_failure = ""
            self.node.get_logger().info("Confirmed sensor head is visible in the MoveIt scene")
        else:
            self._retry_after = self._clock() + 2.0
            if update.detail != self._last_failure:
                self.node.get_logger().warning(f"Sensor head scene refresh failed: {update.detail}")
                self._last_failure = update.detail

    def destroy(self):
        if not self._closed:
            self._closed = True
            self.node.destroy_timer(self._timer)
