"""Schedule guarded execution independently of Behavior Tree polling."""

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType

from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)


class GuardedProbeMonitor:
    """Monitor only active operations; BT polling reads the latest update.

    Force events react immediately. One timer advances the lifecycle at 50 ms
    intervals, including when force updates stop arriving. After first use,
    idle monitors keep one cancelled timer but schedule no work.
    """

    def __init__(self, node, arm_state_source, execution):
        """Keep dependencies without adding work to the ROS executor."""
        self._node = node
        self._source = arm_state_source
        self._execution = execution
        self._closed = False
        self._generation = 0
        self._timer = None
        self._listener = None
        self._update = ArmMovementUpdate(
            ArmMovementOutcome.EXECUTION_ERROR,
            "No guarded probe movement is active",
        )
        self._callback_group = MutuallyExclusiveCallbackGroup()

    def start(self, plan_builder, force_threshold_n=None):
        """Attach monitoring before starting the guarded operation."""
        with self._execution.lock:
            if self._closed:
                raise RuntimeError("Guarded probe monitor is closed")
            self.stop()
            generation = self._generation
            self._listener = lambda sample: self._observe_force(
                sample, generation,
            )
            try:
                self._source.add_force_listener(self._listener)
                self._start_timer()
                self._record(self._execution.start(
                    plan_builder, force_threshold_n=force_threshold_n,
                ))
            except Exception:
                self.stop()
                raise
            return self._update

    def poll(self):
        """Return status without advancing the operation a second time."""
        with self._execution.lock:
            return self._update

    def _record(self, update):
        if update is not None:
            self._update = update
            if update.outcome is not ArmMovementOutcome.RUNNING:
                self.stop()

    def _observe_force(self, sample, generation):
        with self._execution.lock:
            if not self._closed and generation == self._generation:
                # Delivery may have waited long enough to become stale.
                if (
                    sample is not None
                    and self._source.hand_force_sample() is None
                ):
                    sample = None
                self._record(self._execution.observe_force_sample(sample))

    def _advance(self):
        with self._execution.lock:
            if self._closed:
                return
            if self._execution.active:
                self._record(self._execution.poll())
            else:
                self.stop()

    def _start_timer(self):
        if self._timer is None:
            self._timer = self._node.create_timer(
                0.05,
                self._advance,
                callback_group=self._callback_group,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            return
        self._timer.reset()

    def stop(self):
        """Deactivate scheduled work without destroying a live ROS handle."""
        with self._execution.lock:
            self._generation += 1
            if self._listener is not None:
                self._source.remove_force_listener(self._listener)
                self._listener = None
            if self._timer is not None:
                self._timer.cancel()

    def close(self):
        """Detach monitoring permanently and release its timer."""
        with self._execution.lock:
            self._closed = True
            self.stop()
            if self._timer is not None:
                self._node.destroy_timer(self._timer)
                self._timer = None
