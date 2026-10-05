"""Own waypoint preparation and Nav2 dispatch for every application caller."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import time
from threading import RLock

from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fault_detector_spot.navigation.base_goal_verifier import BaseGoalVerifier

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementOutcome
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.navigation.base_movement_executor import BaseMovementOutcome


class WaypointNavigationOutcome(Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    BUSY = "busy"


@dataclass(frozen=True)
class WaypointNavigationUpdate:
    outcome: WaypointNavigationOutcome
    detail: str


class _Phase(Enum):
    STOWING = "stowing"
    PREPARING_BASE = "preparing_base"
    WAITING_FOR_GOAL = "waiting_for_goal"
    NAVIGATING = "navigating"
    CANCELLING = "cancelling"


class WaypointNavigationExecutor:
    """Stow, prepare height, then navigate; poll without spinning or blocking."""

    def __init__(self, arm_executor, base_executor, action_client, stamp_now,
                 monotonic_clock=time.monotonic, goal_response_timeout_sec=2.0, node=None):
        if not math.isfinite(goal_response_timeout_sec) or goal_response_timeout_sec <= 0:
            raise ValueError("Nav2 goal response timeout must be positive and finite")
        self._node = node
        self._lock = RLock()
        self._cancel_timer = None
        self._cancel_terminal = False
        self._stop_verifier = None
        self._cancel_detail = ""
        self.arm_executor = arm_executor
        self.base_executor = base_executor
        self.action_client = action_client
        self._stamp_now = stamp_now
        self._clock = monotonic_clock
        self.goal_response_timeout_sec = goal_response_timeout_sec
        self._phase = None
        self._owned_preparation = None
        self._goal_pose = None
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self._sent_at = None

    @property
    def active(self):
        return self._phase is not None

    def navigate(self, goal_pose):
        with self._lock:
            return self._navigate(goal_pose)

    def _navigate(self, goal_pose):
        if self.active or self.arm_executor.active or self.base_executor.active:
            return WaypointNavigationUpdate(WaypointNavigationOutcome.BUSY,
                                            "Waypoint or robot movement is already active")
        if not isinstance(goal_pose, PoseStamped) or goal_pose.header.frame_id != "map":
            return self._failure("Waypoint requires a map-frame PoseStamped")
        self._goal_pose = deepcopy(goal_pose)
        try:
            if not self.action_client.wait_for_server(timeout_sec=0.0):
                return self._failure("Nav2 action server unavailable")
            self._phase = _Phase.STOWING
            return self._start_preparation(self.arm_executor, self.arm_executor.stow)
        except Exception as exception:
            return self._failure(f"Waypoint preparation failed: {exception}")

    def _start_preparation(self, executor, start):
        if executor.active:
            return self._failure("Preparation executor is busy")
        self._owned_preparation = executor
        update = start()
        if update.outcome in (ArmMovementOutcome.BUSY, BaseMovementOutcome.BUSY):
            self._owned_preparation = None
        return self._preparation_update(update)

    def _preparation_update(self, update):
        arm = self._phase is _Phase.STOWING
        outcomes = ArmMovementOutcome if arm else BaseMovementOutcome
        label = "Arm stow" if arm else "Walking height"
        if update.outcome is outcomes.RUNNING:
            return self._running(f"{label}: {update.detail}")
        self._owned_preparation = None
        if update.outcome is not outcomes.SUCCESS:
            return self._failure(f"{label} failed: {update.detail}")
        if arm:
            self._phase = _Phase.PREPARING_BASE
            return self._start_preparation(
                self.base_executor, self.base_executor.prepare_for_navigation,
            )
        return self._dispatch()

    def _arm_is_stowed(self):
        source = self.arm_executor.arm_state_source
        return (not self.arm_executor.active and source is not None
                and source.stow_state() is ArmStowState.STOWED)

    def _dispatch(self):
        # Height preparation can take seconds: check arm feedback again at dispatch.
        if not self._arm_is_stowed():
            return self._failure("Fresh stowed-arm confirmation lost before Nav2 dispatch")
        if self.base_executor.active:
            return self._failure("Base movement became active before Nav2 dispatch")
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(self._goal_pose)
        goal.pose.header.stamp = self._stamp_now()
        self._send_future = self.action_client.send_goal_async(goal)
        if self._send_future is None:
            return self._failure("Nav2 returned no goal response future")
        self._sent_at = self._clock()
        self._phase = _Phase.WAITING_FOR_GOAL
        return self._running("Prepared arm and height; waiting for Nav2 goal acceptance")

    def poll(self):
        with self._lock:
            return self._poll()

    def _poll(self):
        if not self.active:
            return WaypointNavigationUpdate(WaypointNavigationOutcome.FAILURE,
                                            "No waypoint navigation is active")
        if self._phase is _Phase.CANCELLING:
            return self._poll_cancellation()
        try:
            if self._phase in {_Phase.STOWING, _Phase.PREPARING_BASE}:
                return self._preparation_update(self._owned_preparation.poll())
            if not self._arm_is_stowed():
                return self._failure("Stowed-arm feedback lost during navigation")
            if self._phase is _Phase.WAITING_FOR_GOAL:
                if not self._send_future.done():
                    if self._clock() - self._sent_at >= self.goal_response_timeout_sec:
                        return self._failure("Nav2 goal response timed out")
                    return self._running("Waiting for Nav2 goal acceptance")
                self._goal_handle = self._send_future.result()
                if self._goal_handle is None or not self._goal_handle.accepted:
                    return self._failure("Nav2 goal rejected")
                self._result_future = self._goal_handle.get_result_async()
                if self._result_future is None:
                    return self._failure("Nav2 returned no result future")
                self._phase = _Phase.NAVIGATING
            if not self._result_future.done():
                return self._running("Navigating with arm stowed")
            result = self._result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                return self._failure(f"Nav2 navigation failed with status {result.status}")
            self._clear()
            return WaypointNavigationUpdate(WaypointNavigationOutcome.SUCCESS,
                                            "Navigation succeeded")
        except Exception as exception:
            return self._failure(f"Waypoint navigation failed: {exception}")

    @property
    def cancelling(self):
        return self._phase is _Phase.CANCELLING

    def cancel(self):
        with self._lock:
            if not self.active or self.cancelling:
                return
            self._phase = _Phase.CANCELLING
            self._cancel_detail = "Waiting for Nav2 cancellation"
            if self._node is not None and self._cancel_timer is None:
                self._cancel_timer = self._node.create_timer(
                    0.05, self._poll_cancellation,
                    clock=Clock(clock_type=ClockType.STEADY_TIME),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            elif self._cancel_timer is not None:
                self._cancel_timer.reset()
            if self._owned_preparation is not None:
                self._owned_preparation.cancel()
            if self._send_future is None:
                self._cancel_terminal = True
                return
            self._send_future.add_done_callback(self._cancel_accepted)

    def _cancel_accepted(self, future):
        with self._lock:
            if not self.cancelling or future is not self._send_future:
                return
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    self._cancel_terminal = True
                    return
                self._goal_handle = handle
                self._result_future = self._result_future or handle.get_result_async()
                handle.cancel_goal_async()
            except Exception as exception:
                self._cancel_detail = f"Navigation stop unconfirmed: {exception}"

    def _poll_cancellation(self):
        with self._lock:
            if not self.cancelling:
                return self._running("Navigation cancellation completed")
            preparation = self._owned_preparation
            if preparation is not None and preparation.active:
                preparation.poll()
                return self._running("Waiting for preparation to stop")
            try:
                if not self._cancel_terminal:
                    if self._result_future is None or not self._result_future.done():
                        return self._running(self._cancel_detail)
                    result = self._result_future.result()
                    if result.status not in {
                        GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_ABORTED,
                    }:
                        return self._running("Navigation result is not terminal; stop unconfirmed")
                    self._cancel_terminal = True
                if self._goal_handle is not None and self._goal_handle.accepted:
                    sample = self.base_executor.base_pose_source.sample()
                    stamp = self._stamp_now()
                    ros_now = stamp.sec + stamp.nanosec * 1e-9
                    if self._stop_verifier is None:
                        self._stop_verifier = BaseGoalVerifier(
                            (0.0, 0.0, 0.0), self.base_executor.goal_verification_config,
                            self._clock(),
                        )
                        self._stop_after_stamp = ros_now
                    pose = sample.planar_pose if sample is not None else None
                    observed = sample.stamp_sec if sample is not None else None
                    if observed is None or observed <= self._stop_after_stamp:
                        pose = None
                    self._stop_verifier.update(pose, observed, ros_now, self._clock())
                    if not self._stop_verifier.settled:
                        return self._running("Nav2 ended; waiting for fresh stationary base feedback")
                detail = self._cancel_detail
                self._clear()
                return WaypointNavigationUpdate(WaypointNavigationOutcome.FAILURE, detail)
            except Exception as exception:
                self._cancel_detail = f"Navigation stop unconfirmed: {exception}"
                return self._running(self._cancel_detail)

    def shutdown(self):
        self.cancel()
        if self._cancel_timer is not None:
            self._node.destroy_timer(self._cancel_timer)
            self._cancel_timer = None
        self.action_client.destroy()

    def _clear(self):
        if self._cancel_timer is not None:
            self._cancel_timer.cancel()
        self._cancel_terminal = False
        self._stop_verifier = None
        self._phase = None
        self._owned_preparation = None
        self._goal_pose = None
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self._sent_at = None

    def _failure(self, detail):
        if self.active:
            self.cancel()
            self._cancel_detail = detail
            return self._poll_cancellation()
        self._clear()
        return WaypointNavigationUpdate(WaypointNavigationOutcome.FAILURE, detail)

    @staticmethod
    def _running(detail):
        return WaypointNavigationUpdate(WaypointNavigationOutcome.RUNNING, detail)
