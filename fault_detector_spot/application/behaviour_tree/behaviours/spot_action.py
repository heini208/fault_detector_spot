import math
import time
from typing import Optional, Tuple, Type

import py_trees
from py_trees.common import Access, Status
from rclpy.action import ActionClient
from spot_msgs.action import RobotCommand
import synchros2.scope as ros_scope
from synchros2.utilities import namespace_with


class BoundedActionClientBehaviour(py_trees.behaviour.Behaviour):
    """Run one ROS action goal with bounded acceptance and execution waits."""

    def __init__(
        self,
        name: str,
        goal_response_timeout_sec: float = 2.0,
        result_timeout_sec: float = 30.0,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(name)
        self.goal_response_timeout_sec = self._positive_timeout(
            goal_response_timeout_sec,
            "Goal response timeout",
        )
        self.result_timeout_sec = self._positive_timeout(
            result_timeout_sec,
            "Action result timeout",
        )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        self._monotonic_clock = monotonic_clock

        self.initialized = False
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None
        self._client = None

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if not self.node:
            raise RuntimeError(f"{self.__class__.__name__} requires a ROS node")

    def initialise(self):
        self._reset_state()

    def update(self) -> Status:
        try:
            before_update = self._before_update()
            if before_update is not None:
                return before_update
            for phase in (
                self._phase_initialize,
                self._phase_send_goal,
                self._phase_wait_for_acceptance,
                self._phase_wait_for_result,
            ):
                result = phase()
                if result is not None:
                    return result
        except Exception as exception:
            return self._fail(f"Action client failed: {exception}")
        return Status.RUNNING

    def _before_update(self) -> Optional[Status]:
        return None

    def _phase_initialize(self) -> Optional[Status]:
        if self.initialized:
            return None
        try:
            initialized = self._init_client()
        except Exception as exception:
            return self._fail(f"Action client initialization failed: {exception}")
        if not initialized:
            detail = self.feedback_message or "Action client initialization failed"
            return self._fail(detail)
        self.feedback_message = "Client initialized"
        return None

    def _phase_send_goal(self) -> Optional[Status]:
        if self.send_goal_future is not None:
            return None
        try:
            goal = self._build_goal()
            if isinstance(goal, Status) or goal is None:
                return goal or self._fail("Action goal could not be built")
            self.logger.info(f"[{self.name}] Sending goal")
            self.send_goal_future = self._send_goal(goal)
            if self.send_goal_future is None:
                return self._fail("Action client returned no goal future")
        except Exception as exception:
            return self._fail(f"Action goal preparation failed: {exception}")
        self._goal_sent_monotonic = self._monotonic_clock()
        self.feedback_message = "Goal sent"
        return Status.RUNNING

    def _phase_wait_for_acceptance(self) -> Optional[Status]:
        if self.goal_handle is not None or self.send_goal_future is None:
            return None
        if not self.send_goal_future.done():
            if self._goal_response_timed_out():
                self._request_cancel()
                return self._fail(
                    "Action goal response timed out after "
                    f"{self.goal_response_timeout_sec:.1f} s"
                )
            return Status.RUNNING
        try:
            self.goal_handle = self.send_goal_future.result()
        except Exception as exception:
            return self._fail(f"Action goal submission failed: {exception}")
        if self.goal_handle is None or not self.goal_handle.accepted:
            return self._fail("Action goal was rejected")
        try:
            self.get_result_future = self.goal_handle.get_result_async()
        except Exception as exception:
            return self._fail(f"Action result request failed: {exception}")
        self._result_started_monotonic = self._monotonic_clock()
        self.feedback_message = "Goal accepted"
        return Status.RUNNING

    def _phase_wait_for_result(self) -> Optional[Status]:
        if self.get_result_future is None:
            return None
        if not self.get_result_future.done():
            if self._result_timed_out():
                self._request_cancel()
                timeout = self.result_timeout_sec
                return self._fail(
                    f"Action result timed out after {timeout:.1f} s"
                )
            return Status.RUNNING
        try:
            result_wrapper = self.get_result_future.result()
            result = result_wrapper.result
            status, detail = self._interpret_result(result)
        except Exception as exception:
            return self._fail(f"Action result failed: {exception}")
        if status is Status.FAILURE:
            return self._fail(detail)
        self.feedback_message = detail
        self._reset_state()
        return status

    def terminate(self, new_status: Status):
        if new_status == Status.INVALID:
            self._request_cancel()
        self._reset_state()

    def shutdown(self):
        self._request_cancel()
        self._reset_state()

    def _request_cancel(self) -> None:
        handle = self.goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exception:
                self.logger.error(
                    f"[{self.name}] Goal cancellation failed: {exception}"
                )
            return

        future = self.send_goal_future
        if future is None or future.done():
            return

        def cancel_when_accepted(done_future):
            try:
                accepted_handle = done_future.result()
                if accepted_handle is not None and accepted_handle.accepted:
                    accepted_handle.cancel_goal_async()
            except Exception as exception:
                self.logger.error(
                    f"[{self.name}] Pending goal cancellation failed: {exception}"
                )

        future.add_done_callback(cancel_when_accepted)

    def _fail(self, detail: str) -> Status:
        normalized = str(detail).strip() or "Action failed"
        self._on_failure(normalized)
        self.feedback_message = normalized
        self._reset_state()
        return Status.FAILURE

    def _on_failure(self, detail: str) -> None:
        return None

    def _interpret_result(self, result) -> Tuple[Status, str]:
        if bool(getattr(result, "success", False)):
            return Status.SUCCESS, "Succeeded"
        return Status.FAILURE, f"Action failed: {result}"

    def _reset_state(self):
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None
        self._reset_subclass_state()

    def _reset_subclass_state(self) -> None:
        return None

    def _send_goal(self, goal):
        return self._client.send_goal_async(goal)

    def _goal_response_timed_out(self) -> bool:
        return self._deadline_expired(
            self._goal_sent_monotonic,
            self.goal_response_timeout_sec,
        )

    def _result_timed_out(self) -> bool:
        return self._deadline_expired(
            self._result_started_monotonic,
            self.result_timeout_sec,
        )

    def _deadline_expired(self, started, timeout) -> bool:
        if started is None or timeout is None:
            return False
        return self._monotonic_clock() - started >= timeout

    @staticmethod
    def _positive_timeout(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{label} must be positive and finite")
        return normalized

    def _init_client(self) -> bool:
        raise NotImplementedError

    def _build_goal(self):
        raise NotImplementedError


class RobotCommandActionBehaviour(BoundedActionClientBehaviour):
    """Execute one command through Spot's shared RobotCommand action client."""

    def __init__(
        self,
        name: str,
        robot_name: str = "",
        robot_command_resources=None,
        goal_response_timeout_sec: float = 2.0,
        result_timeout_sec: float = 30.0,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            goal_response_timeout_sec=goal_response_timeout_sec,
            result_timeout_sec=result_timeout_sec,
            monotonic_clock=monotonic_clock,
        )
        self.robot_name = robot_name
        self.robot_command_resources = robot_command_resources

    def _init_client(self) -> bool:
        action_ns = namespace_with(self.robot_name, "robot_command")
        if self.robot_command_resources is not None:
            self._client = self.robot_command_resources.get_action_client(
                self.node,
                self.robot_name,
            )
        elif self._client is None:
            from synchros2.action_client import ActionClientWrapper

            self._client = ActionClientWrapper(
                RobotCommand,
                action_ns,
                self.node,
                wait_for_server=False,
            )
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.feedback_message = f"Action server '{action_ns}' unavailable"
            return False
        self.initialized = True
        return True


class WorkflowActionBehaviour(BoundedActionClientBehaviour):
    """Execute one typed direct-operation action hosted by a dedicated node."""

    def __init__(
        self,
        name: str,
        action_type: Type,
        action_name: str,
        goal_response_timeout_sec: float = 2.0,
        result_timeout_sec: float = 600.0,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            goal_response_timeout_sec=goal_response_timeout_sec,
            result_timeout_sec=result_timeout_sec,
            monotonic_clock=monotonic_clock,
        )
        self.action_type = action_type
        self.action_name = action_name

    def _init_client(self) -> bool:
        if self._client is None:
            self._client = ActionClient(
                self.node,
                self.action_type,
                self.action_name,
            )
        if not self._client.server_is_ready():
            self.feedback_message = (
                f"Action server '{self.action_name}' unavailable"
            )
            return False
        self.initialized = True
        return True

    def shutdown(self):
        client = self._client
        super().shutdown()
        if client is not None:
            client.destroy()
        self._client = None
        self.initialized = False


__all__ = [
    "BoundedActionClientBehaviour",
    "RobotCommandActionBehaviour",
    "WorkflowActionBehaviour",
]
