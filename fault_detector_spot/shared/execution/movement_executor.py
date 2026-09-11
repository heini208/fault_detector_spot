"""Shared RobotCommand lifecycle for physical movement executors."""

import math
import time

from synchros2.utilities import namespace_with

from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryResolver,
    MovementGeometryUnavailable,
)


DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC = 2.0
DEFAULT_RESULT_TIMEOUT_SEC = 30.0


class MovementExecutor:
    """Own the common nonblocking RobotCommand lifecycle."""

    OUTCOME_TYPE = None
    UPDATE_TYPE = None
    MOVEMENT_NAME = "movement"

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        action_client=None,
        goal_response_timeout_sec: float = (
            DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC
        ),
        result_timeout_sec: float = DEFAULT_RESULT_TIMEOUT_SEC,
        monotonic_clock=time.monotonic,
        logger=None,
    ):
        if tf_listener is None:
            raise ValueError(
                f"{self.__class__.__name__} requires a TF listener"
            )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.tf_listener = tf_listener
        self.tag_state_source = tag_state_source
        self.robot_name = robot_name
        self.action_client = action_client
        self.goal_response_timeout_sec = self._positive_timeout(
            goal_response_timeout_sec,
            "Goal response timeout",
        )
        self.result_timeout_sec = self._positive_timeout(
            result_timeout_sec,
            "Action result timeout",
        )
        self._monotonic_clock = monotonic_clock
        self._logger = logger
        self.geometry_resolver = MovementGeometryResolver(
            tf_listener
        )

        self._active = False
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None
        self._pending_goal_builder = None

    @property
    def active(self) -> bool:
        return self._active

    def poll(self):
        """Advance the active RobotCommand lifecycle without blocking."""
        if not self.active:
            return self._new_update(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"No {self.MOVEMENT_NAME} movement is active",
            )

        if self._send_goal_future is None:
            if self._pending_goal_builder is not None:
                return self._submit_goal(
                    self._pending_goal_builder
                )
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Active {self.MOVEMENT_NAME} movement has no goal future",
            )

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def cancel(self) -> None:
        """Request cancellation of the active movement and release it."""
        if not self.active:
            return
        self._request_cancel()
        self._reset_operation()

    def shutdown(self) -> None:
        self.cancel()

    def _start_goal(self, goal_builder):
        if self.active:
            return self._busy_update()

        self._active = True
        self._pending_goal_builder = goal_builder
        return self._submit_goal(goal_builder)

    def _submit_goal(self, goal_builder):
        if self.action_client is None:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"{self._movement_title} movement executor has no "
                "RobotCommand action client",
            )

        try:
            server_ready = self.action_client.wait_for_server(
                timeout_sec=0.0
            )
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"RobotCommand action server check failed: {exception}",
            )

        if not server_ready:
            action_name = namespace_with(
                self.robot_name,
                "robot_command",
            )
            return self._finish(
                self.OUTCOME_TYPE.ACTION_SERVER_UNAVAILABLE,
                f"Action server '{action_name}' unavailable",
            )

        try:
            goal = goal_builder()
        except MovementGeometryUnavailable as exception:
            return self._new_update(
                self.OUTCOME_TYPE.RUNNING,
                str(exception),
            )
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"{self._movement_title} goal preparation failed: "
                f"{exception}",
            )

        try:
            future = self.action_client.send_goal_async(goal)
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"{self._movement_title} goal submission failed: "
                f"{exception}",
            )

        if future is None:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                "RobotCommand action client returned no goal future",
            )

        self._pending_goal_builder = None
        self._send_goal_future = future
        self._goal_sent_monotonic = self._monotonic_clock()
        return self._new_update(
            self.OUTCOME_TYPE.RUNNING,
            "Goal sent",
        )

    def _poll_goal_response(self):
        if self._send_goal_future is None:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Active {self.MOVEMENT_NAME} movement has no goal future",
            )

        if not self._send_goal_future.done():
            if self._deadline_expired(
                self._goal_sent_monotonic,
                self.goal_response_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    self.OUTCOME_TYPE.GOAL_RESPONSE_TIMEOUT,
                    "Action goal response timed out after "
                    f"{self.goal_response_timeout_sec:.1f} s",
                )
            return self._new_update(
                self.OUTCOME_TYPE.RUNNING,
                "Waiting for goal acceptance",
            )

        try:
            goal_handle = self._send_goal_future.result()
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"{self._movement_title} goal submission failed: "
                f"{exception}",
            )

        if goal_handle is None or not goal_handle.accepted:
            return self._finish(
                self.OUTCOME_TYPE.GOAL_REJECTED,
                "Action goal was rejected",
            )

        self._goal_handle = goal_handle
        try:
            self._result_future = goal_handle.get_result_async()
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Action result request failed: {exception}",
            )

        if self._result_future is None:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Accepted {self.MOVEMENT_NAME} goal has no result future",
            )

        self._result_started_monotonic = self._monotonic_clock()
        return self._new_update(
            self.OUTCOME_TYPE.RUNNING,
            "Goal accepted",
        )

    def _poll_result(self):
        if self._result_future is None:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Accepted {self.MOVEMENT_NAME} goal has no result future",
            )

        if not self._result_future.done():
            if self._deadline_expired(
                self._result_started_monotonic,
                self.result_timeout_sec,
            ):
                self._request_cancel()
                return self._finish(
                    self.OUTCOME_TYPE.RESULT_TIMEOUT,
                    "Action result timed out after "
                    f"{self.result_timeout_sec:.1f} s",
                )
            return self._new_update(
                self.OUTCOME_TYPE.RUNNING,
                f"{self._movement_title} movement in progress",
            )

        try:
            result_wrapper = self._result_future.result()
            result = result_wrapper.result
        except Exception as exception:
            return self._finish(
                self.OUTCOME_TYPE.EXECUTION_ERROR,
                f"Action result failed: {exception}",
            )

        if bool(getattr(result, "success", False)):
            return self._handle_successful_result(result)

        return self._finish(
            self.OUTCOME_TYPE.MOTION_FAILED,
            self._command_failure_detail(result),
        )

    def _handle_successful_result(self, _result):
        return self._finish(
            self.OUTCOME_TYPE.SUCCESS,
            "Succeeded",
        )

    def _request_cancel(self) -> None:
        handle = self._goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exception:
                self._log_error(
                    f"{self._movement_title} goal cancellation failed: "
                    f"{exception}"
                )
            return

        future = self._send_goal_future
        if future is None or future.done():
            return

        def cancel_when_accepted(done_future):
            try:
                accepted_handle = done_future.result()
                if (
                    accepted_handle is not None
                    and accepted_handle.accepted
                ):
                    accepted_handle.cancel_goal_async()
            except Exception as exception:
                self._log_error(
                    f"Pending {self.MOVEMENT_NAME} goal cancellation "
                    f"failed: {exception}"
                )

        future.add_done_callback(cancel_when_accepted)

    def _finish(self, outcome, detail: str):
        update = self._new_update(
            outcome,
            str(detail).strip(),
        )
        self._reset_operation()
        return update

    def _reset_goal_lifecycle(self) -> None:
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_monotonic = None
        self._result_started_monotonic = None

    def _reset_operation(self) -> None:
        self._reset_goal_lifecycle()
        self._pending_goal_builder = None
        self._active = False

    def _prepare_move_command(
        self,
        command,
        final_frame: str,
    ):
        return self.geometry_resolver.prepare_move_command(
            command,
            final_frame,
        )

    def _busy_update(self):
        return self._new_update(
            self.OUTCOME_TYPE.BUSY,
            f"Another {self.MOVEMENT_NAME} movement is already active",
        )

    def _new_update(self, outcome, detail: str):
        if self.UPDATE_TYPE is None:
            raise RuntimeError(
                f"{self.__class__.__name__} does not define UPDATE_TYPE"
            )
        return self.UPDATE_TYPE(
            outcome,
            str(detail).strip(),
        )

    def _deadline_expired(self, started, timeout_sec: float) -> bool:
        if started is None:
            return False
        return self._monotonic_clock() - started >= timeout_sec

    @staticmethod
    def _positive_timeout(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized

    @staticmethod
    def _command_failure_detail(result) -> str:
        detail = str(
            getattr(result, "detail", "")
            or getattr(result, "message", "")
        ).strip()
        if detail:
            return detail
        return f"Robot command failed: {result}"

    def _log_error(self, message: str) -> None:
        if self._logger is None:
            return
        log = getattr(self._logger, "error", None)
        if callable(log):
            log(message)

    @property
    def _movement_title(self) -> str:
        return self.MOVEMENT_NAME.capitalize()


__all__ = [
    "DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC",
    "DEFAULT_RESULT_TIMEOUT_SEC",
    "MovementExecutor",
]
