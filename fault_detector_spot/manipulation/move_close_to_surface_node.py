"""Host force-guarded close-surface movement outside the behavior tree."""

from threading import Event, RLock
import math
import time

import rclpy
from fault_detector_msgs.action import MoveCloseToSurface
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with

from fault_detector_spot.inspection.execution.move_close_to_surface_operation import (
    MoveCloseToSurfaceOperation,
    MoveCloseToSurfaceStatus,
)
from fault_detector_spot.inspection.execution.probe_surface_runtime_state import (
    ProbeSurfaceRuntimeStateSource,
)
from fault_detector_spot.manipulation.move_close_to_surface_interface import (
    MOVE_CLOSE_TO_SURFACE_ACTION,
    MoveCloseToSurfaceRequest,
)


class MoveCloseToSurfaceNode(Node):
    """Own close-surface sensing, TF, robot commands, and recovery."""

    def __init__(self):
        super().__init__("move_close_to_surface")
        self._lock = RLock()
        self._active = False
        self._shutdown = Event()
        self._callback_group = ReentrantCallbackGroup()
        self._robot_name = self._parameter("close_surface.robot_name", "")
        action_name = self._parameter(
            "close_surface.action_name",
            MOVE_CLOSE_TO_SURFACE_ACTION,
        )
        self._state_source = ProbeSurfaceRuntimeStateSource(self)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand,
            namespace_with(self._robot_name, "robot_command"),
            self,
            wait_for_server=False,
        )
        self._operation = MoveCloseToSurfaceOperation(
            self,
            self._state_source,
            self._robot_command_client,
            robot_name=self._robot_name,
            motion_duration_sec=float(self._parameter(
                "close_surface.motion_duration_sec",
                2.0,
            )),
            settle_sec=float(self._parameter(
                "close_surface.settle_sec",
                0.5,
            )),
            sample_timeout_sec=float(self._parameter(
                "close_surface.sample_timeout_sec",
                3.0,
            )),
            force_baseline_timeout_sec=float(self._parameter(
                "close_surface.force_baseline_timeout_sec",
                2.0,
            )),
            maximum_step_m=float(self._parameter(
                "close_surface.maximum_step_m",
                0.010,
            )),
            tolerance_m=float(self._parameter(
                "close_surface.tolerance_m",
                0.005,
            )),
            maximum_travel_m=float(self._parameter(
                "close_surface.maximum_travel_m",
                0.400,
            )),
            maximum_approach_steps=int(self._parameter(
                "close_surface.maximum_approach_steps",
                40,
            )),
            minimum_surface_samples=int(self._parameter(
                "close_surface.minimum_surface_samples",
                5,
            )),
            minimum_surface_span_sec=float(self._parameter(
                "close_surface.minimum_surface_span_sec",
                1.0,
            )),
            surface_stability_tolerance_m=float(self._parameter(
                "close_surface.surface_stability_tolerance_m",
                0.005,
            )),
            force_contact_threshold_n=float(self._parameter(
                "close_surface.force_contact_threshold_n",
                5.0,
            )),
            force_contact_consecutive_samples=int(self._parameter(
                "close_surface.force_contact_consecutive_samples",
                2,
            )),
            force_stale_timeout_sec=float(self._parameter(
                "close_surface.force_stale_timeout_sec",
                1.5,
            )),
            maximum_contact_retries=int(self._parameter(
                "close_surface.maximum_contact_retries",
                3,
            )),
            recovery_step_m=float(self._parameter(
                "close_surface.recovery_step_m",
                0.040,
            )),
            maximum_recovery_steps=int(self._parameter(
                "close_surface.maximum_recovery_steps",
                20,
            )),
            maximum_lateral_drift_m=float(self._parameter(
                "close_surface.maximum_lateral_drift_m",
                0.010,
            )),
            maximum_axis_error_rad=float(self._parameter(
                "close_surface.maximum_axis_error_rad",
                math.radians(5.0),
            )),
            minimum_step_progress_ratio=float(self._parameter(
                "close_surface.minimum_step_progress_ratio",
                0.25,
            )),
        )
        self._server = ActionServer(
            self,
            MoveCloseToSurface,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _accept(self, goal_request):
        if self._shutdown.is_set():
            return GoalResponse.REJECT
        target = float(goal_request.target_surface_distance_m)
        if not math.isfinite(target) or target <= 0.0:
            return GoalResponse.REJECT
        if not goal_request.request_id.strip():
            return GoalResponse.REJECT
        with self._lock:
            if self._active:
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MoveCloseToSurface.Result()
        try:
            command = MoveCloseToSurfaceRequest(
                target_surface_distance_m=(
                    goal.target_surface_distance_m
                ),
                request_id=goal.request_id,
            )
            self._operation.start(command)
            last_feedback = None
            while True:
                if self._shutdown.is_set():
                    self._operation.cancel()
                    result.success = False
                    result.detail = "Close-surface server is shutting down"
                    goal_handle.abort()
                    return result
                if goal_handle.is_cancel_requested:
                    self._operation.cancel()
                    result.success = False
                    result.detail = "Close-surface movement cancelled"
                    goal_handle.canceled()
                    return result

                status = self._operation.update()
                feedback_signature = (
                    self._operation.phase,
                    self._operation.feedback_message,
                )
                if feedback_signature != last_feedback:
                    feedback = MoveCloseToSurface.Feedback()
                    feedback.phase = feedback_signature[0]
                    feedback.detail = feedback_signature[1]
                    goal_handle.publish_feedback(feedback)
                    last_feedback = feedback_signature

                if status is MoveCloseToSurfaceStatus.SUCCESS:
                    result.success = True
                    result.detail = self._operation.feedback_message
                    goal_handle.succeed()
                    return result
                if status is MoveCloseToSurfaceStatus.FAILURE:
                    result.success = False
                    result.detail = self._operation.feedback_message
                    goal_handle.abort()
                    return result
                time.sleep(0.05)
        except Exception as exception:
            self._operation.cancel()
            result.success = False
            result.detail = str(exception)
            self.get_logger().error(result.detail)
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._active = False

    def destroy_node(self):
        self.request_shutdown()
        self._operation.cancel()
        self._server.destroy()
        self._state_source.close()
        self._robot_command_client.destroy()
        return super().destroy_node()

    def request_shutdown(self):
        """Stop accepting work and terminate an active execution loop."""
        self._shutdown.set()


def main(args=None):
    rclpy.init(args=args)
    node = MoveCloseToSurfaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.request_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
