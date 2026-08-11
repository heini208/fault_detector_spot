"""Expose the application command boundary as typed ROS interfaces."""

from dataclasses import dataclass, field
import os
from threading import Event, RLock
from typing import Dict, Optional

import rclpy
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState, ProbeSetupState
from fault_detector_msgs.srv import CancelOperation, EmergencyStop
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
    ApplicationOperation,
    ApplicationOperationStatus,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    UnknownCommandRequest,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.inspection.repository.multi_reference_view_repository import (
    MultiReferenceViewRepository,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi
from fault_detector_spot.application.api.probe_reference_capture_api import (
    ProbeReferenceCaptureApi,
)
from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.application.api.probe_setup_motion_api import (
    ProbeSetupMotionApi,
)
from fault_detector_spot.application.api.probe_refinement_finalization_api import (
    ProbeRefinementFinalizationApi,
)
from fault_detector_spot.application.api.probe_surface_verification_api import (
    ProbeSurfaceVerificationApi,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)
from fault_detector_spot.inspection.setup import (
    probe_setup_motion_state_source,
)
from fault_detector_spot.application.api.navigation_setup_api import (
    NavigationSetupApi,
)
from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    NavigationSetupCoordinator,
)
from fault_detector_spot.navigation.setup.navigation_setup_state_source import (
    NavigationSetupStateSource,
)


_TERMINAL_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
})


@dataclass
class _OperationExecution:
    goal_handle: object
    operation: ApplicationOperation
    finished: Event = field(default_factory=Event)
    state: Optional[ApplicationCommandState] = None
    cancellation_requested: bool = False


class ApplicationApiNode(Node):
    """Own the remote operational API and one command controller."""

    def __init__(self):
        super().__init__("application_api")
        self._lock = RLock()
        self._executions: Dict[str, _OperationExecution] = {}
        self._callback_group = ReentrantCallbackGroup()
        self.command_controller = CommandController(self)
        self.application_controller = ApplicationController(
            self.command_controller
        )
        self.application_controller.add_status_listener(
            self._handle_application_status
        )
        default_map_root = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps",
        )
        self.declare_parameter("navigation.map_root", default_map_root)
        map_root = self.get_parameter(
            "navigation.map_root"
        ).get_parameter_value().string_value
        self.navigation_setup_coordinator = NavigationSetupCoordinator(
            setup_coordinator=self.application_controller.setup_coordinator,
            map_repository=MapRepository(map_root),
            map_artifacts=MapArtifactStore(map_root),
            current_pose=lambda: self.navigation_setup_state.current_pose(),
            visible_tag_pose=(
                lambda tag_id:
                self.navigation_setup_state.visible_tag_pose(tag_id)
            ),
        )
        self.navigation_setup_state = NavigationSetupStateSource(
            self,
            active_map_changed=(
                self.navigation_setup_coordinator.observe_active_map
            ),
        )
        self.application_controller.attach_navigation_setup(
            self.navigation_setup_coordinator
        )
        self.navigation_setup_api = NavigationSetupApi(
            self,
            self.application_controller.navigation_setup_coordinator,
        )
        self.declare_parameter("inspection.object_root", "")
        self.declare_parameter("sensor.root", "")
        object_root = self.get_parameter(
            "inspection.object_root"
        ).value.strip()
        sensor_root = self.get_parameter("sensor.root").value.strip()
        reference_repository = MultiReferenceViewRepository(
            object_root or None
        )
        self.probe_setup_motion_state = (
            probe_setup_motion_state_source.ProbeSetupMotionStateSource(self)
        )
        self.probe_setup_coordinator = ProbeSetupCoordinator(
            setup_coordinator=self.application_controller.setup_coordinator,
            reference_repository=reference_repository,
            sensor_repository=SensorRepository(sensor_root or None),
            motion_state_source=self.probe_setup_motion_state,
        )
        self.application_controller.attach_probe_setup(
            self.probe_setup_coordinator
        )
        self.probe_setup_state_publisher = self.create_publisher(
            ProbeSetupState,
            "fault_detector/application/probe_setup_state",
            APPLICATION_STATE_QOS,
        )
        self.probe_setup_state_adapter = ProbeSetupStateAdapter(
            self.get_clock()
        )
        self.probe_setup_api = ProbeSetupApi(
            self,
            self.application_controller.probe_setup_coordinator,
            self.probe_setup_state_publisher,
            self.probe_setup_state_adapter,
        )
        self.probe_reference_capture_coordinator = (
            ProbeReferenceCaptureCoordinator(
                node=self,
                probe_setup_coordinator=(
                    self.application_controller.probe_setup_coordinator
                ),
                reference_repository=reference_repository,
                motion_state_source=self.probe_setup_motion_state,
            )
        )
        self.probe_reference_capture_api = ProbeReferenceCaptureApi(
            self,
            self.application_controller.probe_setup_coordinator,
            self.probe_reference_capture_coordinator,
            self.probe_setup_state_publisher,
            self.probe_setup_state_adapter,
        )
        self.probe_setup_motion_api = ProbeSetupMotionApi(
            self,
            self.application_controller.probe_setup_coordinator,
            self.probe_setup_state_publisher,
            self.probe_setup_state_adapter,
        )
        self.probe_surface_verification_api = ProbeSurfaceVerificationApi(
            self,
            self.application_controller.probe_setup_coordinator,
            self.probe_setup_state_publisher,
            self.probe_setup_state_adapter,
        )
        self.probe_refinement_finalization_api = (
            ProbeRefinementFinalizationApi(
                self,
                self.application_controller.probe_setup_coordinator,
                self.probe_setup_state_publisher,
                self.probe_setup_state_adapter,
            )
        )
        self._state_publisher = self.create_publisher(
            ApplicationCommandState,
            "fault_detector/application/command_state",
            APPLICATION_STATE_QOS,
        )
        self._operation_server = ActionServer(
            self,
            ExecuteOperation,
            "fault_detector/application/execute_operation",
            execute_callback=self._execute_operation,
            goal_callback=self._accept_operation,
            cancel_callback=self._accept_cancellation,
            callback_group=self._callback_group,
        )
        self._cancel_service = self.create_service(
            CancelOperation,
            "fault_detector/application/cancel_operation",
            self._cancel_operation,
            callback_group=self._callback_group,
        )
        self._emergency_service = self.create_service(
            EmergencyStop,
            "fault_detector/application/emergency_stop",
            self._emergency_stop,
            callback_group=self._callback_group,
        )

    def _accept_operation(self, goal_request):
        try:
            self.application_controller.validate_operation(
                intent=goal_request.intent,
                client_id=goal_request.client_id,
            )
        except (TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Rejected operational intent: {exception}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancellation(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_operation(self, goal_handle):
        goal = goal_handle.request
        operation = self.application_controller.prepare_operation(
            intent=goal.intent,
            client_id=goal.client_id,
            context_id=goal.context_id.strip(),
        )
        execution = _OperationExecution(
            goal_handle=goal_handle,
            operation=operation,
        )
        with self._lock:
            self._executions[operation.request_id] = execution
        try:
            self.application_controller.submit(operation)
        except Exception as exception:
            state = self._failure_state(execution, str(exception))
            execution.state = state
            execution.finished.set()

        while not execution.finished.wait(0.05):
            if (
                goal_handle.is_cancel_requested
                and not execution.cancellation_requested
            ):
                execution.cancellation_requested = True
                try:
                    self.application_controller.cancel(
                        operation.request.client_id,
                        operation.request_id,
                    )
                except UnknownCommandRequest:
                    pass

        state = execution.state or self._failure_state(
            execution,
            "Operation ended without a terminal state",
        )
        result = ExecuteOperation.Result()
        result.state = state
        if state.state == ApplicationCommandState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif state.state == ApplicationCommandState.STATE_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        with self._lock:
            self._executions.pop(operation.request_id, None)
        return result

    def _cancel_operation(self, request, response):
        try:
            cancellation_id = self.application_controller.cancel(
                request.client_id,
                request.request_id,
            )
        except (TypeError, ValueError, UnknownCommandRequest) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.cancellation_request_id = cancellation_id
        response.detail = "Cancellation accepted"
        return response

    def _emergency_stop(self, request, response):
        try:
            request_id = self.application_controller.emergency_stop(
                request.client_id
            )
        except (TypeError, ValueError) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.request_id = request_id
        response.detail = "Emergency stop accepted"
        return response

    def _handle_application_status(
        self,
        status: ApplicationOperationStatus,
    ) -> None:
        state = self._state_message(status)
        self._state_publisher.publish(state)
        with self._lock:
            execution = self._executions.get(
                status.operation.request_id
            )
        if execution is None:
            return
        execution.state = state
        feedback = ExecuteOperation.Feedback()
        feedback.state = state
        execution.goal_handle.publish_feedback(feedback)
        if status.state in _TERMINAL_STATES:
            execution.finished.set()

    def _state_message(self, status):
        message = ApplicationCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        operation = status.operation
        message.request_id = operation.request_id
        message.client_id = operation.request.client_id
        message.context_id = operation.request.context_id
        message.intent = operation.intent
        message.state = self._public_state(status.state)
        message.detail = status.detail or status.state.value
        message.buffered_command_count = status.buffered_command_count
        return message

    def _failure_state(self, execution, detail):
        message = ApplicationCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        operation = execution.operation
        message.request_id = operation.request_id
        message.client_id = operation.request.client_id
        message.context_id = operation.request.context_id
        message.intent = operation.intent
        message.state = ApplicationCommandState.STATE_FAILED
        message.detail = detail
        self._state_publisher.publish(message)
        return message

    @staticmethod
    def _public_state(state):
        values = {
            CommandControllerState.QUEUED: (
                ApplicationCommandState.STATE_QUEUED
            ),
            CommandControllerState.DISPATCHED: (
                ApplicationCommandState.STATE_DISPATCHED
            ),
            CommandControllerState.RUNNING: (
                ApplicationCommandState.STATE_RUNNING
            ),
            CommandControllerState.SUCCEEDED: (
                ApplicationCommandState.STATE_SUCCEEDED
            ),
            CommandControllerState.FAILED: (
                ApplicationCommandState.STATE_FAILED
            ),
            CommandControllerState.CANCELLED: (
                ApplicationCommandState.STATE_CANCELLED
            ),
        }
        return values[state]

    def destroy_node(self):
        self.probe_reference_capture_api.close()
        self.probe_refinement_finalization_api.close()
        self.probe_surface_verification_api.close()
        self.probe_setup_motion_api.close()
        self.probe_setup_api.close()
        self.navigation_setup_api.close()
        self.application_controller.remove_status_listener(
            self._handle_application_status
        )
        self.application_controller.close()
        self.probe_setup_motion_state.close()
        self._operation_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApplicationApiNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
