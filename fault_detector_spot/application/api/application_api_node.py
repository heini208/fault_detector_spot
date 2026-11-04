"""Expose the application command boundary as typed ROS interfaces."""

from fault_detector_spot.shared.persistence.basic_settings import startup_settings
from fault_detector_spot.application.api.probe_point_execution_api import ProbePointExecutionApi

from dataclasses import dataclass, field
import os
import sys
from threading import Event, RLock
from typing import Dict
from uuid import uuid4

import rclpy
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState, ProbeSetupState
from fault_detector_msgs.srv import CancelOperation, EmergencyStop, ResetSoftwareStop
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory

from fault_detector_spot.application.api.sensor_attachment_api import (
    SensorAttachmentApi,
)
from fault_detector_spot.application.api.sensor_acquisition_api import (
    SensorAcquisitionApi,
)
from fault_detector_spot.application.api.sensor_registry_api import (
    SensorRegistryApi,
)
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
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController,
)
from fault_detector_spot.application.controllers.sensor_registry_controller import (
    SensorRegistryController,
)
from fault_detector_spot.application.ros.command_transport import (
    RosCommandTransport,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.inspection.repository.multi_reference_view_repository import (
    MultiReferenceViewRepository,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    SensorAttachmentStateStore,
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
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionCoordinator,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_command_handler import (
    SensorAcquisitionCommandHandler,
)
from fault_detector_spot.application.api.probe_setup_motion_api import (
    ProbeSetupMotionApi,
)
from fault_detector_spot.application.api.probe_refinement_finalization_api import (
    ProbeRefinementFinalizationApi,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)
from fault_detector_spot.inspection.setup.probe_reference_preview import (
    ProbeReferencePreviewSource,
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
from fault_detector_spot.inspection.measurement import MeasurementRepository
from fault_detector_spot.application.commanding.command_lifecycle import CommandLifecycleConfig
from fault_detector_spot.application.api.shutdown_callbacks import (
    ShutdownCallbackActivity, tracked_shutdown_callback,
)
from fault_detector_spot.shared.ros.shutdown_coordinator import (
    ShutdownCoordinator, shutdown_signals,
)
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from fault_detector_spot.application.api.async_operation import OperationAdmission


_TERMINAL_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
    CommandControllerState.STOP_CONFIRMED,
})


@dataclass
class _OperationExecution:
    goal_handle: object
    operation: ApplicationOperation
    finished: Future = field(default_factory=Future)
    submitted: bool = False
    cancellation_sent: bool = False


class ApplicationApiNode(Node):
    """Own the remote operational API and one command controller."""

    def __init__(self):
        super().__init__("application_api")
        startup_settings()
        self._lock = RLock()
        self._shutdown = Event()
        self._shutdown_callbacks = ShutdownCallbackActivity()
        self._shutdown_coordinator = None
        self._shutdown_lock = RLock()
        self._shutdown_store = StopUnconfirmedStore(owner="application_api")
        self._startup_unresolved = {}
        # A failed marker write must prevent motion admission from starting.
        if not self._shutdown_store.begin_session():
            self._startup_unresolved["application_api/active_session"] = (
                "Application API could not acquire a fresh session: another process or unreadable ownership record remains"
            )
        self.declare_parameter("shutdown.confirmation_timeout_sec", 10.0)
        self.declare_parameter("shutdown.cleanup_timeout_sec", 20.0)
        self._operation_admission = OperationAdmission(limit=1)
        self._executions: Dict[str, _OperationExecution] = {}
        self._callback_group = ReentrantCallbackGroup()
        lifecycle = CommandLifecycleConfig.from_node(self)
        self.command_controller = CommandController(
            ack_timeout_sec=lifecycle.ack_timeout_sec,
            heartbeat_timeout_sec=lifecycle.heartbeat_timeout_sec,
            reconciliation_interval_sec=lifecycle.reconciliation_interval_sec,
            session_work_changed=self._shutdown_store.set_session_work,
            listener_error_handler=(
                lambda exception: self.get_logger().error(
                    f"Command controller listener failed: {exception}"
                )
            )
        )
        if self._startup_unresolved:
            self.command_controller.restore_shutdown_uncertainty(
                "; ".join(self._startup_unresolved.values()),
            )
        self.command_transport = RosCommandTransport(
            self,
            self.command_controller,
            query_timeout_sec=lifecycle.query_timeout_sec,
            poll_period_sec=lifecycle.poll_period_sec,
        )
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
            begin_landmark_capture=(
                lambda tag_id: self.navigation_setup_state.begin_landmark_capture(tag_id)
            ),
            finish_landmark_capture=(
                lambda capture: self.navigation_setup_state.finish_landmark_capture(capture)
            ),
            begin_waypoint_capture=(
                lambda: self.navigation_setup_state.begin_waypoint_capture()
            ),
            waypoint_capture_pose=(
                lambda capture: self.navigation_setup_state.waypoint_capture_pose(capture)
            ),
            finish_waypoint_capture=(
                lambda capture: self.navigation_setup_state.finish_waypoint_capture(capture)
            ),
        )
        self.navigation_setup_state = NavigationSetupStateSource(self)
        self.application_controller.attach_navigation_setup(
            self.navigation_setup_coordinator
        )
        self.navigation_setup_api = NavigationSetupApi(
            self,
            self.application_controller.navigation_setup_coordinator,
            admission=self._operation_admission,
        )
        self.declare_parameter("inspection.object_root", "")
        self.declare_parameter("sensor.root", "")
        self.declare_parameter("measurement.root", "")
        object_root = self.get_parameter(
            "inspection.object_root"
        ).value.strip()
        sensor_root = self.get_parameter("sensor.root").value.strip()
        measurement_root = self.get_parameter(
            "measurement.root"
        ).value.strip()
        reference_repository = MultiReferenceViewRepository(
            object_root or None
        )
        sensor_repository = SensorRepository(sensor_root or None)
        self.sensor_attachment_controller = SensorAttachmentController(
            sensor_repository,
            SensorAttachmentStateStore(),
            self.command_controller,
        )
        self.sensor_registry_controller = SensorRegistryController(
            sensor_repository,
            self.sensor_attachment_controller,
            self.command_controller,
        )
        self.command_controller.add_request_preparer(
            self.sensor_attachment_controller.prepare_request
        )
        self.sensor_registry_api = SensorRegistryApi(
            self,
            self.sensor_registry_controller,
        )
        self.sensor_attachment_api = SensorAttachmentApi(
            self,
            self.sensor_attachment_controller,
        )
        self.sensor_acquisition_coordinator = SensorAcquisitionCoordinator(
            node=self,
            measurement_repository=MeasurementRepository(
                measurement_root or None
            ),
            sensor_attachment_controller=(
                self.sensor_attachment_controller
            ),
            sensor_repository=sensor_repository,
            error_handler=(
                lambda exception: self.get_logger().error(
                    f"Sensor acquisition callback failed: {exception}"
                )
            ),
        )
        self.sensor_acquisition_coordinator.add_state_listener(self._track_acquisition_ownership)
        self.sensor_acquisition_api = SensorAcquisitionApi(
            self,
            self.sensor_acquisition_coordinator,
        )
        self.sensor_acquisition_command_handler = (
            SensorAcquisitionCommandHandler(
                self,
                self.command_controller,
                self.sensor_acquisition_coordinator,
            )
        )
        self.command_transport.set_local_dispatch(
            self.sensor_acquisition_command_handler.dispatch,
            query_status=self.sensor_acquisition_command_handler.query_status,
        )
        self.command_controller.configure_stop_participant(
            self.sensor_acquisition_command_handler.request_software_stop,
            self.sensor_acquisition_command_handler.software_stop_status,
            reset=self.sensor_acquisition_command_handler.reset_software_stop,
        )
        self.probe_setup_motion_state = (
            probe_setup_motion_state_source.ProbeSetupMotionStateSource(self)
        )
        self.probe_setup_coordinator = ProbeSetupCoordinator(
            setup_coordinator=self.application_controller.setup_coordinator,
            reference_repository=reference_repository,
            sensor_attachment_controller=(
                self.sensor_attachment_controller
            ),
            motion_state_source=self.probe_setup_motion_state,
            map_repository=self.navigation_setup_coordinator.map_repository,
            map_artifacts=self.navigation_setup_coordinator.map_artifacts,
        )
        self.application_controller.attach_probe_setup(
            self.probe_setup_coordinator
        )
        self.probe_point_execution_api = ProbePointExecutionApi(
            self, self.command_controller, self.probe_setup_coordinator,
            self.sensor_acquisition_coordinator,
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
            ProbeReferencePreviewSource(reference_repository),
            admission=self._operation_admission,
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
            admission=self._operation_admission,
        )
        self.probe_setup_motion_api = ProbeSetupMotionApi(
            self,
            self.application_controller.probe_setup_coordinator,
            self.probe_setup_state_publisher,
            self.probe_setup_state_adapter,
            admission=self._operation_admission,
        )
        self.probe_refinement_finalization_api = (
            ProbeRefinementFinalizationApi(
                self,
                self.application_controller.probe_setup_coordinator,
                self.probe_setup_state_publisher,
                self.probe_setup_state_adapter,
                admission=self._operation_admission,
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
        self._reset_software_stop_service = self.create_service(
            ResetSoftwareStop,
            "fault_detector/application/reset_software_stop",
            self._reset_software_stop,
            callback_group=self._callback_group,
        )

    def _accept_operation(self, goal_request):
        if self._shutdown.is_set():
            return GoalResponse.REJECT
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
        return (GoalResponse.ACCEPT if self._operation_admission.reserve(
            id(goal_request), context_id=goal_request.context_id.strip(), lane="physical",
        ) else GoalResponse.REJECT)

    def _accept_cancellation(self, goal_handle):
        self._operation_admission.cancel(id(goal_handle.request))
        with self._lock:
            execution = next(
                (
                    candidate
                    for candidate in self._executions.values()
                    if candidate.goal_handle is goal_handle
                ),
                None,
            )
        if execution is not None:
            self._request_execution_cancellation(execution)
        return CancelResponse.ACCEPT

    @tracked_shutdown_callback
    async def _execute_operation(self, goal_handle):
        key = id(goal_handle.request)
        if not self._operation_admission.claim(key):
            try:
                return self._abort_preparation(goal_handle, "Operational lane is busy or closing")
            finally:
                self._operation_admission.release_pending(key)
        try:
            return await self._execute_admitted_operation(goal_handle)
        finally:
            self._operation_admission.release(key)

    async def _execute_admitted_operation(self, goal_handle):
        goal = goal_handle.request
        if self._shutdown.is_set():
            return self._abort_preparation(goal_handle, "Application is shutting down")
        if (goal_handle.is_cancel_requested
                or self._operation_admission.cancel_requested(id(goal))):
            return self._abort_preparation(goal_handle, "Operation cancelled before dispatch",
                                           cancelled=goal_handle.is_cancel_requested)
        try:
            operation = self.application_controller.prepare_operation(
                intent=goal.intent,
                client_id=goal.client_id,
                context_id=goal.context_id.strip(),
            )
        except Exception as exception:
            return self._abort_preparation(goal_handle, str(exception))
        execution = _OperationExecution(
            goal_handle=goal_handle,
            operation=operation,
        )
        with self._lock:
            self._executions[operation.request_id] = execution
        try:
            self.application_controller.submit(operation)
        except Exception as exception:
            with self._lock:
                if not execution.finished.done():
                    state = self._failure_state(execution, str(exception))
                    execution.finished.set_result(state)
        with self._lock:
            execution.submitted = True

        if (not self._shutdown.is_set() and (goal_handle.is_cancel_requested
                or self._operation_admission.cancel_requested(id(goal)))):
            self._request_execution_cancellation(execution)

        try:
            state = await execution.finished
        finally:
            with self._lock:
                self._executions.pop(operation.request_id, None)
        result = ExecuteOperation.Result()
        result.state = state
        if state.state == ApplicationCommandState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif (
            state.state in {
                ApplicationCommandState.STATE_CANCELLED,
                ApplicationCommandState.STATE_STOP_CONFIRMED,
            }
            and goal_handle.is_cancel_requested
        ):
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _abort_preparation(self, goal_handle, detail, *, cancelled=False):
        """Report preparation failure before a command request exists."""
        goal = goal_handle.request
        state = ApplicationCommandState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.request_id = uuid4().hex
        state.client_id = goal.client_id.strip()
        state.context_id = goal.context_id.strip()
        state.intent = int(goal.intent.intent)
        state.state = (ApplicationCommandState.STATE_CANCELLED if cancelled
                       else ApplicationCommandState.STATE_FAILED)
        state.detail = detail
        self._state_publisher.publish(state)
        result = ExecuteOperation.Result()
        result.state = state
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _request_execution_cancellation(self, execution):
        with self._lock:
            if (self._shutdown.is_set() or not execution.submitted or execution.cancellation_sent
                    or execution.finished.done()):
                return
            execution.cancellation_sent = True
        try:
            self.application_controller.cancel(
                execution.operation.request.client_id,
                execution.operation.request_id,
            )
        except UnknownCommandRequest:
            with self._lock:
                execution.cancellation_sent = False
        except Exception as exception:
            # A failed cancellation transport does not finish the physical
            # request. Keep its callback and admission until owner completion.
            with self._lock:
                execution.cancellation_sent = False
            self.get_logger().error(f"Cancellation failed; stop unconfirmed: {exception}")

    def _cancel_operation(self, request, response):
        try:
            if self._shutdown.is_set():
                raise RuntimeError("Application is shutting down")
            cancellation_id = self.application_controller.cancel(
                request.client_id,
                request.request_id,
            )
        except (TypeError, ValueError, RuntimeError, UnknownCommandRequest) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.cancellation_request_id = cancellation_id
        response.detail = "Cancellation accepted"
        return response

    def _emergency_stop(self, request, response):
        try:
            if self._shutdown.is_set():
                raise RuntimeError("Application is shutting down")
            request_id = self.application_controller.emergency_stop(
                request.client_id
            )
        except (TypeError, ValueError, RuntimeError) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.request_id = request_id
        response.detail = "Software stop latched; stop confirmation remains pending"
        return response

    def _reset_software_stop(self, request, response):
        try:
            if self._shutdown.is_set():
                raise RuntimeError("Application is shutting down")
            if self._startup_unresolved:
                raise RuntimeError("Application API has no session ownership; reset is unavailable")
            request_id = self.application_controller.reset_software_stop(
                request.client_id
            )
        except (TypeError, ValueError, RuntimeError) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.request_id = request_id
        response.detail = "Software-stop reset requested; awaiting owner confirmation. No motion or replay requested."
        return response

    def _handle_application_status(
        self,
        status: ApplicationOperationStatus,
    ) -> None:
        state = self._state_message(status)
        with self._lock:
            execution = self._executions.get(
                status.operation.request_id
            )
            active = execution is not None and not execution.finished.done()
            if active:
                if status.state in _TERMINAL_STATES:
                    execution.finished.set_result(state)
        try:
            self._state_publisher.publish(state)
        except Exception as exception:
            self.get_logger().error(f"Operational state publication failed: {exception}")
        if not active:
            return
        feedback = ExecuteOperation.Feedback()
        feedback.state = state
        try:
            execution.goal_handle.publish_feedback(feedback)
        except Exception as exception:
            self.get_logger().error(f"Operational feedback publication failed: {exception}")

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
        message.execution_instance_id = status.execution_instance_id
        message.status_sequence = status.status_sequence
        return message

    def _failure_state(self, execution, detail, *, public_state=ApplicationCommandState.STATE_FAILED):
        message = ApplicationCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        operation = execution.operation
        message.request_id = operation.request_id
        message.client_id = operation.request.client_id
        message.context_id = operation.request.context_id
        message.intent = operation.intent
        message.state = public_state
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
            CommandControllerState.FAILED: ApplicationCommandState.STATE_FAILED,
            CommandControllerState.CANCELLED: (
                ApplicationCommandState.STATE_CANCELLED
            ),
            CommandControllerState.UNKNOWN: ApplicationCommandState.STATE_UNKNOWN,
            CommandControllerState.STOP_REQUESTED: ApplicationCommandState.STATE_STOP_REQUESTED,
            CommandControllerState.STOP_CONFIRMED: ApplicationCommandState.STATE_STOP_CONFIRMED,
            CommandControllerState.STOP_UNCONFIRMED: ApplicationCommandState.STATE_STOP_UNCONFIRMED,
        }
        return values[state]

    def request_shutdown(self):
        """Close local ownership without sending cancellation to the robot."""
        with self._shutdown_lock:
            self._close_operation_admission()
            errors = self._finish_shutdown_operations(
                "Application closed without cancellation; dispatched execution remains unresolved",
                public_state=ApplicationCommandState.STATE_UNKNOWN,
            )
            if errors:
                raise RuntimeError("; ".join(errors))

    def _track_acquisition_ownership(self, _state):
        self.command_controller.set_external_session_work(
            not self.sensor_acquisition_coordinator.recording_stopped,
        )

    def _close_operation_admission(self):
        self._shutdown.set()
        self._operation_admission.close()
        self.command_controller.close_admission()

    def _command_shutdown_status(self):
        return self.command_controller.shutdown_status()

    def _report_shutdown_unconfirmed(self, unresolved):
        detail = "; ".join(f"{name}: {reason}" for name, reason in unresolved.items())
        errors = []
        try:
            if not self._startup_unresolved:
                self._shutdown_store.record({name: reason for name, reason in unresolved.items()
                                             if not name.startswith("prior shutdown/")})
        except Exception as error:
            errors.append(f"persist shutdown uncertainty: {error}")
        try:
            self.get_logger().error(f"Application closed with unresolved ownership: {detail}")
        except Exception as error:
            errors.append(f"log shutdown uncertainty: {error}")
        errors.extend(self._finish_shutdown_operations(
            f"Application closed with unresolved ownership: {detail}",
            public_state=ApplicationCommandState.STATE_STOP_UNCONFIRMED,
        ))
        if errors:
            raise RuntimeError("; ".join(errors))

    def _finish_shutdown_operations(self, detail, *, public_state):
        # Finish public actions explicitly without inventing a physical stop.
        errors = []
        with self._lock:
            for execution in tuple(self._executions.values()):
                if execution.finished.done():
                    continue
                try:
                    state = self._failure_state(
                        execution, detail, public_state=public_state,
                    )
                except Exception as error:
                    errors.append(f"publish shutdown uncertainty: {error}")
                    state = ApplicationCommandState(
                        state=public_state, detail=detail,
                    )
                execution.finished.set_result(state)
        return errors

    def _confirm_shutdown_persistence(self):
        coordinator = self._shutdown_coordinator
        if self._startup_unresolved or coordinator.unresolved:
            return  # Preserve diagnostics for archival on the next fresh launch.
        errors = {name: detail for name, detail in coordinator.cleanup_errors.items()
                  if name != "confirm shutdown persistence"}
        if errors:
            raise RuntimeError("Shutdown session retained until failed cleanup is retried")
        self._shutdown_store.confirm_session()

    def shutdown(self, executor=None):
        """Keep callbacks and state owners available through bounded confirmation."""
        with self._shutdown_lock:
            if self._shutdown_coordinator is None:
                setup_owners = {
                    name: getattr(self, name) for name in (
                        "probe_reference_capture_api", "probe_refinement_finalization_api",
                        "probe_setup_motion_api", "probe_setup_api", "navigation_setup_api",
                    )
                }
                admission = {"application": self._close_operation_admission}
                admission.update({name: owner.request_shutdown for name, owner in setup_owners.items()})
                confirmations = {
                    "acquisition": self.sensor_acquisition_coordinator.shutdown_status,
                    "operations": self._shutdown_callbacks.status,
                    "accepted operations": self._operation_admission.shutdown_status,
                }
                confirmations.update({name: owner._shutdown_callbacks.status
                                      for name, owner in setup_owners.items()})
                cleanup = {
                    "recording": lambda: self.sensor_acquisition_coordinator.close(
                        stop_unconfirmed=bool(self._shutdown_coordinator.unresolved),
                    ),
                    "acquisition ownership listener": lambda: self.sensor_acquisition_coordinator.remove_state_listener(
                        self._track_acquisition_ownership,
                    ),
                }
                for name in (
                    "probe_reference_capture_api", "probe_refinement_finalization_api",
                    "probe_setup_motion_api", "probe_setup_api", "navigation_setup_api",
                    "probe_point_execution_api", "sensor_acquisition_command_handler",
                    "sensor_acquisition_api", "sensor_attachment_api", "sensor_registry_api",
                ):
                    cleanup[name] = getattr(self, name).close
                cleanup.update({
                    "request_preparer": lambda: self.command_controller.remove_request_preparer(
                        self.sensor_attachment_controller.prepare_request,
                    ),
                    "status_listener": lambda: self.application_controller.remove_status_listener(
                        self._handle_application_status,
                    ),
                    "application_controller": self.application_controller.close,
                    "command_transport": self.command_transport.close,
                    "probe_setup_state": self.probe_setup_motion_state.close,
                    "navigation_setup_state": self.navigation_setup_state.close,
                    "operation_server": self._operation_server.destroy,
                    "node": super().destroy_node,
                })
                if executor is not None:
                    cleanup["executor"] = lambda: executor.shutdown(timeout_sec=1.0)
                    cleanup["ros"] = rclpy.try_shutdown
                cleanup["confirm shutdown persistence"] = self._confirm_shutdown_persistence
                self._shutdown_coordinator = ShutdownCoordinator(
                    close_admission=admission,
                    request_stop={"commands": self.request_shutdown,
                                  "acquisition": self.sensor_acquisition_coordinator.request_shutdown},
                    confirmations=confirmations,
                    passive_confirmations={"commands": self._command_shutdown_status},
                    retained_unresolved={f"prior shutdown/{name}": detail
                                         for name, detail in self._startup_unresolved.items()},
                    cleanup=cleanup,
                    progress=(lambda timeout: executor.spin_once(timeout_sec=timeout))
                    if executor is not None else Event().wait,
                    report_unconfirmed=self._report_shutdown_unconfirmed,
                    timeout_sec=float(self.get_parameter("shutdown.confirmation_timeout_sec").value),
                    cleanup_timeout_sec=float(self.get_parameter("shutdown.cleanup_timeout_sec").value),
                )
        return self._shutdown_coordinator.close()

    def destroy_node(self):
        return self.shutdown()


def main(args=None):
    with shutdown_signals() as stopping:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        try:
            node = ApplicationApiNode()
        except Exception:
            rclpy.try_shutdown()
            raise
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            while rclpy.ok() and not stopping.is_set():
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            report = node.shutdown(executor)
            if report.cleanup_errors:
                print(f"Application shutdown cleanup incomplete: {report.cleanup_errors}", file=sys.stderr)


if __name__ == "__main__":
    main()
