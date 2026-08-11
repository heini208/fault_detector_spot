"""Expose probe authoring and motion through typed ROS interfaces."""

from dataclasses import dataclass, field
from threading import Event, RLock
from uuid import uuid4

from fault_detector_msgs.action import ExecuteProbeSetupMotion
from fault_detector_msgs.msg import (
    ProbeSetupIntent,
    ProbeSetupMotionIntent,
    ProbeSetupState,
)
from fault_detector_msgs.srv import (
    CloseProbeSetup,
    ExecuteProbeSetup,
    GetProbeReferencePreview,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.inspection.model.models import ImagePoint, Vector3Data
from fault_detector_spot.inspection.setup.probe_setup_coordinator import (
    ProbeSetupCoordinator,
    ProbeSetupMotionStatus,
)
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_reference_preview import (
    ProbeReferencePreviewSource,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionFrame,
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.shared.geometry.transforms import (
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


@dataclass
class _ProbeMotionExecution:
    goal_handle: object
    request_id: str
    context: object
    finished: Event = field(default_factory=Event)
    state: object = None
    cancellation_requested: bool = False


class ProbeSetupApi:
    """Own ROS transport for one probe setup coordinator."""

    def __init__(
        self,
        node,
        coordinator: ProbeSetupCoordinator,
        preview_source=None,
    ):
        self.node = node
        self.coordinator = coordinator
        self._lock = RLock()
        self._motion_executions = {}
        self.preview_source = preview_source or ProbeReferencePreviewSource(
            coordinator.reference_repository
        )
        self._callback_group = ReentrantCallbackGroup()
        self._handlers = self._transaction_handlers()
        self._state_publisher = node.create_publisher(
            ProbeSetupState,
            "fault_detector/application/probe_setup_state",
            APPLICATION_STATE_QOS,
        )
        self._execute_service = node.create_service(
            ExecuteProbeSetup,
            "fault_detector/application/execute_probe_setup",
            self._execute,
            callback_group=self._callback_group,
        )
        self._close_service = node.create_service(
            CloseProbeSetup,
            "fault_detector/application/close_probe_setup",
            self._close,
            callback_group=self._callback_group,
        )
        self._preview_service = node.create_service(
            GetProbeReferencePreview,
            "fault_detector/application/get_probe_reference_preview",
            self._preview,
            callback_group=self._callback_group,
        )
        self._motion_action_server = ActionServer(
            node,
            ExecuteProbeSetupMotion,
            "fault_detector/application/execute_probe_setup_motion",
            execute_callback=self._execute_motion,
            goal_callback=self._accept_motion,
            cancel_callback=self._accept_motion_cancel,
            callback_group=self._callback_group,
        )
        coordinator.add_motion_status_listener(
            self._receive_motion_status
        )

    def _transaction_handlers(self):
        return {
            ProbeSetupIntent.OPERATION_REFRESH: self._refresh,
            ProbeSetupIntent.OPERATION_SELECT_OBJECT: self._select_object,
            ProbeSetupIntent.OPERATION_SELECT_ROUTINE: self._select_routine,
            ProbeSetupIntent.OPERATION_CREATE_OBJECT: self._create_object,
            ProbeSetupIntent.OPERATION_DELETE_OBJECT: self._delete_object,
            ProbeSetupIntent.OPERATION_CREATE_ROUTINE: self._create_routine,
            ProbeSetupIntent.OPERATION_DELETE_ROUTINE: self._delete_routine,
            ProbeSetupIntent.OPERATION_SELECT_REFERENCE_PIXEL: (
                self._select_reference_pixel
            ),
            ProbeSetupIntent.OPERATION_CLEAR_REFERENCE_PIXEL: (
                self._clear_reference_pixel
            ),
            ProbeSetupIntent.OPERATION_UPDATE_GEOMETRY: (
                self._update_geometry
            ),
            ProbeSetupIntent.OPERATION_APPROVE_SAFE_POSE: (
                self._approve_safe_pose
            ),
            ProbeSetupIntent.OPERATION_APPROVE_ALIGNED_POSE: (
                self._approve_aligned_pose
            ),
            ProbeSetupIntent.OPERATION_APPROVE_PROBE_POSE: (
                self._approve_probe_pose
            ),
            ProbeSetupIntent.OPERATION_SAVE_PROBE_POINT: (
                self._save_probe_point
            ),
            ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT: (
                self._begin_refinement
            ),
            ProbeSetupIntent.OPERATION_END_REFINEMENT: (
                self._end_refinement
            ),
        }

    def _execute(self, request, response):
        goal = request
        operation = int(goal.intent.operation)
        context = None
        try:
            required_client_id(goal.client_id)
            self._validate_operation(operation)
            if operation == ProbeSetupIntent.OPERATION_OPEN:
                snapshot = self.coordinator.open_context(goal.client_id)
            else:
                context = self.coordinator.context(
                    goal.context_id,
                    goal.client_id,
                )
                snapshot = self._handlers[operation](
                    context,
                    goal.intent,
                )
        except Exception as exception:
            response.state = self._failure_state(
                goal,
                str(exception),
                context,
            )
            return response
        state = self._success_state(snapshot, operation)
        self._state_publisher.publish(state)
        response.state = state
        return response

    def _success_state(self, snapshot, operation):
        opened = operation == ProbeSetupIntent.OPERATION_OPEN
        return self._state(
            snapshot,
            operation,
            ProbeSetupState.STATE_READY
            if opened
            else ProbeSetupState.STATE_SUCCEEDED,
            "Probe setup opened"
            if opened
            else "Probe setup transaction succeeded",
        )

    def _refresh(self, context, _intent):
        return self.coordinator.refresh(context)

    def _select_routine(self, context, intent):
        return self.coordinator.select_routine(
            context,
            intent.object_id,
            intent.routine_id,
        )

    def _select_object(self, context, intent):
        return self.coordinator.select_object(
            context,
            intent.object_id,
        )

    def _create_object(self, context, intent):
        return self.coordinator.create_object(
            context,
            intent.object_id,
            intent.object_display_name,
            int(intent.reference_tag_id),
            intent.reference_tag_family,
        )

    def _delete_object(self, context, intent):
        return self.coordinator.delete_object(context, intent.object_id)

    def _create_routine(self, context, intent):
        return self.coordinator.create_routine(
            context,
            intent.object_id,
            intent.routine_id,
            intent.routine_display_name,
            intent.sensor_id,
        )

    def _delete_routine(self, context, intent):
        return self.coordinator.delete_routine(
            context,
            intent.object_id,
            intent.routine_id,
        )

    def _select_reference_pixel(self, context, intent):
        return self.coordinator.select_reference_pixel(
            context,
            intent.reference_view_id,
            ImagePoint(u=int(intent.pixel_u), v=int(intent.pixel_v)),
            intent.approach_mode,
            float(intent.target_surface_distance_m),
            float(intent.aligned_preapproach_distance_m),
        )

    def _clear_reference_pixel(self, context, _intent):
        return self.coordinator.clear_reference_pixel(context)

    def _update_geometry(self, context, intent):
        return self.coordinator.update_geometry(
            context,
            intent.approach_mode,
            float(intent.target_surface_distance_m),
            float(intent.aligned_preapproach_distance_m),
        )

    def _approve_safe_pose(self, context, intent):
        return self.coordinator.approve_safe_pose(context)

    def _approve_aligned_pose(self, context, intent):
        return self.coordinator.approve_aligned_pose(context)

    def _approve_probe_pose(self, context, intent):
        return self.coordinator.approve_probe_pose(context)

    def _begin_refinement(self, context, _intent):
        return self.coordinator.begin_refinement(context)

    def _end_refinement(self, context, _intent):
        return self.coordinator.end_refinement(context)

    def _save_probe_point(self, context, intent):
        return self.coordinator.save_probe_point(
            context,
            intent.probe_point_id,
            intent.probe_point_display_name,
            float(intent.position_tolerance_m),
            float(intent.orientation_tolerance_rad),
            float(intent.measurement_duration_sec),
        )

    def _accept_motion(self, goal_request):
        try:
            required_client_id(goal_request.client_id)
            self._motion_request(goal_request.intent)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_motion_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_motion(self, goal_handle):
        goal = goal_handle.request
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            motion = self._motion_request(goal.intent)
            operation = self.coordinator.prepare_motion(context, motion)
        except Exception as exception:
            return self._abort_motion(
                goal_handle,
                goal,
                str(exception),
            )
        execution = _ProbeMotionExecution(
            goal_handle=goal_handle,
            request_id=operation.request_id,
            context=context,
        )
        with self._lock:
            self._motion_executions[operation.request_id] = execution
        try:
            self.coordinator.submit_motion(operation)
        except Exception as exception:
            with self._lock:
                self._motion_executions.pop(operation.request_id, None)
            return self._abort_motion(
                goal_handle,
                goal,
                str(exception),
            )
        while not execution.finished.wait(0.05):
            if (
                goal_handle.is_cancel_requested
                and not execution.cancellation_requested
            ):
                execution.cancellation_requested = True
                try:
                    self.coordinator.cancel_motion(
                        context,
                        operation.request_id,
                    )
                except LookupError:
                    pass
        with self._lock:
            self._motion_executions.pop(operation.request_id, None)
        result = ExecuteProbeSetupMotion.Result()
        result.state = execution.state
        if execution.state.state == ProbeSetupState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif execution.state.state == ProbeSetupState.STATE_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _receive_motion_status(
        self,
        status: ProbeSetupMotionStatus,
    ) -> None:
        state = self._state(
            status.snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            self._public_motion_state(status.state),
            status.detail,
            request_id=status.request_id,
            motion_operation=self._motion_operation(status.motion.kind),
        )
        self._state_publisher.publish(state)
        with self._lock:
            execution = self._motion_executions.get(status.request_id)
        if execution is None:
            return
        execution.state = state
        feedback = ExecuteProbeSetupMotion.Feedback()
        feedback.state = state
        execution.goal_handle.publish_feedback(feedback)
        if status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }:
            execution.finished.set()

    def _abort_motion(self, goal_handle, goal, detail):
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            snapshot = self.coordinator.snapshot(context)
            state = self._state(
                snapshot,
                ProbeSetupIntent.OPERATION_UNSPECIFIED,
                ProbeSetupState.STATE_FAILED,
                detail,
                motion_operation=int(goal.intent.operation),
            )
        except Exception:
            state = ProbeSetupState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.request_id = uuid4().hex
            state.client_id = goal.client_id.strip()
            state.context_id = goal.context_id.strip()
            state.motion_operation = int(goal.intent.operation)
            state.state = ProbeSetupState.STATE_FAILED
            state.detail = detail
            state.validation_error = detail
        self._state_publisher.publish(state)
        result = ExecuteProbeSetupMotion.Result()
        result.state = state
        goal_handle.abort()
        return result

    @staticmethod
    def _motion_request(intent):
        kinds = {
            ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH: (
                ProbeMotionKind.MOVE_SAFE_APPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH: (
                ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_ADJUST_SAFE_APPROACH: (
                ProbeMotionKind.ADJUST_SAFE_APPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH: (
                ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH
            ),
        }
        frames = {
            ProbeSetupMotionIntent.FRAME_SENSOR: ProbeMotionFrame.SENSOR,
            ProbeSetupMotionIntent.FRAME_HAND: ProbeMotionFrame.HAND,
            ProbeSetupMotionIntent.FRAME_TAG: ProbeMotionFrame.TAG,
            ProbeSetupMotionIntent.FRAME_BODY: ProbeMotionFrame.BODY,
            ProbeSetupMotionIntent.FRAME_MAP: ProbeMotionFrame.MAP,
        }
        try:
            kind = kinds[int(intent.operation)]
            frame = frames[int(intent.frame)]
        except KeyError as exception:
            raise ValueError(
                "Unsupported probe setup motion intent"
            ) from exception
        request = ProbeMotionRequest(
            kind=kind,
            frame=frame,
            translation=Vector3Data(
                x=float(intent.translation.x),
                y=float(intent.translation.y),
                z=float(intent.translation.z),
            ),
            pitch_rad=float(intent.pitch_rad),
            yaw_rad=float(intent.yaw_rad),
            position_tolerance_m=float(intent.position_tolerance_m),
            orientation_tolerance_rad=float(
                intent.orientation_tolerance_rad
            ),
        )
        request.validate()
        return request

    @staticmethod
    def _motion_operation(kind):
        return {
            ProbeMotionKind.MOVE_SAFE_APPROACH: (
                ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH
            ),
            ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH: (
                ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH
            ),
            ProbeMotionKind.ADJUST_SAFE_APPROACH: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_SAFE_APPROACH
            ),
            ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH
            ),
        }[kind]

    @staticmethod
    def _public_motion_state(state):
        return {
            CommandControllerState.QUEUED: ProbeSetupState.STATE_QUEUED,
            CommandControllerState.DISPATCHED: ProbeSetupState.STATE_RUNNING,
            CommandControllerState.RUNNING: ProbeSetupState.STATE_RUNNING,
            CommandControllerState.SUCCEEDED: ProbeSetupState.STATE_SUCCEEDED,
            CommandControllerState.FAILED: ProbeSetupState.STATE_FAILED,
            CommandControllerState.CANCELLED: ProbeSetupState.STATE_CANCELLED,
        }[state]

    def _close(self, request, response):
        try:
            context = self.coordinator.context(
                request.context_id,
                request.client_id,
            )
            self.coordinator.close_context(context)
        except (TypeError, ValueError, LookupError, RuntimeError) as exception:
            response.closed = False
            response.detail = str(exception)
            return response
        response.closed = True
        response.detail = "Probe setup closed"
        return response

    def _preview(self, request, response):
        try:
            context = self.coordinator.context(
                request.context_id,
                request.client_id,
            )
            snapshot = self.coordinator.snapshot(context)
            preview = self.preview_source.load(
                snapshot,
                request.reference_view_id,
            )
        except Exception as exception:
            response.success = False
            response.detail = str(exception)
            return response
        region = preview.selectable_region
        response.success = True
        response.detail = "Reference preview loaded"
        response.reference_view_id = preview.reference_view_id
        response.camera_id = preview.camera_id
        response.slot_index = preview.slot_index
        response.image = preview.image
        response.selectable_x = region.x
        response.selectable_y = region.y
        response.selectable_width = region.width
        response.selectable_height = region.height
        return response

    def _failure_state(self, goal, detail, context=None):
        if (
            context is not None
            and self.coordinator.setup_coordinator.is_current(context)
        ):
            snapshot = self.coordinator.snapshot(context)
            state = self._state(
                snapshot,
                int(goal.intent.operation),
                ProbeSetupState.STATE_FAILED,
                detail,
            )
        else:
            state = ProbeSetupState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.request_id = uuid4().hex
            state.client_id = goal.client_id.strip()
            state.context_id = goal.context_id.strip()
            state.operation = int(goal.intent.operation)
            state.state = ProbeSetupState.STATE_FAILED
            state.detail = detail
            state.validation_error = detail
        self._state_publisher.publish(state)
        return state

    def _state(
        self,
        snapshot: ProbeSetupSnapshot,
        operation: int,
        state_code: int,
        detail: str,
        request_id: str = "",
        motion_operation: int = 0,
    ) -> ProbeSetupState:
        message = ProbeSetupState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = request_id or uuid4().hex
        message.client_id = snapshot.context.client_id
        message.context_id = snapshot.context.context_id
        message.revision = snapshot.context.revision
        message.operation = int(operation)
        message.motion_operation = int(motion_operation)
        message.state = int(state_code)
        message.detail = detail
        message.validation_error = snapshot.validation_error
        message.dirty = snapshot.dirty
        message.selected_object_id = snapshot.selected_object_id
        message.selected_routine_id = snapshot.selected_routine_id
        message.selected_reference_view_id = (
            snapshot.selected_reference_view_id
        )
        message.selected_reference_tag_id = (
            snapshot.selected_reference_tag_id
        )
        message.selected_reference_tag_family = (
            snapshot.selected_reference_tag_family
        )
        message.selected_sensor_id = snapshot.selected_sensor_id
        message.object_ids = list(snapshot.object_ids)
        message.routine_ids = list(snapshot.routine_ids)
        message.reference_view_ids = list(snapshot.reference_view_ids)
        message.reference_camera_ids = list(snapshot.reference_camera_ids)
        message.sensor_ids = list(snapshot.sensor_ids)
        message.probe_point_ids = list(snapshot.probe_point_ids)
        self._write_geometry(message, snapshot)
        self._write_refinement(message, snapshot)
        return message

    @staticmethod
    def _write_refinement(message, snapshot):
        refinement = snapshot.refinement
        if refinement is None:
            return
        message.refinement_active = True
        stages = {
            RefinementStage.SAFE_APPROACH: (
                ProbeSetupState.REFINEMENT_STAGE_SAFE_APPROACH
            ),
            RefinementStage.ALIGNMENT: (
                ProbeSetupState.REFINEMENT_STAGE_ALIGNMENT
            ),
            RefinementStage.PROBE: ProbeSetupState.REFINEMENT_STAGE_PROBE,
        }
        message.refinement_stage = stages[refinement.active_stage]
        message.safe_approach_candidate_pose_object = pose_data_to_pose(
            refinement.candidate_pose(RefinementStage.SAFE_APPROACH)
        )
        message.aligned_preapproach_candidate_pose_object = (
            pose_data_to_pose(
                refinement.candidate_pose(RefinementStage.ALIGNMENT)
            )
        )
        message.probe_candidate_pose_object = pose_data_to_pose(
            refinement.candidate_pose(RefinementStage.PROBE)
        )
        states = {
            RefinementMotionState.NOT_TESTED: (
                ProbeSetupState.MOTION_NOT_TESTED
            ),
            RefinementMotionState.MOVING: ProbeSetupState.MOTION_MOVING,
            RefinementMotionState.REACHED: ProbeSetupState.MOTION_REACHED,
            RefinementMotionState.FAILED: ProbeSetupState.MOTION_FAILED,
        }
        message.safe_approach_motion_state = states[
            refinement.motion_states[RefinementStage.SAFE_APPROACH]
        ]
        message.alignment_motion_state = states[
            refinement.motion_states[RefinementStage.ALIGNMENT]
        ]
        message.probe_motion_state = states[
            refinement.motion_states[RefinementStage.PROBE]
        ]
        pending = refinement.pending_motion
        if pending is not None:
            message.motion_pending = True
            message.motion_request_id = pending.request_id

    @staticmethod
    def _write_geometry(message, snapshot):
        ProbeSetupApi._write_reference_geometry(message, snapshot)
        ProbeSetupApi._write_probe_setup(message, snapshot)

    @staticmethod
    def _write_reference_geometry(message, snapshot):
        if snapshot.reference_pixel is not None:
            message.has_reference_pixel = True
            message.reference_pixel_u = snapshot.reference_pixel.u
            message.reference_pixel_v = snapshot.reference_pixel.v
        geometry = snapshot.geometry
        if geometry is None:
            return
        ProbeSetupApi._write_projected_point(
            message,
            geometry.projected_point,
        )
        ProbeSetupApi._write_normal_and_approach(message, geometry)

    @staticmethod
    def _write_projected_point(message, projected):
        message.has_surface_point = True
        message.surface_point_camera.x = projected.point_camera.x
        message.surface_point_camera.y = projected.point_camera.y
        message.surface_point_camera.z = projected.point_camera.z
        message.surface_frame = projected.frame_id
        message.mapped_depth_u = projected.mapped_pixel.u
        message.mapped_depth_v = projected.mapped_pixel.v
        message.sampled_depth_u = projected.sampled_pixel.u
        message.sampled_depth_v = projected.sampled_pixel.v
        message.depth_m = projected.depth_m

    @staticmethod
    def _write_normal_and_approach(message, geometry):
        message.surface_normal_error = geometry.surface_normal_error
        if geometry.surface_normal is not None:
            normal = geometry.surface_normal
            message.has_surface_normal = True
            message.surface_normal_camera.x = normal.normal_camera.x
            message.surface_normal_camera.y = normal.normal_camera.y
            message.surface_normal_camera.z = normal.normal_camera.z
            message.surface_normal_sample_count = normal.sample_count
            message.surface_normal_rmse_m = normal.plane_rmse_m
        direction = geometry.approach_direction
        message.has_approach_direction = True
        message.approach_direction_camera.x = direction.direction_camera.x
        message.approach_direction_camera.y = direction.direction_camera.y
        message.approach_direction_camera.z = direction.direction_camera.z
        message.approach_source = direction.source

    @staticmethod
    def _write_probe_setup(message, snapshot):
        setup = snapshot.setup
        if setup is None:
            return
        message.has_probe_setup = True
        target = setup.surface_target
        ProbeSetupApi._write_surface_target(message, target)
        calculated = snapshot.geometry.probe_setup
        message.calculated_safe_approach_pose_object = pose_data_to_pose(
            calculated.safe_approach_pose_object
        )
        message.calculated_aligned_preapproach_pose_object = (
            pose_data_to_pose(
                calculated.aligned_preapproach_pose_object
            )
        )
        message.calculated_probe_pose_object = pose_data_to_pose(
            calculated.probe_pose_object
        )
        ProbeSetupApi._write_approved_poses(message, setup)
        message.target_surface_distance_m = (
            target.target_surface_distance_m
        )
        message.aligned_preapproach_distance_m = (
            target.aligned_preapproach_distance_m
        )

    @staticmethod
    def _write_surface_target(message, target):
        message.surface_point_object.x = target.surface_point_object.x
        message.surface_point_object.y = target.surface_point_object.y
        message.surface_point_object.z = target.surface_point_object.z
        message.outward_direction_object.x = (
            target.outward_direction_object.x
        )
        message.outward_direction_object.y = (
            target.outward_direction_object.y
        )
        message.outward_direction_object.z = (
            target.outward_direction_object.z
        )

    @staticmethod
    def _write_approved_poses(message, setup):
        message.safe_approach_pose_object = pose_data_to_pose(
            setup.safe_approach_pose_object
        )
        message.aligned_preapproach_pose_object = pose_data_to_pose(
            setup.aligned_preapproach_pose_object
        )
        message.probe_pose_object = pose_data_to_pose(
            setup.probe_pose_object
        )
        message.safe_approach_approved = setup.safe_approach_approved
        message.surface_alignment_approved = (
            setup.surface_alignment_approved
        )
        message.probe_pose_approved = setup.probe_pose_approved

    @staticmethod
    def _validate_operation(operation: int) -> None:
        supported = {
            value
            for name, value in vars(ProbeSetupIntent).items()
            if name.startswith("OPERATION_")
            and name != "OPERATION_UNSPECIFIED"
        }
        if int(operation) not in supported:
            raise ValueError(f"Unsupported probe setup operation: {operation}")

    def close(self) -> None:
        """Destroy transport resources owned by this API."""
        self.coordinator.remove_motion_status_listener(
            self._receive_motion_status
        )
        self._motion_action_server.destroy()
        self.node.destroy_service(self._execute_service)
        self.node.destroy_service(self._close_service)
        self.node.destroy_service(self._preview_service)
        with self._lock:
            self._motion_executions.clear()


__all__ = ["ProbeSetupApi"]
