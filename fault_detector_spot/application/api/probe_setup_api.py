"""Expose probe authoring through typed ROS service interfaces."""

from uuid import uuid4

from fault_detector_msgs.msg import (
    ProbeSetupIntent,
    ProbeSetupState,
)
from fault_detector_msgs.srv import (
    CloseProbeSetup,
    ExecuteProbeSetup,
    GetProbeReferencePreview,
)
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.setup.probe_reference_preview import (
    ProbeReferencePreviewSource,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)


class ProbeSetupApi:
    """Own ROS transport for one probe setup coordinator."""

    def __init__(
        self,
        node,
        coordinator: ProbeSetupCoordinator,
        state_publisher,
        state_adapter: ProbeSetupStateAdapter,
        preview_source=None,
    ):
        self.node = node
        self.coordinator = coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self.preview_source = preview_source or ProbeReferencePreviewSource(
            coordinator.reference_repository
        )
        self._callback_group = ReentrantCallbackGroup()
        self._handlers = self._transaction_handlers()
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
        self.state_publisher.publish(state)
        response.state = state
        return response

    def _success_state(self, snapshot, operation):
        opened = operation == ProbeSetupIntent.OPERATION_OPEN
        return self.state_adapter.message(
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

    def _approve_safe_pose(self, context, _intent):
        return self.coordinator.approve_safe_pose(context)

    def _approve_aligned_pose(self, context, _intent):
        return self.coordinator.approve_aligned_pose(context)

    def _begin_refinement(self, context, _intent):
        return self.coordinator.begin_refinement(context)

    def _end_refinement(self, context, _intent):
        return self.coordinator.end_refinement(context)

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
            state = self.state_adapter.message(
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
        self.state_publisher.publish(state)
        return state

    def _validate_operation(self, operation: int) -> None:
        supported = set(self._handlers)
        supported.add(ProbeSetupIntent.OPERATION_OPEN)
        if int(operation) not in supported:
            raise ValueError(f"Unsupported probe setup operation: {operation}")

    def close(self) -> None:
        """Destroy transport resources owned by this API."""
        self.node.destroy_service(self._execute_service)
        self.node.destroy_service(self._close_service)
        self.node.destroy_service(self._preview_service)


__all__ = ["ProbeSetupApi"]
