"""Expose authoritative sensor attachment state through ROS services."""

from fault_detector_msgs.msg import SensorAttachmentState as StateMessage
from fault_detector_msgs.srv import (
    ConfirmSensorAttachment,
    SelectSensorAttachment,
)
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentStatus,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


class SensorAttachmentApi:
    """Own ROS transport for physical sensor selection and confirmation."""

    STATE_TOPIC = "fault_detector/application/sensor_attachment_state"
    SELECT_SERVICE = "fault_detector/application/select_sensor_attachment"
    CONFIRM_SERVICE = "fault_detector/application/confirm_sensor_attachment"

    def __init__(self, node, controller):
        self.node = node
        self.controller = controller
        self._callback_group = ReentrantCallbackGroup()
        self._publisher = node.create_publisher(
            StateMessage,
            self.STATE_TOPIC,
            APPLICATION_STATE_QOS,
        )
        self._select_service = node.create_service(
            SelectSensorAttachment,
            self.SELECT_SERVICE,
            self._select,
            callback_group=self._callback_group,
        )
        self._confirm_service = node.create_service(
            ConfirmSensorAttachment,
            self.CONFIRM_SERVICE,
            self._confirm,
            callback_group=self._callback_group,
        )
        self.publish_state(controller.snapshot())

    def publish_state(self, state) -> StateMessage:
        """Publish and return the public attachment state."""
        message = self._message(state)
        self._publisher.publish(message)
        return message

    def _select(self, request, response):
        try:
            state = self.controller.select_sensor(request.sensor_id)
        except Exception as exception:
            response.success = False
            response.detail = str(exception)
            response.state = self._message(self.controller.snapshot())
            return response
        response.success = True
        response.detail = (
            f"Selected sensor '{state.pending_sensor_id}'; "
            "physical confirmation is required"
        )
        response.state = self.publish_state(state)
        return response

    def _confirm(self, request, response):
        try:
            state = self.controller.confirm_sensor(
                request.sensor_id,
                int(request.attachment_revision),
            )
        except Exception as exception:
            response.success = False
            response.detail = str(exception)
            response.state = self._message(self.controller.snapshot())
            return response
        response.success = True
        response.detail = (
            f"Confirmed physical sensor '{state.active_sensor_id}'"
        )
        response.state = self.publish_state(state)
        return response

    @staticmethod
    def _message(state) -> StateMessage:
        message = StateMessage()
        values = {
            SensorAttachmentStatus.NO_SENSOR: StateMessage.STATUS_NO_SENSOR,
            SensorAttachmentStatus.CONFIRMATION_PENDING: (
                StateMessage.STATUS_CONFIRMATION_PENDING
            ),
            SensorAttachmentStatus.ACTIVE: StateMessage.STATUS_ACTIVE,
        }
        message.status = values[state.status]
        message.active_sensor_id = state.active_sensor_id
        message.pending_sensor_id = state.pending_sensor_id
        message.attachment_revision = int(state.attachment_revision)
        return message

    def close(self):
        """Destroy sensor attachment ROS resources."""
        self.node.destroy_service(self._select_service)
        self.node.destroy_service(self._confirm_service)
        self.node.destroy_publisher(self._publisher)


__all__ = ["SensorAttachmentApi"]
