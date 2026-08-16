"""ROS client for authoritative physical sensor attachment state."""

from PyQt5.QtCore import QObject, pyqtSignal
from fault_detector_msgs.msg import (
    SensorAttachmentState as SensorAttachmentStateMessage,
)
from fault_detector_msgs.srv import (
    ConfirmSensorAttachment,
    SelectSensorAttachment,
)

from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.ui.sensor.models import (
    SensorAttachmentView,
    SensorAttachmentViewStatus,
)


class SensorAttachmentClient(QObject):
    """Expose physical sensor attachment transport as Qt signals."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)

    STATE_TOPIC = "fault_detector/application/sensor_attachment_state"
    SELECT_SERVICE = "fault_detector/application/select_sensor_attachment"
    CONFIRM_SERVICE = "fault_detector/application/confirm_sensor_attachment"

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._state_subscription = node.create_subscription(
            SensorAttachmentStateMessage,
            self.STATE_TOPIC,
            self._receive_state,
            APPLICATION_STATE_QOS,
        )
        self._select_client = node.create_client(
            SelectSensorAttachment,
            self.SELECT_SERVICE,
        )
        self._confirm_client = node.create_client(
            ConfirmSensorAttachment,
            self.CONFIRM_SERVICE,
        )

    def select(self, sensor_id: str):
        """Select a physical sensor or clear selection with an empty ID."""
        if not self._select_client.service_is_ready():
            self.request_rejected.emit(
                "Sensor attachment service is unavailable"
            )
            return None
        request = SelectSensorAttachment.Request()
        request.sensor_id = sensor_id.strip()
        future = self._select_client.call_async(request)
        future.add_done_callback(self._handle_service_result)
        return future

    def confirm(self, sensor_id: str, attachment_revision: int):
        """Confirm the exact pending physical sensor attachment."""
        if not self._confirm_client.service_is_ready():
            self.request_rejected.emit(
                "Sensor confirmation service is unavailable"
            )
            return None
        request = ConfirmSensorAttachment.Request()
        request.sensor_id = sensor_id.strip()
        request.attachment_revision = int(attachment_revision)
        future = self._confirm_client.call_async(request)
        future.add_done_callback(self._handle_service_result)
        return future

    def _receive_state(self, message):
        self.state_changed.emit(self._state_view(message))

    def _handle_service_result(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if response is None:
            self.request_rejected.emit(
                "Sensor attachment service returned no response"
            )
            return
        if not response.success:
            self.request_rejected.emit(response.detail)
            self.state_changed.emit(self._state_view(response.state))
            return
        self.state_changed.emit(self._state_view(response.state))

    @staticmethod
    def _state_view(message) -> SensorAttachmentView:
        values = {
            SensorAttachmentStateMessage.STATUS_NO_SENSOR: (
                SensorAttachmentViewStatus.NONE
            ),
            SensorAttachmentStateMessage.STATUS_CONFIRMATION_PENDING: (
                SensorAttachmentViewStatus.PENDING
            ),
            SensorAttachmentStateMessage.STATUS_ACTIVE: (
                SensorAttachmentViewStatus.ACTIVE
            ),
        }
        return SensorAttachmentView(
            status=values.get(
                int(message.status),
                SensorAttachmentViewStatus.NONE,
            ),
            active_sensor_id=message.active_sensor_id.strip(),
            pending_sensor_id=message.pending_sensor_id.strip(),
            attachment_revision=int(message.attachment_revision),
        )

    def destroy(self):
        """Destroy attachment subscriptions and service clients."""
        self.node.destroy_subscription(self._state_subscription)
        self.node.destroy_client(self._select_client)
        self.node.destroy_client(self._confirm_client)


__all__ = [
    "SensorAttachmentClient",
]

