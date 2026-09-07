"""Qt adapter for authoritative sensor acquisition state."""

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_msgs.msg import (
    OperationalIntent,
    SensorAcquisitionState,
)

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.ui.sensor.models import (
    SensorAcquisitionView,
    SensorAcquisitionViewStatus,
)


class SensorAcquisitionClient(QObject):
    """Expose acquisition state and build manual recording intents."""

    state_changed = pyqtSignal(object)
    STATE_TOPIC = "fault_detector/application/sensor_acquisition_state"

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._state_subscription = node.create_subscription(
            SensorAcquisitionState,
            self.STATE_TOPIC,
            self._receive_state,
            LATCHED_QOS,
        )

    @staticmethod
    def make_intent(start: bool) -> OperationalIntent:
        """Build one recordable manual start or stop intent."""
        intent = OperationalIntent()
        intent.intent = (
            OperationalIntent.INTENT_START_SENSOR_RECORDING
            if start
            else OperationalIntent.INTENT_STOP_SENSOR_RECORDING
        )
        return intent

    def destroy(self) -> None:
        self.node.destroy_subscription(self._state_subscription)

    def _receive_state(self, message) -> None:
        values = {
            SensorAcquisitionState.STATE_IDLE: (
                SensorAcquisitionViewStatus.IDLE
            ),
            SensorAcquisitionState.STATE_STARTING: (
                SensorAcquisitionViewStatus.STARTING
            ),
            SensorAcquisitionState.STATE_RECORDING: (
                SensorAcquisitionViewStatus.RECORDING
            ),
            SensorAcquisitionState.STATE_STOPPING: (
                SensorAcquisitionViewStatus.STOPPING
            ),
            SensorAcquisitionState.STATE_FAILED: (
                SensorAcquisitionViewStatus.FAILED
            ),
        }
        self.state_changed.emit(SensorAcquisitionView(
            status=values.get(
                int(message.state),
                SensorAcquisitionViewStatus.UNAVAILABLE,
            ),
            sensor_id=message.sensor_id.strip(),
            detail=message.detail.strip(),
        ))

__all__ = ["SensorAcquisitionClient"]
