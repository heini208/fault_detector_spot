"""Publish the authoritative sensor acquisition state for all clients."""

from fault_detector_msgs.msg import SensorAcquisitionState as StateMessage

from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionStatus,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


class SensorAcquisitionApi:
    """Expose coordinator state without introducing a second state owner."""

    STATE_TOPIC = "fault_detector/application/sensor_acquisition_state"

    def __init__(self, node, coordinator):
        self.node = node
        self.coordinator = coordinator
        self._publisher = node.create_publisher(
            StateMessage,
            self.STATE_TOPIC,
            LATCHED_QOS,
        )
        coordinator.add_state_listener(self.publish_state)
        self.publish_state(coordinator.snapshot())

    def publish_state(self, state) -> StateMessage:
        """Publish and return one typed authoritative state snapshot."""
        message = StateMessage()
        message.stamp = self.node.get_clock().now().to_msg()
        values = {
            SensorAcquisitionStatus.IDLE: StateMessage.STATE_IDLE,
            SensorAcquisitionStatus.STARTING: StateMessage.STATE_STARTING,
            SensorAcquisitionStatus.RECORDING: StateMessage.STATE_RECORDING,
            SensorAcquisitionStatus.STOPPING: StateMessage.STATE_STOPPING,
            SensorAcquisitionStatus.FAILED: StateMessage.STATE_FAILED,
        }
        message.state = values[state.status]
        message.sensor_id = state.sensor_id
        message.detail = state.detail
        self._publisher.publish(message)
        return message

    def close(self) -> None:
        """Release the state listener and ROS publisher."""
        self.coordinator.remove_state_listener(self.publish_state)
        self.node.destroy_publisher(self._publisher)


__all__ = ["SensorAcquisitionApi"]
