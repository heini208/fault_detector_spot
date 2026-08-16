"""ROS client for the physical sensor definition registry."""

from PyQt5.QtCore import QObject, pyqtSignal
from fault_detector_msgs.msg import SensorDefinitionArray
from fault_detector_msgs.srv import (
    AddSensor,
    DeleteSensor,
    UpdateSensor,
)

from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.inspection.model.sensor_models import (
    quaternion_from_rpy_degrees,
    rpy_degrees_from_quaternion,
    sensor_probe_frame,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.ui.sensor.models import SensorDefinitionView


class SensorRegistryClient(QObject):
    """Expose sensor definition registry transport as Qt signals."""

    definitions_changed = pyqtSignal(object)
    creation_finished = pyqtSignal(bool, str)
    update_finished = pyqtSignal(bool, str)
    deletion_finished = pyqtSignal(str, bool, str)

    SENSOR_TOPIC = "fault_detector/sensors"
    ADD_SENSOR_SERVICE = "fault_detector/add_sensor"
    UPDATE_SENSOR_SERVICE = "fault_detector/update_sensor"
    DELETE_SENSOR_SERVICE = "fault_detector/delete_sensor"

    def __init__(self, node):
        super().__init__()
        self.node = node
        self._definitions_subscription = node.create_subscription(
            SensorDefinitionArray,
            self.SENSOR_TOPIC,
            self._receive_definitions,
            LATCHED_QOS,
        )
        self._add_sensor_client = node.create_client(
            AddSensor,
            self.ADD_SENSOR_SERVICE,
        )
        self._update_sensor_client = node.create_client(
            UpdateSensor,
            self.UPDATE_SENSOR_SERVICE,
        )
        self._delete_sensor_client = node.create_client(
            DeleteSensor,
            self.DELETE_SENSOR_SERVICE,
        )

    def create_sensor(
        self,
        sensor_id: str,
        display_name: str,
        translation_m,
        rotation_degrees,
    ):
        """Create one manually configured sensor definition."""
        return self._save_sensor(
            self._add_sensor_client,
            AddSensor.Request,
            self.creation_finished,
            sensor_id,
            display_name,
            translation_m,
            rotation_degrees,
        )

    def update_sensor(
        self,
        sensor_id: str,
        display_name: str,
        translation_m,
        rotation_degrees,
    ):
        """Replace one existing sensor definition."""
        return self._save_sensor(
            self._update_sensor_client,
            UpdateSensor.Request,
            self.update_finished,
            sensor_id,
            display_name,
            translation_m,
            rotation_degrees,
        )

    def delete_sensor(self, sensor_id: str):
        """Delete one registered sensor definition."""
        if not self._delete_sensor_client.service_is_ready():
            self.deletion_finished.emit(
                sensor_id.strip(),
                False,
                "Sensor deletion service is unavailable",
            )
            return None
        request = DeleteSensor.Request()
        request.sensor_id = sensor_id.strip()
        future = self._delete_sensor_client.call_async(request)
        future.add_done_callback(
            lambda result: self._handle_deletion_result(
                result,
                request.sensor_id,
            )
        )
        return future

    def _save_sensor(
        self,
        client,
        request_type,
        result_signal,
        sensor_id,
        display_name,
        translation_m,
        rotation_degrees,
    ):
        if not client.service_is_ready():
            result_signal.emit(
                False,
                "Sensor registry service is unavailable",
            )
            return None

        translation = self._three_values(
            translation_m,
            "translation",
        )
        rotation = self._three_values(
            rotation_degrees,
            "rotation",
        )
        quaternion = quaternion_from_rpy_degrees(*rotation)

        request = request_type()
        request.sensor.sensor_id = sensor_id.strip()
        request.sensor.display_name = display_name.strip()
        request.sensor.hand_to_probe.position.x = translation[0]
        request.sensor.hand_to_probe.position.y = translation[1]
        request.sensor.hand_to_probe.position.z = translation[2]
        request.sensor.hand_to_probe.orientation.x = quaternion.x
        request.sensor.hand_to_probe.orientation.y = quaternion.y
        request.sensor.hand_to_probe.orientation.z = quaternion.z
        request.sensor.hand_to_probe.orientation.w = quaternion.w

        future = client.call_async(request)
        future.add_done_callback(
            lambda result: self._handle_registry_result(
                result,
                result_signal,
            )
        )
        return future

    def _receive_definitions(self, message):
        definitions = tuple(
            self._definition_view(sensor)
            for sensor in message.sensors
        )
        self.definitions_changed.emit(definitions)

    def _definition_view(self, sensor):
        orientation = QuaternionData(
            x=float(sensor.hand_to_probe.orientation.x),
            y=float(sensor.hand_to_probe.orientation.y),
            z=float(sensor.hand_to_probe.orientation.z),
            w=float(sensor.hand_to_probe.orientation.w),
        )
        return SensorDefinitionView(
            sensor_id=sensor.sensor_id,
            display_name=sensor.display_name,
            probe_frame=sensor_probe_frame(sensor.sensor_id),
            position=(
                float(sensor.hand_to_probe.position.x),
                float(sensor.hand_to_probe.position.y),
                float(sensor.hand_to_probe.position.z),
            ),
            orientation=(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
            rotation_degrees=rpy_degrees_from_quaternion(orientation),
        )

    def _handle_deletion_result(self, future, sensor_id):
        try:
            response = future.result()
        except Exception as exception:
            self.deletion_finished.emit(
                sensor_id,
                False,
                str(exception),
            )
            return
        if response is None:
            self.deletion_finished.emit(
                sensor_id,
                False,
                "Sensor registry service returned no response",
            )
            return
        self.deletion_finished.emit(
            sensor_id,
            bool(response.success),
            response.message,
        )

    @staticmethod
    def _handle_registry_result(future, result_signal):
        try:
            response = future.result()
        except Exception as exception:
            result_signal.emit(False, str(exception))
            return
        if response is None:
            result_signal.emit(
                False,
                "Sensor registry service returned no response",
            )
            return
        result_signal.emit(
            bool(response.success),
            response.message,
        )

    @staticmethod
    def _three_values(values, label):
        try:
            normalized = tuple(float(value) for value in values)
        except (TypeError, ValueError) as exception:
            raise ValueError(
                f"Sensor {label} must contain three numeric values"
            ) from exception
        if len(normalized) != 3:
            raise ValueError(
                f"Sensor {label} must contain exactly three values"
            )
        return normalized

    def destroy(self):
        """Destroy registry subscriptions and service clients."""
        self.node.destroy_subscription(self._definitions_subscription)
        self.node.destroy_client(self._add_sensor_client)
        self.node.destroy_client(self._update_sensor_client)
        self.node.destroy_client(self._delete_sensor_client)


__all__ = [
    "SensorRegistryClient",
]
