"""ROS client for the physical sensor definition registry."""

from array import array
from concurrent.futures import Future

from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from fault_detector_msgs.msg import SensorChannel, SensorDefinitionArray
from fault_detector_msgs.srv import (
    AddSensor,
    DeleteSensor,
    UpdateSensor,
    UploadSensorMesh,
)

from fault_detector_spot.shared.geometry.models import (
    QuaternionData,
)
from fault_detector_spot.inspection.model.sensor_models import (
    quaternion_from_rpy_degrees,
    rpy_degrees_from_quaternion,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_MESH_CHUNK_BYTES,
    validate_collision_mesh,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.shared.ros.sensor_geometry_adapter import (
    head_geometry_from_messages,
    head_geometry_to_messages,
)
from fault_detector_spot.ui.sensor.models import (
    SensorChannelView,
    SensorDefinitionView,
)


class SensorRegistryClient(QObject):
    """Expose sensor definition registry transport as Qt signals."""

    definitions_changed = pyqtSignal(object)
    creation_finished = pyqtSignal(bool, str)
    update_finished = pyqtSignal(bool, str)
    deletion_finished = pyqtSignal(str, bool, str)
    upload_progress = pyqtSignal(int, int)

    SENSOR_TOPIC = "fault_detector/sensors"
    ADD_SENSOR_SERVICE = "fault_detector/add_sensor"
    UPDATE_SENSOR_SERVICE = "fault_detector/update_sensor"
    DELETE_SENSOR_SERVICE = "fault_detector/delete_sensor"
    UPLOAD_MESH_SERVICE = "fault_detector/upload_sensor_mesh"

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
        self._upload_mesh_client = node.create_client(
            UploadSensorMesh,
            self.UPLOAD_MESH_SERVICE,
        )

    def create_sensor(
        self,
        sensor_id: str,
        display_name: str,
        translation_m,
        rotation_degrees,
        channels=(),
        head_geometry=None,
        collision_mesh=b"",
        orientation=None,
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
            channels,
            head_geometry,
            collision_mesh,
            orientation,
        )

    def update_sensor(
        self,
        sensor_id: str,
        display_name: str,
        translation_m,
        rotation_degrees,
        channels=(),
        head_geometry=None,
        collision_mesh=b"",
        orientation=None,
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
            channels,
            head_geometry,
            collision_mesh,
            orientation,
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
        channels,
        head_geometry,
        collision_mesh,
        orientation,
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
        quaternion = (
            QuaternionData(*orientation)
            if orientation is not None
            else quaternion_from_rpy_degrees(*rotation)
        )
        quaternion.validate()
        if collision_mesh:
            if head_geometry is None:
                raise ValueError("Collision mesh requires geometry metadata")
            validate_collision_mesh(
                collision_mesh,
                head_geometry.collision_sha256,
                head_geometry.collision_byte_count,
            )
            if not self._upload_mesh_client.service_is_ready():
                result_signal.emit(False, "Collision mesh upload service is unavailable")
                return None

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
        request.sensor.channels = [
            self._channel_message(channel) for channel in channels
        ]
        request.sensor.head_geometry = head_geometry_to_messages(head_geometry)

        try:
            future = (
                self._upload_and_save(client, request, head_geometry, collision_mesh)
                if collision_mesh
                else client.call_async(request)
            )
        except Exception as exception:
            result_signal.emit(False, str(exception))
            return None
        future.add_done_callback(
            lambda result: self._handle_registry_result(
                result,
                result_signal,
            )
        )
        return future

    def _upload_and_save(self, client, request, geometry, mesh):
        """Upload bounded chunks, then submit one combined definition update."""
        completion = Future()
        total = len(mesh)
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(30_000)

        def timed_out():
            if not completion.done():
                completion.set_exception(TimeoutError(
                    "Timed out waiting for the sensor registry during mesh save"
                ))

        def cleanup(_future):
            timer.stop()
            timer.deleteLater()

        timer.timeout.connect(timed_out)
        completion.add_done_callback(cleanup)

        def finish(future):
            if completion.done():
                return
            try:
                completion.set_result(future.result())
            except Exception as exception:
                completion.set_exception(exception)

        def upload(offset):
            chunk = UploadSensorMesh.Request()
            chunk.sha256 = geometry.collision_sha256
            chunk.byte_count = total
            chunk.offset = offset
            chunk.data = array("B", mesh[offset:offset + MAX_MESH_CHUNK_BYTES])
            try:
                timer.start()
                future = self._upload_mesh_client.call_async(chunk)
                future.add_done_callback(lambda result: accept(result, offset + len(chunk.data)))
            except Exception as exception:
                completion.set_exception(exception)

        def accept(future, next_offset):
            if completion.done():
                return
            try:
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError(
                        response.message if response is not None
                        else "Collision mesh upload returned no response"
                    )
                if response.complete:
                    self.upload_progress.emit(total, total)
                    timer.start()
                    client.call_async(request).add_done_callback(finish)
                elif next_offset < total:
                    self.upload_progress.emit(next_offset, total)
                    upload(next_offset)
                else:
                    raise RuntimeError("Collision mesh upload did not complete")
            except Exception as exception:
                completion.set_exception(exception)

        self.upload_progress.emit(0, total)
        upload(0)
        return completion

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
            head_geometry=head_geometry_from_messages(sensor.head_geometry),
            channels=tuple(
                SensorChannelView(
                    channel_id=channel.channel_id,
                    topic=channel.topic,
                    message_type=channel.message_type,
                    source_kind=(
                        getattr(channel, "source_kind", "")
                        or "ros_topic"
                    ),
                )
                for channel in sensor.channels
            ),
        )

    @staticmethod
    def _channel_message(channel):
        message = SensorChannel()
        message.channel_id = str(channel.channel_id).strip()
        message.topic = str(channel.topic).strip()
        message.message_type = str(channel.message_type).strip()
        source_kind = getattr(channel, "source_kind", "ros_topic")
        message.source_kind = str(
            getattr(source_kind, "value", source_kind)
        ).strip()
        return message

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
        self.node.destroy_client(self._upload_mesh_client)


__all__ = [
    "SensorRegistryClient",
]
