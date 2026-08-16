"""ROS owner for persistent sensor transforms and registry mutations."""

from pathlib import Path
from typing import Dict, Optional

import rclpy
from fault_detector_msgs.msg import (
    SensorAttachmentState,
    SensorDefinition as SensorDefinitionMessage,
    SensorDefinitionArray,
)
from fault_detector_msgs.srv import (
    AddSensor,
    DeleteSensor,
    UpdateSensor,
)
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from synchros2.static_transform_broadcaster import (
    StaticTransformBroadcaster,
)

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SENSOR_PARENT_FRAME,
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.shared.ros.qos_profiles import (
    APPLICATION_STATE_QOS,
    LATCHED_QOS,
)


class SensorRegistryNode(Node):
    """Persist sensor mounts and expose them through ROS and TF."""

    SENSOR_LIST_TOPIC = "fault_detector/sensors"
    ADD_SENSOR_SERVICE = "fault_detector/add_sensor"
    UPDATE_SENSOR_SERVICE = "fault_detector/update_sensor"
    DELETE_SENSOR_SERVICE = "fault_detector/delete_sensor"
    ATTACHMENT_STATE_TOPIC = (
        "fault_detector/application/sensor_attachment_state"
    )

    def __init__(
        self,
        sensor_root: Optional[Path] = None,
    ):
        """Load stored definitions and publish their static transforms."""
        super().__init__("sensor_registry")
        configured_sensor_root = ""
        if sensor_root is None:
            self.declare_parameter("sensor.root", "")
            configured_sensor_root = (
                self.get_parameter("sensor.root").value.strip()
            )
        self.repository = SensorRepository(
            sensor_root or configured_sensor_root or None,
        )
        self._definitions: Dict[str, SensorDefinition] = {}
        self._attachment_state = None
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._sensor_list_publisher = self.create_publisher(
            SensorDefinitionArray,
            self.SENSOR_LIST_TOPIC,
            LATCHED_QOS,
        )
        self._attachment_subscription = self.create_subscription(
            SensorAttachmentState,
            self.ATTACHMENT_STATE_TOPIC,
            self._receive_attachment_state,
            APPLICATION_STATE_QOS,
        )
        self._add_sensor_service = self.create_service(
            AddSensor,
            self.ADD_SENSOR_SERVICE,
            self._handle_add_sensor,
        )
        self._update_sensor_service = self.create_service(
            UpdateSensor,
            self.UPDATE_SENSOR_SERVICE,
            self._handle_update_sensor,
        )
        self._delete_sensor_service = self.create_service(
            DeleteSensor,
            self.DELETE_SENSOR_SERVICE,
            self._handle_delete_sensor,
        )
        self._load_stored_definitions()
        self._publish_sensor_list()

    def _load_stored_definitions(self) -> None:
        transforms = []
        for sensor_id in self.repository.list_sensor_ids():
            try:
                definition = self.repository.load(sensor_id)
            except Exception as exception:
                self.get_logger().error(
                    f"Failed to load sensor '{sensor_id}': {exception}"
                )
                continue
            self._definitions[sensor_id] = definition
            transforms.append(self._transform_message(definition))
        if transforms:
            self._static_broadcaster.sendTransform(transforms)

    def _receive_attachment_state(
        self,
        state: SensorAttachmentState,
    ) -> None:
        self._attachment_state = state

    def _handle_add_sensor(self, request, response):
        try:
            definition = self._definition_from_message(request.sensor)
            self.repository.create(definition)
            self._definitions[definition.sensor_id] = definition
            self._static_broadcaster.sendTransform(
                self._transform_message(definition)
            )
            self._publish_sensor_list()
        except Exception as exception:
            response.success = False
            response.message = str(exception)
            self.get_logger().error(
                f"Sensor creation failed: {exception}"
            )
            return response

        response.success = True
        response.message = (
            f"Created sensor '{definition.sensor_id}' as "
            f"{definition.probe_frame}"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_update_sensor(self, request, response):
        try:
            definition = self._definition_from_message(request.sensor)
            self._require_mutation_allowed(definition.sensor_id)
            self.repository.update(definition)
            self._definitions[definition.sensor_id] = definition
            self._static_broadcaster.sendTransform(
                self._transform_message(definition)
            )
            self._publish_sensor_list()
        except Exception as exception:
            response.success = False
            response.message = str(exception)
            self.get_logger().error(
                f"Sensor update failed: {exception}"
            )
            return response

        response.success = True
        response.message = (
            f"Updated sensor '{definition.sensor_id}' transform"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_delete_sensor(self, request, response):
        return self._delete_sensor_response(request, response)

    def _delete_sensor_response(self, request, response):
        try:
            definition = self._delete_sensor(request.sensor_id)
            self._publish_sensor_list()
        except Exception as exception:
            response.success = False
            response.message = str(exception)
            self.get_logger().error(
                f"Sensor deletion failed: {exception}"
            )
            return response

        response.success = True
        response.message = (
            f"Deleted sensor '{definition.sensor_id}'. Restart the complete "
            "system to clear its old static TF from existing listeners."
        )
        self.get_logger().info(response.message)
        return response

    def _delete_sensor(self, sensor_id: str) -> SensorDefinition:
        self._require_mutation_allowed(sensor_id)
        definition = self.repository.delete(sensor_id)
        self._definitions.pop(sensor_id, None)
        return definition

    def _require_mutation_allowed(self, sensor_id: str) -> None:
        state = self._attachment_state
        if state is None:
            raise RuntimeError(
                "Sensor attachment state is unavailable; update or delete "
                "is blocked until application state is received"
            )
        if sensor_id in {
            state.active_sensor_id.strip(),
            state.pending_sensor_id.strip(),
        }:
            raise RuntimeError(
                f"Sensor '{sensor_id}' is currently selected. Remove or "
                "select another sensor before editing or deleting it."
            )

    def _publish_sensor_list(self) -> None:
        message = SensorDefinitionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = SENSOR_PARENT_FRAME
        message.sensors = [
            self._definition_message(self._definitions[sensor_id])
            for sensor_id in sorted(self._definitions)
        ]
        self._sensor_list_publisher.publish(message)

    def _transform_message(
        self,
        definition: SensorDefinition,
    ) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = SENSOR_PARENT_FRAME
        message.child_frame_id = definition.probe_frame
        position = definition.hand_to_probe.position
        orientation = definition.hand_to_probe.orientation
        message.transform.translation.x = position.x
        message.transform.translation.y = position.y
        message.transform.translation.z = position.z
        message.transform.rotation.x = orientation.x
        message.transform.rotation.y = orientation.y
        message.transform.rotation.z = orientation.z
        message.transform.rotation.w = orientation.w
        return message

    @staticmethod
    def _definition_message(
        definition: SensorDefinition,
    ) -> SensorDefinitionMessage:
        message = SensorDefinitionMessage()
        message.sensor_id = definition.sensor_id
        message.display_name = definition.display_name
        position = definition.hand_to_probe.position
        orientation = definition.hand_to_probe.orientation
        message.hand_to_probe.position.x = position.x
        message.hand_to_probe.position.y = position.y
        message.hand_to_probe.position.z = position.z
        message.hand_to_probe.orientation.x = orientation.x
        message.hand_to_probe.orientation.y = orientation.y
        message.hand_to_probe.orientation.z = orientation.z
        message.hand_to_probe.orientation.w = orientation.w
        return message

    @staticmethod
    def _definition_from_message(
        message: SensorDefinitionMessage,
    ) -> SensorDefinition:
        definition = SensorDefinition(
            sensor_id=message.sensor_id,
            display_name=message.display_name,
            hand_to_probe=PoseData(
                position=Vector3Data(
                    x=message.hand_to_probe.position.x,
                    y=message.hand_to_probe.position.y,
                    z=message.hand_to_probe.position.z,
                ),
                orientation=QuaternionData(
                    x=message.hand_to_probe.orientation.x,
                    y=message.hand_to_probe.orientation.y,
                    z=message.hand_to_probe.orientation.z,
                    w=message.hand_to_probe.orientation.w,
                ),
            ),
        )
        definition.validate()
        return definition


def main(args=None):
    rclpy.init(args=args)
    node = SensorRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
