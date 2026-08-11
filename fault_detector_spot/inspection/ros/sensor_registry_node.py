"""ROS owner for persistent sensor calibrations and static transforms."""

from pathlib import Path
from typing import Dict, Optional

import rclpy
from fault_detector_msgs.msg import (
    SensorDefinition as SensorDefinitionMessage,
    SensorDefinitionArray,
)
from fault_detector_msgs.srv import AddSensor, RetireSensor
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from synchros2.static_transform_broadcaster import (
    StaticTransformBroadcaster,
)

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.repository.object_repository import ObjectRepository
from fault_detector_spot.inspection.model.sensor_models import (
    SENSOR_PARENT_FRAME,
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_repository import SensorRepository


class SensorRegistryNode(Node):
    """Persist sensor mounts and expose them through ROS and TF."""

    SENSOR_LIST_TOPIC = "fault_detector/sensors"
    ADD_SENSOR_SERVICE = "fault_detector/add_sensor"
    RETIRE_SENSOR_SERVICE = "fault_detector/retire_sensor"

    def __init__(
        self,
        sensor_root: Optional[Path] = None,
        object_root: Optional[Path] = None,
        retired_sensor_root: Optional[Path] = None,
    ):
        """Load stored definitions and publish their static transforms."""
        super().__init__("sensor_registry")
        configured_sensor_root = ""
        if sensor_root is None:
            self.declare_parameter("sensor.root", "")
            configured_sensor_root = (
                self.get_parameter("sensor.root").value.strip()
            )
        configured_retired_root = ""
        if retired_sensor_root is None:
            self.declare_parameter("sensor.retired_root", "")
            configured_retired_root = (
                self.get_parameter("sensor.retired_root").value.strip()
            )
        configured_object_root = ""
        if object_root is None:
            self.declare_parameter("inspection.object_root", "")
            configured_object_root = (
                self.get_parameter("inspection.object_root").value.strip()
            )
        self.repository = SensorRepository(
            sensor_root or configured_sensor_root or None,
            retired_sensor_root or configured_retired_root or None,
        )
        self.object_repository = ObjectRepository(
            object_root or configured_object_root or None
        )
        self._definitions: Dict[str, SensorDefinition] = {}
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._sensor_list_publisher = self.create_publisher(
            SensorDefinitionArray,
            self.SENSOR_LIST_TOPIC,
            LATCHED_QOS,
        )
        self._add_sensor_service = self.create_service(
            AddSensor,
            self.ADD_SENSOR_SERVICE,
            self._handle_add_sensor,
        )
        self._retire_sensor_service = self.create_service(
            RetireSensor,
            self.RETIRE_SENSOR_SERVICE,
            self._handle_retire_sensor,
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

    def _handle_retire_sensor(self, request, response):
        try:
            definition = self._retire_sensor(request.sensor_id)
            self._publish_sensor_list()
        except Exception as exception:
            response.success = False
            response.message = str(exception)
            self.get_logger().error(
                f"Sensor retirement failed: {exception}"
            )
            return response

        response.success = True
        response.message = (
            f"Retired sensor '{definition.sensor_id}'. Restart the complete "
            "system to clear its static TF. The sensor ID remains reserved."
        )
        self.get_logger().info(response.message)
        return response

    def _retire_sensor(self, sensor_id: str) -> SensorDefinition:
        references = self.object_repository.find_sensor_references(sensor_id)
        if references:
            raise ValueError(
                f"Sensor '{sensor_id}' is referenced by saved routines: "
                + ", ".join(references)
            )
        definition = self.repository.retire(sensor_id)
        self._definitions.pop(sensor_id, None)
        return definition

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
