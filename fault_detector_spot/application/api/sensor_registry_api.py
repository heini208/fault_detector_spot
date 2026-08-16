"""Expose application-owned sensor definition registry through ROS."""

from threading import RLock

from fault_detector_msgs.msg import (
    SensorDefinition as SensorDefinitionMessage,
    SensorDefinitionArray,
)
from fault_detector_msgs.srv import AddSensor, DeleteSensor, UpdateSensor
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
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
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


class SensorRegistryApi:
    """Own registry services, definition publication, and static TF."""

    SENSOR_LIST_TOPIC = "fault_detector/sensors"
    ADD_SENSOR_SERVICE = "fault_detector/add_sensor"
    UPDATE_SENSOR_SERVICE = "fault_detector/update_sensor"
    DELETE_SENSOR_SERVICE = "fault_detector/delete_sensor"

    def __init__(self, node, controller):
        self.node = node
        self.controller = controller
        self._lock = RLock()
        self._callback_group = ReentrantCallbackGroup()
        self._static_broadcaster = StaticTransformBroadcaster(node)
        self._publisher = node.create_publisher(
            SensorDefinitionArray,
            self.SENSOR_LIST_TOPIC,
            LATCHED_QOS,
        )
        self._add_service = node.create_service(
            AddSensor,
            self.ADD_SENSOR_SERVICE,
            self._handle_add_sensor,
            callback_group=self._callback_group,
        )
        self._update_service = node.create_service(
            UpdateSensor,
            self.UPDATE_SENSOR_SERVICE,
            self._handle_update_sensor,
            callback_group=self._callback_group,
        )
        self._delete_service = node.create_service(
            DeleteSensor,
            self.DELETE_SENSOR_SERVICE,
            self._handle_delete_sensor,
            callback_group=self._callback_group,
        )
        definitions = controller.definitions()
        self._broadcast_definitions(definitions)
        self._publish_definitions(definitions)

    def _handle_add_sensor(self, request, response):
        return self._handle_definition_mutation(
            response,
            lambda: self.controller.create(
                self._definition_from_message(request.sensor)
            ),
            "Created",
        )

    def _handle_update_sensor(self, request, response):
        return self._handle_definition_mutation(
            response,
            lambda: self.controller.update(
                self._definition_from_message(request.sensor)
            ),
            "Updated",
        )

    def _handle_definition_mutation(
        self,
        response,
        mutation,
        operation,
    ):
        with self._lock:
            try:
                definition = mutation()
                self._static_broadcaster.sendTransform(
                    self._transform_message(definition)
                )
                self._publish_definitions(self.controller.definitions())
            except Exception as exception:
                response.success = False
                response.message = str(exception)
                self.node.get_logger().error(
                    f"Sensor {operation.lower()} failed: {exception}"
                )
                return response

        response.success = True
        response.message = (
            f"{operation} sensor '{definition.sensor_id}' as "
            f"{definition.probe_frame}"
        )
        self.node.get_logger().info(response.message)
        return response

    def _handle_delete_sensor(self, request, response):
        with self._lock:
            try:
                definition = self.controller.delete(request.sensor_id)
                self._publish_definitions(self.controller.definitions())
            except Exception as exception:
                response.success = False
                response.message = str(exception)
                self.node.get_logger().error(
                    f"Sensor deletion failed: {exception}"
                )
                return response

        response.success = True
        response.message = (
            f"Deleted sensor '{definition.sensor_id}'. Restart the complete "
            "system to clear its old static TF from existing listeners."
        )
        self.node.get_logger().info(response.message)
        return response

    def _publish_definitions(self, definitions) -> SensorDefinitionArray:
        message = SensorDefinitionArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = SENSOR_PARENT_FRAME
        message.sensors = [
            self._definition_message(definition)
            for definition in definitions
        ]
        self._publisher.publish(message)
        return message

    def _broadcast_definitions(self, definitions) -> None:
        transforms = [
            self._transform_message(definition)
            for definition in definitions
        ]
        if transforms:
            self._static_broadcaster.sendTransform(transforms)

    def _transform_message(
        self,
        definition: SensorDefinition,
    ) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
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

    def close(self):
        """Destroy sensor registry ROS resources."""
        self.node.destroy_service(self._add_service)
        self.node.destroy_service(self._update_service)
        self.node.destroy_service(self._delete_service)
        self.node.destroy_publisher(self._publisher)


__all__ = ["SensorRegistryApi"]
