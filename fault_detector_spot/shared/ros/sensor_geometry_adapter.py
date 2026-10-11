"""Convert optional sensor collision metadata at the ROS boundary."""

from fault_detector_msgs.msg import SensorHeadGeometry as GeometryMessage

from fault_detector_spot.inspection.model.sensor_geometry import SensorHeadGeometry
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


def head_geometry_to_messages(geometry):
    """Encode an optional geometry as a bounded zero-or-one ROS array."""
    if geometry is None:
        return []
    geometry.validate()
    message = GeometryMessage()
    message.collision_sha256 = geometry.collision_sha256
    message.collision_byte_count = geometry.collision_byte_count
    position = geometry.hand_to_geometry.position
    orientation = geometry.hand_to_geometry.orientation
    message.hand_to_geometry.position.x = float(position.x)
    message.hand_to_geometry.position.y = float(position.y)
    message.hand_to_geometry.position.z = float(position.z)
    message.hand_to_geometry.orientation.x = float(orientation.x)
    message.hand_to_geometry.orientation.y = float(orientation.y)
    message.hand_to_geometry.orientation.z = float(orientation.z)
    message.hand_to_geometry.orientation.w = float(orientation.w)
    message.mounting_template_id = geometry.mounting_template_id
    message.mounting_template_revision = geometry.mounting_template_revision
    message.reference_model_fingerprint = geometry.reference_model_fingerprint
    message.mounting_template_verified = geometry.mounting_template_verified
    message.clearance_m = float(geometry.clearance_m)
    message.configuration_method = geometry.configuration_method
    return [message]


def head_geometry_from_messages(messages):
    """Decode geometry metadata without resolving backend asset paths."""
    if not messages:
        return None
    if len(messages) != 1:
        raise ValueError("A sensor definition supports one collision geometry")
    message = messages[0]
    position = message.hand_to_geometry.position
    orientation = message.hand_to_geometry.orientation
    geometry = SensorHeadGeometry(
        collision_sha256=message.collision_sha256,
        collision_byte_count=message.collision_byte_count,
        hand_to_geometry=PoseData(
            Vector3Data(position.x, position.y, position.z),
            QuaternionData(orientation.x, orientation.y, orientation.z, orientation.w),
        ),
        mounting_template_id=message.mounting_template_id,
        mounting_template_revision=message.mounting_template_revision,
        reference_model_fingerprint=message.reference_model_fingerprint,
        mounting_template_verified=message.mounting_template_verified,
        clearance_m=message.clearance_m,
        configuration_method=message.configuration_method,
    )
    geometry.validate()
    return geometry
