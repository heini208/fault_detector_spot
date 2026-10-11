"""Resolve a confirmed sensor head through the application ROS boundary."""

from copy import deepcopy

from fault_detector_msgs.msg import SensorAttachmentState
from fault_detector_msgs.srv import GetSensorAttachment, GetSensorMesh
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from fault_detector_spot.inspection.model.sensor_geometry import MAX_MESH_CHUNK_BYTES
from fault_detector_spot.manipulation.sensor_head_geometry import (
    sensor_head_attachment,
    stock_head_attachment,
    validate_sensor_head_model,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS
from fault_detector_spot.shared.ros.sensor_geometry_adapter import head_geometry_from_messages


class MoveItSensorHead:
    """Prepare read requests; the existing planner sequences every scene write."""

    def __init__(self, node):
        self.node = node
        self.attachment_client = node.create_client(
            GetSensorAttachment, "fault_detector/application/get_sensor_attachment",
        )
        self.mesh_client = node.create_client(GetSensorMesh, "fault_detector/get_sensor_mesh")
        self.model_client = node.create_client(GetParameters, "/move_group/get_parameters")
        self._subscription = node.create_subscription(
            SensorAttachmentState, "fault_detector/application/sensor_attachment_state",
            self._observe_state, APPLICATION_STATE_QOS,
        )
        self._observed_state = None
        self._synchronized_state = None
        self._state = None
        self._geometry = None
        self._mesh_bytes = bytearray()
        self._cached_mesh = None
        self._cached_attachment = None
        self.expected = None

    @staticmethod
    def _state_key(state):
        return (state.status, state.active_sensor_id, state.pending_sensor_id,
                state.attachment_revision)

    def _observe_state(self, message):
        state = self._state_key(message)
        if state != self._observed_state:
            self._synchronized_state = None
        self._observed_state = state

    @property
    def needs_scene_sync(self):
        """Refresh only a confirmed attachment not yet verified in the scene."""
        state = self._observed_state
        return bool(
            state is not None and state[0] == SensorAttachmentState.STATUS_ACTIVE
            and not state[2] and state != self._synchronized_state
        )

    def prepare(self):
        self._state = None
        self.expected = None
        self._mesh_bytes.clear()
        return "head_attachment", self.attachment_client, GetSensorAttachment.Request()

    def verification_request(self):
        return "head_verify", self.attachment_client, GetSensorAttachment.Request()

    def advance(self, phase, response):
        """Return the next read request, or None when geometry is ready."""
        if phase in ("head_attachment", "head_verify"):
            if not response.success:
                raise RuntimeError(response.detail)
            state = self._state_key(response.state)
            if (state[0] != SensorAttachmentState.STATUS_ACTIVE or state[2]):
                raise RuntimeError("Sensor attachment confirmation is pending")
            geometry = head_geometry_from_messages(response.head_geometry)
            if bool(state[1]) != (geometry is not None):
                raise RuntimeError("Confirmed sensor mount has no collision geometry")
            if phase == "head_verify":
                if state != self._state or geometry != self._geometry:
                    raise RuntimeError("Sensor attachment changed during planning; retry the movement")
                self.validate()
                self._synchronized_state = state
                return None
            self._state, self._geometry = state, geometry
            self.validate()
            return ("head_model", self.model_client,
                    GetParameters.Request(names=["robot_description"]))

        self.validate()
        if phase == "head_model":
            if (len(response.values) != 1
                    or response.values[0].type != ParameterType.PARAMETER_STRING):
                raise RuntimeError("MoveIt robot description is unavailable")
            description = response.values[0].string_value
            validate_sensor_head_model(description)
            if self._geometry is None:
                self.expected = stock_head_attachment(description)
                return None
            if (self._cached_attachment is not None
                    and self._cached_attachment[0] == self._geometry):
                self.expected = deepcopy(self._cached_attachment[1])
                return None
            if self._cached_mesh is not None:
                digest, mesh_bytes = self._cached_mesh
                if digest == self._geometry.collision_sha256:
                    self._set_custom_attachment(mesh_bytes)
                    return None
            return self._mesh_request()

        if phase == "head_mesh":
            if not response.success:
                raise RuntimeError(response.message)
            data = bytes(response.data)
            size = len(self._mesh_bytes) + len(data)
            total = self._geometry.collision_byte_count
            if (not data or len(data) > MAX_MESH_CHUNK_BYTES or size > total
                    or response.complete != (size == total)):
                raise RuntimeError("Sensor collision mesh download has invalid chunk bounds")
            self._mesh_bytes.extend(data)
            if size < total:
                return self._mesh_request()
            mesh_bytes = bytes(self._mesh_bytes)
            self._set_custom_attachment(mesh_bytes)
            self._cached_mesh = (self._geometry.collision_sha256, mesh_bytes)
            self._mesh_bytes.clear()
            return None
        raise ValueError(f"Unknown sensor head read phase: {phase}")

    def _set_custom_attachment(self, mesh_bytes):
        self.expected = sensor_head_attachment(self._geometry, mesh_bytes)
        self._cached_attachment = (self._geometry, deepcopy(self.expected))

    def _mesh_request(self):
        return "head_mesh", self.mesh_client, GetSensorMesh.Request(
            sha256=self._geometry.collision_sha256,
            byte_count=self._geometry.collision_byte_count,
            offset=len(self._mesh_bytes),
        )

    def validate(self):
        if (self._state is not None and self._observed_state is not None
                and self._observed_state != self._state):
            raise RuntimeError("Sensor attachment changed; synchronize the head before planning")

    def destroy(self):
        self.node.destroy_subscription(self._subscription)
        for client in (self.attachment_client, self.mesh_client, self.model_client):
            self.node.destroy_client(client)
