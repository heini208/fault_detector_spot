"""Own authoritative physical inspection sensor attachment state."""

from dataclasses import dataclass
from enum import Enum
from threading import RLock
from typing import Tuple

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    PersistedSensorAttachmentSelection,
    SensorAttachmentStateStore,
)


class SensorAttachmentStatus(str, Enum):
    """Describe whether a physical sensor attachment is trusted."""

    NO_SENSOR = "no_sensor"
    CONFIRMATION_PENDING = "confirmation_pending"
    ACTIVE = "active"


@dataclass(frozen=True)
class SensorAttachmentState:
    """Expose authoritative attachment state without mutable geometry."""

    active_sensor_id: str
    pending_sensor_id: str
    status: SensorAttachmentStatus
    attachment_revision: int

    @property
    def selected_sensor_id(self) -> str:
        """Return the pending selection or active sensor identity."""
        return self.pending_sensor_id or self.active_sensor_id

    @property
    def sensor_dependent_motion_allowed(self) -> bool:
        """Return whether calibrated sensor motion may be admitted."""
        return (
            self.status is SensorAttachmentStatus.ACTIVE
            and bool(self.active_sensor_id)
            and not self.pending_sensor_id
        )


@dataclass(frozen=True)
class ConfirmedSensorAttachment:
    """Freeze calibrated geometry for one confirmed attachment."""

    sensor_id: str
    display_name: str
    probe_frame: str
    attachment_revision: int
    hand_to_probe_position: Tuple[float, float, float]
    hand_to_probe_orientation: Tuple[float, float, float, float]

    @classmethod
    def from_definition(
        cls,
        definition: SensorDefinition,
        attachment_revision: int,
    ) -> "ConfirmedSensorAttachment":
        """Freeze one validated sensor definition."""
        definition.validate()
        position = definition.hand_to_probe.position
        orientation = definition.hand_to_probe.orientation
        return cls(
            sensor_id=definition.sensor_id,
            display_name=definition.display_name,
            probe_frame=definition.probe_frame,
            attachment_revision=attachment_revision,
            hand_to_probe_position=(
                float(position.x),
                float(position.y),
                float(position.z),
            ),
            hand_to_probe_orientation=(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )

    def hand_to_probe(self) -> PoseData:
        """Return an independent pose for motion calculations."""
        return PoseData(
            position=Vector3Data(*self.hand_to_probe_position),
            orientation=QuaternionData(*self.hand_to_probe_orientation),
        )


class SensorAttachmentController:
    """Own sensor selection, confirmation, and motion admission."""

    def __init__(
        self,
        sensor_repository,
        state_store: SensorAttachmentStateStore,
        command_controller,
    ):
        """Restore selection while invalidating prior confirmation."""
        self.sensor_repository = sensor_repository
        self.state_store = state_store
        self.command_controller = command_controller
        self._lock = RLock()
        self._state = self._restore_state()

    def snapshot(self) -> SensorAttachmentState:
        """Return the current immutable attachment state."""
        with self._lock:
            return self._state

    def select_sensor(self, sensor_id: str) -> SensorAttachmentState:
        """Select one registered sensor and require confirmation."""
        with self._lock:
            self._require_command_lane_idle()
            definition = self.sensor_repository.load(sensor_id)
            definition.validate()

            revision = self._state.attachment_revision + 1
            self.state_store.save(
                PersistedSensorAttachmentSelection(
                    sensor_id=definition.sensor_id,
                    attachment_revision=revision,
                )
            )
            self._state = SensorAttachmentState(
                active_sensor_id=self._state.active_sensor_id,
                pending_sensor_id=definition.sensor_id,
                status=SensorAttachmentStatus.CONFIRMATION_PENDING,
                attachment_revision=revision,
            )
            return self._state

    def clear_sensor(self) -> SensorAttachmentState:
        """Select no inspection sensor and disable sensor-dependent motion."""
        with self._lock:
            self._require_command_lane_idle()
            revision = self._state.attachment_revision + 1
            self.state_store.save(
                PersistedSensorAttachmentSelection(
                    sensor_id="",
                    attachment_revision=revision,
                )
            )
            self._state = SensorAttachmentState(
                active_sensor_id="",
                pending_sensor_id="",
                status=SensorAttachmentStatus.NO_SENSOR,
                attachment_revision=revision,
            )
            return self._state

    def confirm_sensor(
        self,
        sensor_id: str,
        attachment_revision: int,
    ) -> SensorAttachmentState:
        """Confirm the exact pending physical sensor attachment."""
        with self._lock:
            self._require_command_lane_idle()
            if (
                self._state.status
                is not SensorAttachmentStatus.CONFIRMATION_PENDING
            ):
                raise RuntimeError(
                    "No sensor attachment confirmation is pending"
                )
            if attachment_revision != self._state.attachment_revision:
                raise RuntimeError(
                    "Sensor attachment confirmation uses a stale revision"
                )
            if sensor_id != self._state.pending_sensor_id:
                raise RuntimeError(
                    "Sensor attachment confirmation does not match "
                    "the pending sensor"
                )

            definition = self.sensor_repository.load(sensor_id)
            definition.validate()
            self._state = SensorAttachmentState(
                active_sensor_id=definition.sensor_id,
                pending_sensor_id="",
                status=SensorAttachmentStatus.ACTIVE,
                attachment_revision=self._state.attachment_revision,
            )
            return self._state

    def require_confirmed_sensor(
        self,
        expected_sensor_id: str,
    ) -> ConfirmedSensorAttachment:
        """Return frozen geometry for the expected confirmed sensor."""
        with self._lock:
            if not self._state.sensor_dependent_motion_allowed:
                raise RuntimeError(
                    "A confirmed physical sensor attachment is required"
                )
            if expected_sensor_id != self._state.active_sensor_id:
                raise RuntimeError(
                    "Inspection routine requires sensor "
                    f"'{expected_sensor_id}' but the confirmed attachment is "
                    f"'{self._state.active_sensor_id}'"
                )

            definition = self.sensor_repository.load(
                self._state.active_sensor_id
            )
            definition.validate()
            return ConfirmedSensorAttachment.from_definition(
                definition,
                self._state.attachment_revision,
            )

    def _restore_state(self) -> SensorAttachmentState:
        persisted = self.state_store.load()
        if persisted is None:
            return SensorAttachmentState(
                active_sensor_id="",
                pending_sensor_id="",
                status=SensorAttachmentStatus.NO_SENSOR,
                attachment_revision=0,
            )
        if not persisted.sensor_id:
            return SensorAttachmentState(
                active_sensor_id="",
                pending_sensor_id="",
                status=SensorAttachmentStatus.NO_SENSOR,
                attachment_revision=persisted.attachment_revision,
            )

        revision = persisted.attachment_revision + 1
        self.state_store.save(
            PersistedSensorAttachmentSelection(
                sensor_id=persisted.sensor_id,
                attachment_revision=revision,
            )
        )
        return SensorAttachmentState(
            active_sensor_id="",
            pending_sensor_id=persisted.sensor_id,
            status=SensorAttachmentStatus.CONFIRMATION_PENDING,
            attachment_revision=revision,
        )

    def _require_command_lane_idle(self) -> None:
        if (
            self.command_controller.active_request_id
            or self.command_controller.queued_request_ids
        ):
            raise RuntimeError(
                "Cannot change sensor attachment while physical commands "
                "are active or queued"
            )
