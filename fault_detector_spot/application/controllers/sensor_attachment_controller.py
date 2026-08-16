"""Own authoritative physical inspection sensor attachment state."""

from contextlib import contextmanager
from dataclasses import dataclass, replace
from enum import Enum
from threading import RLock
from typing import Tuple

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    NO_SENSOR_MOUNT_ID,
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    PersistedSensorAttachmentSelection,
    SensorAttachmentStateStore,
)


_AUTOMATIC_SENSOR_COMMANDS = frozenset({
    CommandID.MOVE_ARM_TO_TAG,
    CommandID.MOVE_ARM_TO_TAG_AND_WAIT,
    CommandID.SCAN_ALL_IN_RANGE,
})


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
        self._reservation_count = 0
        self._state = self._restore_state()

    def snapshot(self) -> SensorAttachmentState:
        """Return the current immutable attachment state."""
        with self._lock:
            return self._state

    def select_sensor(self, sensor_id: str) -> SensorAttachmentState:
        """Select one registered sensor and require confirmation."""
        def select():
            with self._lock:
                self._require_no_reservation_locked()
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

        return self._change_while_idle(select)

    def clear_sensor(self) -> SensorAttachmentState:
        """Select the built-in no-sensor mount for confirmation."""
        return self.select_sensor(NO_SENSOR_MOUNT_ID)

    def confirm_sensor(
        self,
        sensor_id: str,
        attachment_revision: int,
    ) -> SensorAttachmentState:
        """Confirm the exact pending physical sensor attachment."""
        def confirm():
            with self._lock:
                self._require_no_reservation_locked()
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

        return self._change_while_idle(confirm)

    def require_confirmed_sensor(
        self,
        expected_sensor_id: str = "",
    ) -> ConfirmedSensorAttachment:
        """Return frozen geometry for the current confirmed sensor."""
        with self._lock:
            attachment = self._confirmed_attachment_locked()
            expected = expected_sensor_id.strip()
            if expected and expected != attachment.sensor_id:
                raise RuntimeError(
                    "Expected sensor "
                    f"'{expected}' but the confirmed attachment is "
                    f"'{attachment.sensor_id}'"
                )
            return attachment

    @contextmanager
    def reserve_confirmed_attachment(self):
        """Freeze one confirmed attachment across a multi-step workflow."""
        def acquire():
            with self._lock:
                attachment = self._confirmed_attachment_locked()
                self._reservation_count += 1
                return attachment

        attachment = self.command_controller.run_if_idle(
            acquire,
            "Cannot start sensor-dependent workflow while physical "
            "commands are active or queued",
        )
        try:
            yield attachment
        finally:
            with self._lock:
                self._reservation_count -= 1

    def prepare_request(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> CommandRequest[SemanticCommand]:
        """Bind or validate active sensor geometry at command admission."""
        command = request.command
        requires_automatic_binding = (
            command.command_id in _AUTOMATIC_SENSOR_COMMANDS
        )
        if not requires_automatic_binding and not command.motion_sensor_id:
            return request

        with self._lock:
            attachment = self._confirmed_attachment_locked()
            if (
                command.motion_sensor_id
                and command.motion_sensor_id != attachment.sensor_id
            ):
                raise RuntimeError(
                    "Motion was prepared for sensor "
                    f"'{command.motion_sensor_id}' but the confirmed "
                    f"attachment is '{attachment.sensor_id}'"
                )
            if command.motion_sensor_id:
                return request
            return replace(
                request,
                command=replace(
                    command,
                    motion_sensor_id=attachment.sensor_id,
                ),
            )

    def _confirmed_attachment_locked(self) -> ConfirmedSensorAttachment:
        if not self._state.sensor_dependent_motion_allowed:
            raise RuntimeError(
                "A confirmed physical sensor attachment is required"
            )
        definition = self.sensor_repository.load(
            self._state.active_sensor_id
        )
        definition.validate()
        return ConfirmedSensorAttachment.from_definition(
            definition,
            self._state.attachment_revision,
        )

    def _change_while_idle(self, action):
        return self.command_controller.run_if_idle(
            action,
            "Cannot change sensor attachment while physical commands "
            "are active or queued",
        )

    def _require_no_reservation_locked(self) -> None:
        if self._reservation_count:
            raise RuntimeError(
                "Cannot change sensor attachment while a sensor-dependent "
                "workflow is active"
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
