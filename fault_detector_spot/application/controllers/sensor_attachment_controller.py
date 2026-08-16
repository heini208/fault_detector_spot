"""Own authoritative physical inspection sensor attachment state."""

from contextlib import contextmanager
from dataclasses import dataclass, replace
from enum import Enum
from threading import RLock

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    MotionAttachmentSnapshot,
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
    """Describe authoritative physical sensor attachment state."""

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
        """Return the selected sensor identity for the current state."""
        if self.status is SensorAttachmentStatus.CONFIRMATION_PENDING:
            return self.pending_sensor_id
        return self.active_sensor_id

    @property
    def sensor_dependent_motion_allowed(self) -> bool:
        """Return whether effective hand/probe geometry is trustworthy."""
        return (
            self.status is SensorAttachmentStatus.ACTIVE
            and not self.pending_sensor_id
        )


class MotionAttachmentReservation:
    """Own one explicitly released immutable attachment snapshot."""

    def __init__(self, controller, attachment):
        self._controller = controller
        self._attachment = attachment
        self._released = False

    @property
    def attachment(self) -> MotionAttachmentSnapshot:
        """Return geometry frozen when the reservation was acquired."""
        return self._attachment

    @property
    def released(self) -> bool:
        """Return whether this reservation no longer blocks changes."""
        return self._released

    def release(self) -> bool:
        """Release exactly once and report whether this call released it."""
        return self._controller._release_motion_attachment(self)


class SensorAttachmentController:
    """Own sensor selection, confirmation, and motion admission."""

    def __init__(
        self,
        sensor_repository,
        state_store: SensorAttachmentStateStore,
        command_controller,
    ):
        """Restore selection while invalidating prior sensor confirmation."""
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
        """Select one registered physical sensor and require confirmation."""
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
        """Select the bare hand and require physical confirmation."""
        def clear():
            with self._lock:
                self._require_no_reservation_locked()
                revision = self._state.attachment_revision + 1
                self.state_store.clear()
                self._state = SensorAttachmentState(
                    active_sensor_id="",
                    pending_sensor_id="",
                    status=SensorAttachmentStatus.NO_SENSOR,
                    attachment_revision=revision,
                )
                return self._state

        return self._change_while_idle(clear)

    def confirm_sensor(
        self,
        sensor_id: str,
        attachment_revision: int,
    ) -> SensorAttachmentState:
        """Confirm the exact pending sensor or bare-hand state."""
        def confirm():
            with self._lock:
                self._require_no_reservation_locked()
                if self._state.status not in {
                    SensorAttachmentStatus.NO_SENSOR,
                    SensorAttachmentStatus.CONFIRMATION_PENDING,
                }:
                    raise RuntimeError(
                        "No sensor attachment confirmation is pending"
                    )
                if attachment_revision != self._state.attachment_revision:
                    raise RuntimeError(
                        "Sensor attachment confirmation uses a stale revision"
                    )

                normalized = sensor_id.strip()
                if self._state.status is SensorAttachmentStatus.NO_SENSOR:
                    if normalized:
                        raise RuntimeError(
                            "Sensor attachment confirmation does not match "
                            "the pending no-sensor state"
                        )
                    active_sensor_id = ""
                else:
                    if normalized != self._state.pending_sensor_id:
                        raise RuntimeError(
                            "Sensor attachment confirmation does not match "
                            "the pending sensor"
                        )
                    definition = self.sensor_repository.load(normalized)
                    definition.validate()
                    active_sensor_id = definition.sensor_id

                self._state = SensorAttachmentState(
                    active_sensor_id=active_sensor_id,
                    pending_sensor_id="",
                    status=SensorAttachmentStatus.ACTIVE,
                    attachment_revision=self._state.attachment_revision,
                )
                return self._state

        return self._change_while_idle(confirm)

    def require_motion_attachment(self) -> MotionAttachmentSnapshot:
        """Return effective geometry for the active sensor or bare hand."""
        with self._lock:
            return self._motion_attachment_locked()

    def require_confirmed_sensor(
        self,
        expected_sensor_id: str = "",
    ) -> MotionAttachmentSnapshot:
        """Return one confirmed physical sensor attachment."""
        with self._lock:
            attachment = self._motion_attachment_locked()
            if not attachment.has_sensor:
                raise RuntimeError(
                    "A confirmed physical sensor attachment is required"
                )
            expected = expected_sensor_id.strip()
            if expected and expected != attachment.sensor_id:
                raise RuntimeError(
                    "Expected sensor "
                    f"'{expected}' but the confirmed attachment is "
                    f"'{attachment.sensor_id}'"
                )
            return attachment

    def require_definition_mutation_allowed(
        self,
        sensor_id: str,
    ) -> None:
        """Reject mutation of selected or reserved sensor geometry."""
        if not isinstance(sensor_id, str) or not sensor_id.strip():
            raise ValueError("Sensor ID must not be empty")
        normalized = sensor_id.strip()
        with self._lock:
            if self._reservation_count:
                raise RuntimeError(
                    "Cannot change sensor definitions while a "
                    "sensor-dependent workflow is active"
                )
            if normalized in {
                self._state.active_sensor_id,
                self._state.pending_sensor_id,
            }:
                raise RuntimeError(
                    f"Sensor '{normalized}' is currently selected. Remove "
                    "or select another sensor before editing or deleting it."
                )

    @contextmanager
    def reserve_motion_attachment(self):
        """Freeze effective geometry across a multi-step motion workflow."""
        reservation = self.acquire_motion_attachment()
        try:
            yield reservation.attachment
        finally:
            reservation.release()

    def acquire_motion_attachment(self) -> MotionAttachmentReservation:
        """Acquire geometry for a workflow spanning multiple API calls."""
        def acquire():
            with self._lock:
                attachment = self._motion_attachment_locked()
                self._reservation_count += 1
                return MotionAttachmentReservation(self, attachment)

        return self.command_controller.run_if_idle(
            acquire,
            "Cannot start sensor-dependent workflow while physical "
            "commands are active or queued",
        )

    def _release_motion_attachment(
        self,
        reservation: MotionAttachmentReservation,
    ) -> bool:
        with self._lock:
            if reservation._controller is not self:
                raise ValueError(
                    "Motion attachment reservation belongs to another "
                    "controller"
                )
            if reservation._released:
                return False
            if self._reservation_count <= 0:
                raise RuntimeError(
                    "Motion attachment reservation state is inconsistent"
                )
            reservation._released = True
            self._reservation_count -= 1
            return True

    def prepare_request(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> CommandRequest[SemanticCommand]:
        """Bind or validate effective probe geometry at command admission."""
        command = request.command
        requires_automatic_binding = (
            command.command_id in _AUTOMATIC_SENSOR_COMMANDS
        )
        if not requires_automatic_binding and not command.motion_sensor_id:
            return request

        with self._lock:
            attachment = self._motion_attachment_locked()
            motion_sensor_id = attachment.motion_sensor_id
            if (
                command.motion_sensor_id
                and command.motion_sensor_id != motion_sensor_id
            ):
                raise RuntimeError(
                    "Motion was prepared for attachment "
                    f"'{command.motion_sensor_id}' but the current "
                    f"attachment is '{motion_sensor_id}'"
                )
            if command.motion_sensor_id:
                return request
            return replace(
                request,
                command=replace(
                    command,
                    motion_sensor_id=motion_sensor_id,
                ),
            )

    def _motion_attachment_locked(self) -> MotionAttachmentSnapshot:
        if not self._state.sensor_dependent_motion_allowed:
            raise RuntimeError(
                "Sensor attachment confirmation is pending"
            )
        if not self._state.active_sensor_id:
            return MotionAttachmentSnapshot.bare_hand(
                self._state.attachment_revision
            )
        definition = self.sensor_repository.load(
            self._state.active_sensor_id
        )
        definition.validate()
        return MotionAttachmentSnapshot.from_definition(
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
        if (
            not persisted.sensor_id
            or persisted.sensor_id == BARE_HAND_MOTION_ID
        ):
            self.state_store.clear()
            return SensorAttachmentState(
                active_sensor_id="",
                pending_sensor_id="",
                status=SensorAttachmentStatus.NO_SENSOR,
                attachment_revision=persisted.attachment_revision,
            )

        try:
            definition = self.sensor_repository.load(persisted.sensor_id)
            definition.validate()
        except FileNotFoundError:
            self.state_store.clear()
            return SensorAttachmentState(
                active_sensor_id="",
                pending_sensor_id="",
                status=SensorAttachmentStatus.NO_SENSOR,
                attachment_revision=persisted.attachment_revision,
            )

        revision = persisted.attachment_revision + 1
        self.state_store.save(
            PersistedSensorAttachmentSelection(
                sensor_id=definition.sensor_id,
                attachment_revision=revision,
            )
        )
        return SensorAttachmentState(
            active_sensor_id="",
            pending_sensor_id=definition.sensor_id,
            status=SensorAttachmentStatus.CONFIRMATION_PENDING,
            attachment_revision=revision,
        )
