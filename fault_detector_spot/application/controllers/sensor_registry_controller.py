"""Own persistent sensor definitions under application synchronization."""

from typing import Tuple

from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
)


class SensorRegistryController:
    """Serialize sensor definition mutations with attachment authority."""

    def __init__(
        self,
        sensor_repository,
        sensor_attachment_controller,
        command_controller,
    ):
        if (
            sensor_attachment_controller.sensor_repository
            is not sensor_repository
        ):
            raise ValueError(
                "Sensor registry and attachment controllers must share "
                "one sensor repository"
            )
        if (
            sensor_attachment_controller.command_controller
            is not command_controller
        ):
            raise ValueError(
                "Sensor registry and attachment controllers must share "
                "one command controller"
            )
        self.sensor_repository = sensor_repository
        self.sensor_attachment_controller = sensor_attachment_controller
        self.command_controller = command_controller

    def definitions(self) -> Tuple[SensorDefinition, ...]:
        """Return validated definitions in stable sensor ID order."""
        return tuple(self.sensor_repository.load_all())

    def create(self, definition: SensorDefinition) -> SensorDefinition:
        """Create one definition while the physical command lane is idle."""
        return self._mutate(
            lambda: self.sensor_repository.create(definition),
        )

    def update(self, definition: SensorDefinition) -> SensorDefinition:
        """Replace one definition when its attachment is not selected."""
        def update_definition():
            self._require_mutation_allowed(definition.sensor_id)
            return self.sensor_repository.update(definition)

        return self._mutate(update_definition)

    def delete(self, sensor_id: str) -> SensorDefinition:
        """Delete one definition when its attachment is not selected."""
        def delete_definition():
            self._require_mutation_allowed(sensor_id)
            return self.sensor_repository.delete(sensor_id)

        return self._mutate(delete_definition)

    def _mutate(self, mutation):
        return self.command_controller.run_if_idle(
            mutation,
            "Cannot change sensor definitions while physical commands "
            "are active or queued",
        )

    def _require_mutation_allowed(self, sensor_id: str) -> None:
        self.sensor_attachment_controller.require_definition_mutation_allowed(
            sensor_id
        )


__all__ = ["SensorRegistryController"]
