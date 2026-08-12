"""Object and routine definition operations for probe setup."""

from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class ProbeDefinitionService:
    """Own inspection object and routine definition persistence."""

    def __init__(self, object_repository, sensor_repository):
        self.object_repository = object_repository
        self.sensor_repository = sensor_repository

    def select_routine(
        self,
        object_id: str,
        routine_id: str,
    ) -> tuple[str, str]:
        object_name = self._name(object_id, "object ID")
        routine_name = self._name(routine_id, "routine ID")
        definition = self.object_repository.load(object_name)
        if definition.get_routine(routine_name) is None:
            raise LookupError(
                f"Unknown inspection routine: {object_name}/{routine_name}"
            )
        return object_name, routine_name

    def select_object(self, object_id: str) -> str:
        object_name = self._name(object_id, "object ID")
        self.object_repository.load(object_name)
        return object_name

    def create_object(
        self,
        object_id: str,
        display_name: str,
        reference_tag_id: int,
        reference_tag_family: str,
    ) -> InspectionObject:
        if (
            isinstance(reference_tag_id, bool)
            or not isinstance(reference_tag_id, int)
        ):
            raise TypeError("Reference tag ID must be an integer")
        definition = InspectionObject(
            object_id=self._name(object_id, "object ID"),
            display_name=self._text(
                display_name,
                "object display name",
            ),
            reference_tag=ReferenceTag(
                tag_id=reference_tag_id,
                tag_family=self._text(
                    reference_tag_family,
                    "reference tag family",
                ),
            ),
        )
        self.object_repository.create(definition)
        return definition

    def delete_object(self, object_id: str) -> str:
        object_name = self._name(object_id, "object ID")
        if not self.object_repository.delete_object(object_name):
            raise FileNotFoundError(
                f"Unknown inspection object: {object_name}"
            )
        return object_name

    def create_routine(
        self,
        object_id: str,
        routine_id: str,
        display_name: str,
        sensor_id: str,
    ) -> tuple[str, InspectionRoutine]:
        object_name = self._name(object_id, "object ID")
        sensor_name = self._name(sensor_id, "sensor ID")
        self.sensor_repository.load(sensor_name)
        routine = InspectionRoutine(
            routine_id=self._name(routine_id, "routine ID"),
            display_name=self._text(
                display_name,
                "routine display name",
            ),
            sensor_id=sensor_name,
        )
        self.object_repository.add_routine(object_name, routine)
        return object_name, routine

    def delete_routine(
        self,
        object_id: str,
        routine_id: str,
    ) -> tuple[str, str]:
        object_name = self._name(object_id, "object ID")
        routine_name = self._name(routine_id, "routine ID")
        self.object_repository.delete_routine(
            object_name,
            routine_name,
        )
        return object_name, routine_name

    def selected_definition_lists(
        self,
        selected_object_id: str,
        selected_routine_id: str,
        object_ids: tuple,
    ):
        if selected_object_id not in object_ids:
            return (), (), (), (), -1, "", ""
        definition = self.object_repository.load(selected_object_id)
        routine_ids = tuple(
            routine.routine_id for routine in definition.routines
        )
        routine = definition.get_routine(selected_routine_id)
        if routine is None:
            return (
                routine_ids,
                (),
                (),
                (),
                definition.reference_tag.tag_id,
                definition.reference_tag.tag_family,
                "",
            )
        return (
            routine_ids,
            tuple(view.view_id for view in routine.reference_views),
            tuple(view.camera_id for view in routine.reference_views),
            tuple(point.probe_point_id for point in routine.probe_points),
            definition.reference_tag.tag_id,
            definition.reference_tag.tag_family,
            routine.sensor_id,
        )

    @staticmethod
    def _name(value: str, label: str) -> str:
        return validate_storage_name(value.strip(), label)

    @staticmethod
    def _text(value: str, label: str) -> str:
        if not isinstance(value, str):
            raise TypeError(f"{label.title()} must be text")
        normalized = value.strip()
        if not normalized:
            raise ValueError(
                f"{label.title()} must not be empty"
            )
        if normalized != value:
            raise ValueError(
                f"{label.title()} must not contain surrounding whitespace"
            )
        return normalized


__all__ = ["ProbeDefinitionService"]
