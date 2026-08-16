"""Tests for extracted probe definition authoring."""

from dataclasses import replace

from fault_detector_spot.inspection.model.models import (
    PoseData,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.inspection.setup.probe_definition_service import (
    ProbeDefinitionService,
)


class FakeObjectRepository:
    def __init__(self):
        self.definitions = {}

    def create(self, definition):
        self.definitions[definition.object_id] = definition

    def load(self, object_id):
        if object_id not in self.definitions:
            raise FileNotFoundError(object_id)
        return self.definitions[object_id]

    def delete_object(self, object_id):
        return self.definitions.pop(object_id, None) is not None

    def add_routine(self, object_id, routine):
        definition = self.load(object_id)
        self.definitions[object_id] = replace(
            definition,
            routines=[*definition.routines, routine],
        )

    def delete_routine(self, object_id, routine_id):
        definition = self.load(object_id)
        self.definitions[object_id] = replace(
            definition,
            routines=[
                routine
                for routine in definition.routines
                if routine.routine_id != routine_id
            ],
        )


def service():
    objects = FakeObjectRepository()
    return ProbeDefinitionService(objects), objects


def test_definition_service_creates_and_selects_object_and_routine():
    definitions, objects = service()

    definition = definitions.create_object(
        "motor",
        "Motor",
        7,
        "36h11",
    )
    object_id, routine = definitions.create_routine(
        "motor",
        "magnetic_scan",
        "Magnetic scan",
    )

    assert definition.object_id == "motor"
    assert definition.reference_tag == ReferenceTag(
        tag_id=7,
        tag_family="36h11",
    )
    assert object_id == "motor"
    assert routine.routine_id == "magnetic_scan"
    assert definitions.select_object("motor") == "motor"
    assert definitions.select_routine(
        "motor",
        "magnetic_scan",
    ) == ("motor", "magnetic_scan")
    assert objects.load("motor").get_routine(
        "magnetic_scan"
    ) is not None


def test_definition_service_builds_snapshot_metadata():
    definitions, objects = service()
    definitions.create_object(
        "motor",
        "Motor",
        7,
        "36h11",
    )
    definitions.create_routine(
        "motor",
        "magnetic_scan",
        "Magnetic scan",
    )
    definition = objects.load("motor")
    routine = definition.get_routine("magnetic_scan")
    stored = replace(
        routine,
        reference_views=[
            ReferenceView(
                controlled_frame_pose_object=PoseData.identity(),
                controlled_frame="hand_depth",
                reference_dataset_path=(
                    "reference_datasets/magnetic_scan/set/slot1_hand"
                ),
                view_id="slot1_hand",
                camera_id="hand",
                slot_index=0,
            )
        ],
    )
    objects.definitions["motor"] = replace(
        definition,
        routines=[stored],
    )

    metadata = definitions.selected_definition_lists(
        "motor",
        "magnetic_scan",
        ("motor",),
    )

    assert metadata[0] == ("magnetic_scan",)
    assert metadata[1] == ("slot1_hand",)
    assert metadata[2] == ("hand",)
    assert metadata[4] == 7
    assert metadata[5] == "36h11"
    assert len(metadata) == 6


def test_definition_service_delete_operations_are_repository_owned():
    definitions, objects = service()
    definitions.create_object(
        "motor",
        "Motor",
        7,
        "36h11",
    )
    definitions.create_routine(
        "motor",
        "magnetic_scan",
        "Magnetic scan",
    )

    assert definitions.delete_routine(
        "motor",
        "magnetic_scan",
    ) == ("motor", "magnetic_scan")
    assert objects.load("motor").routines == []
    assert definitions.delete_object("motor") == "motor"
