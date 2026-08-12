"""Regression guards for the current inspection model contract."""

from fault_detector_spot.inspection.model.models import (
    InspectionRoutine,
    PoseData,
    ReferenceView,
)


def stored_view(view_id, camera_id, slot_index):
    return ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame=f"{camera_id}_frame",
        reference_dataset_path=(
            f"reference_datasets/routine/set/{view_id}"
        ),
        view_id=view_id,
        camera_id=camera_id,
        slot_index=slot_index,
    )


def test_routine_has_one_authoritative_multi_view_collection():
    first = stored_view("slot1_left", "left", 0)
    second = stored_view("slot2_right", "right", 1)
    routine = InspectionRoutine(
        routine_id="routine",
        display_name="Routine",
        sensor_id="sensor",
        reference_views=[first, second],
    )

    assert routine.reference_views == [first, second]
    assert "reference_view" not in vars(InspectionRoutine)
    assert "reference_view" not in routine.to_dict()
    assert routine.to_dict()["reference_views"] == [
        first.to_dict(),
        second.to_dict(),
    ]


def test_routine_round_trip_preserves_all_reference_views():
    routine = InspectionRoutine(
        routine_id="routine",
        display_name="Routine",
        sensor_id="sensor",
        reference_views=[
            stored_view("slot1_left", "left", 0),
            stored_view("slot2_right", "right", 1),
            stored_view("slot3_hand", "hand", 2),
        ],
    )

    restored = InspectionRoutine.from_dict(
        routine.to_dict()
    )

    assert restored == routine
    assert [
        view.view_id
        for view in restored.reference_views
    ] == [
        "slot1_left",
        "slot2_right",
        "slot3_hand",
    ]
