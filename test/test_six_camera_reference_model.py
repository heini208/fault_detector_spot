"""Tests for six persisted reference views."""

from fault_detector_spot.inspection.model.models import (
    InspectionRoutine,
    PoseData,
    ReferenceView,
)


CAMERAS = (
    "frontleft",
    "frontright",
    "left",
    "right",
    "back",
    "hand",
)


def make_view(slot_index, camera_id):
    return ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame=f"{camera_id}_frame",
        reference_dataset_path=(
            f"reference_datasets/scan/set/slot{slot_index + 1}_{camera_id}"
        ),
        view_id=f"slot{slot_index + 1}_{camera_id}",
        camera_id=camera_id,
        slot_index=slot_index,
    )


def test_reference_view_accepts_slot_five():
    make_view(5, "hand").validate()


def test_routine_accepts_six_reference_views():
    routine = InspectionRoutine(
        routine_id="scan",
        display_name="Scan",
        reference_views=[
            make_view(slot_index, camera_id)
            for slot_index, camera_id in enumerate(CAMERAS)
        ],
    )

    routine.validate()

    assert len(routine.reference_views) == 6
