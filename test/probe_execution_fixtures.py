"""Shared object, pose, and attachment fixtures for probe execution tests."""

import math

from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    MotionAttachmentSnapshot,
    SensorDefinition,
)


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def yaw_quaternion(degrees):
    radians = math.radians(degrees) * 0.5
    return QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(radians),
        w=math.cos(radians),
    )


def inspection_object():
    view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/scan/set_1/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )
    point = ProbePoint(
        probe_point_id="point_1",
        display_name="Point 1",
        aligned_preapproach_pose_object=pose(
            x=0.10,
            orientation=yaw_quaternion(180.0),
        ),
        target_surface_distance_m=0.03,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        measurement_duration_sec=1.5,
        aligned_preapproach_distance_m=0.10,
        reference_view_id="slot1_hand",
    )
    routine = InspectionRoutine(
        safe_approach_pose_object=pose(x=0.30),
        routine_id="scan",
        display_name="Scan",
        reference_tag=ReferenceTag(
            tag_id=2,
            tag_family="36h11",
        ),
        reference_views=[view],
        probe_points=[point],
    )
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        routines=[routine],
    )


def sensor(sensor_id="bmm150_01", hand_to_probe=None):
    return MotionAttachmentSnapshot.from_definition(
        SensorDefinition(
            sensor_id=sensor_id,
            display_name="BMM150 01",
            hand_to_probe=hand_to_probe or pose(x=0.20),
        ),
        attachment_revision=7,
    )
