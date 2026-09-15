"""Behavior checks for surface targets with a nontrivial sensor mount."""

import math

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeSetupGeometry,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup.reference_view_approach_direction import (
    APPROACH_SOURCE_SURFACE_FIT,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ProjectedReferencePoint,
)
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose
from fault_detector_spot.manipulation.probe_motion_planner import ProbeMotionPlanner


@pytest.mark.parametrize("mount_degrees", [(90, 0, 0), (10, 20, 30)])
def test_mounted_probe_keeps_requested_distance_after_hand_conversion(mount_degrees):
    pixel = ImagePoint(u=4, v=4)
    point = ProjectedReferencePoint(
        requested_pixel=pixel,
        mapped_pixel=pixel,
        sampled_pixel=pixel,
        point_camera=Vector3Data(x=0.0, y=0.0, z=1.0),
        frame_id="camera",
        depth_m=1.0,
    )
    mounting = PoseData(
        position=Vector3Data(x=0.12, y=-0.04, z=0.08),
        orientation=quaternion_from_euler(
            "xyz", [math.radians(angle) for angle in mount_degrees],
        ),
    )
    target = ProbeSetupGeometry._build_surface_target(
        projected_point=point,
        outward_direction_camera=Vector3Data(x=1.0, y=0.0, z=0.0),
        controlled_frame_pose_object=PoseData.identity(),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.15,
        hand_to_probe_pose=mounting,
        direction_source=APPROACH_SOURCE_SURFACE_FIT,
    )
    setup = initialize_reference_probe_setup(target)
    mount_rotation = rotation_from_quaternion(mounting.orientation).as_matrix()
    mount_translation = np.array([0.12, -0.04, 0.08])

    for probe, distance in [
        (setup.probe_pose_object, 0.03),
        (setup.aligned_preapproach_pose_object, 0.15),
    ]:
        desired = PoseStamped()
        desired.pose = pose_data_to_pose(probe)
        hand = ProbeMotionPlanner.probe_pose_to_hand_pose(
            desired, pose_data_to_pose(mounting),
        ).pose
        # Reconstruct the actual probe origin from the hand pose and mounting.
        q = hand.orientation
        hand_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        hand_position = np.array(
            [hand.position.x, hand.position.y, hand.position.z]
        )
        actual_probe = hand_position + hand_rotation @ mount_translation
        assert actual_probe == pytest.approx([distance, 0.0, 1.0], abs=1e-9)
        assert hand_position != pytest.approx(actual_probe)
        assert (hand_rotation @ mount_rotation)[:, 0] == pytest.approx(
            [-1.0, 0.0, 0.0], abs=1e-9,
        )
        if mount_degrees == (90, 0, 0):
            assert hand_rotation[:, 2] == pytest.approx(
                [0.0, 0.0, 1.0], abs=1e-9,
            )
