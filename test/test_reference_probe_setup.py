"""Tests for transient probe setup approval and frame conversion."""

import math

from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    compose_poses,
    initialize_reference_probe_setup,
    probe_pose_to_hand_pose,
    relative_pose,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def target():
    return ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(x=0.03),
        aligned_preapproach_pose_object=pose(x=0.15),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.15,
        direction_source="surface_fit",
    )


def yaw_quaternion(degrees):
    radians = math.radians(degrees) * 0.5
    return QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(radians),
        w=math.cos(radians),
    )


def test_initial_setup_uses_calculated_poses():
    setup = initialize_reference_probe_setup(target())

    assert setup.safe_approach_pose_object.position.x == 0.15
    assert setup.aligned_preapproach_pose_object.position.x == 0.15
    assert setup.probe_pose_object.position.x == 0.03
    assert not setup.safe_approach_approved


def test_approach_capture_preserves_positions_and_shares_orientation():
    setup = initialize_reference_probe_setup(target())
    current = pose(x=0.4, y=0.2, z=0.5, orientation=yaw_quaternion(30.0))

    approved = approve_safe_approach_pose(setup, current)

    assert approved.safe_approach_pose_object == current
    assert approved.aligned_preapproach_pose_object.position.x == 0.15
    assert approved.probe_pose_object.position.x == 0.03
    assert approved.aligned_preapproach_pose_object.orientation == (
        current.orientation
    )
    assert approved.probe_pose_object.orientation == current.orientation
    assert approved.safe_approach_approved


def test_alignment_capture_rebuilds_probe_along_local_positive_x():
    setup = initialize_reference_probe_setup(target())
    current = pose(x=0.20, y=0.10, z=0.30)

    approved = approve_surface_alignment_pose(setup, current)

    assert approved.aligned_preapproach_pose_object == current
    assert math.isclose(approved.probe_pose_object.position.x, 0.08)
    assert math.isclose(approved.probe_pose_object.position.y, 0.10)
    assert math.isclose(approved.probe_pose_object.position.z, 0.30)
    assert approved.surface_alignment_approved
    assert not approved.probe_pose_approved


def test_probe_capture_rebuilds_aligned_pose():
    setup = initialize_reference_probe_setup(target())
    current = pose(x=0.04, y=-0.10, z=0.20)

    approved = approve_probe_pose(setup, current)

    assert approved.probe_pose_object == current
    assert math.isclose(
        approved.aligned_preapproach_pose_object.position.x,
        0.16,
    )
    assert approved.surface_alignment_approved
    assert approved.probe_pose_approved


def test_probe_pose_is_converted_to_required_hand_pose():
    desired_probe = pose(x=1.0, y=2.0, z=3.0)
    hand_to_probe = pose(x=0.10, y=0.0, z=0.0)

    desired_hand = probe_pose_to_hand_pose(desired_probe, hand_to_probe)

    assert math.isclose(desired_hand.position.x, 0.90)
    assert math.isclose(desired_hand.position.y, 2.0)
    assert math.isclose(desired_hand.position.z, 3.0)


def test_relative_pose_round_trip():
    parent_to_reference = pose(
        x=1.0,
        y=2.0,
        z=0.5,
        orientation=yaw_quaternion(90.0),
    )
    reference_to_target = pose(x=0.2, y=-0.1, z=0.3)
    parent_to_target = compose_poses(
        parent_to_reference,
        reference_to_target,
    )

    recovered = relative_pose(parent_to_reference, parent_to_target)

    assert math.isclose(recovered.position.x, 0.2, abs_tol=1e-9)
    assert math.isclose(recovered.position.y, -0.1, abs_tol=1e-9)
    assert math.isclose(recovered.position.z, 0.3, abs_tol=1e-9)
