"""Tests for transient probe setup approval and frame conversion."""

import math

import pytest

from fault_detector_spot.inspection.data.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    compose_poses,
    derive_aligned_preapproach_pose,
    initialize_reference_probe_setup,
    invalidate_probe_setup_approvals,
    probe_pose_to_hand_pose,
    refine_probe_pose,
    relative_pose,
    rotate_vector,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
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


def roll_quaternion(degrees):
    radians = math.radians(degrees) * 0.5
    return QuaternionData(
        x=math.sin(radians),
        y=0.0,
        z=0.0,
        w=math.cos(radians),
    )


def test_initial_setup_uses_calculated_poses():
    setup = initialize_reference_probe_setup(target())

    assert setup.safe_approach_pose_object.position.x == 0.15
    assert setup.aligned_preapproach_pose_object.position.x == 0.15
    assert setup.probe_pose_object.position.x == 0.03
    assert not setup.safe_approach_approved


def test_initial_setup_compensates_sensor_roll_with_level_hand_pose():
    mounting = pose(orientation=roll_quaternion(90.0))

    setup = initialize_reference_probe_setup(target(), mounting)
    hand_pose = probe_pose_to_hand_pose(
        setup.probe_pose_object,
        mounting,
    )
    inward = rotate_vector(
        setup.probe_pose_object.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )

    assert hand_pose.orientation == QuaternionData.identity()
    assert inward.x == pytest.approx(1.0)
    assert inward.y == pytest.approx(0.0)
    assert inward.z == pytest.approx(0.0)


def test_approach_capture_changes_only_the_independent_safe_pose():
    setup = initialize_reference_probe_setup(target())
    current = pose(x=0.4, y=0.2, z=0.5, orientation=yaw_quaternion(30.0))

    approved = approve_safe_approach_pose(setup, current)

    assert approved.safe_approach_pose_object == current
    assert approved.aligned_preapproach_pose_object.position.x == 0.15
    assert approved.probe_pose_object.position.x == 0.03
    assert approved.aligned_preapproach_pose_object.orientation == (
        QuaternionData.identity()
    )
    assert approved.probe_pose_object.orientation == (
        QuaternionData.identity()
    )
    assert approved.safe_approach_approved
    assert not approved.surface_alignment_approved
    assert not approved.probe_pose_approved


def test_alignment_capture_rebuilds_probe_along_local_positive_x():
    setup = initialize_reference_probe_setup(target())
    current = pose(x=0.20, y=0.10, z=0.30)

    approved = approve_surface_alignment_pose(setup, current)

    assert approved.aligned_preapproach_pose_object == current
    assert math.isclose(approved.probe_pose_object.position.x, 0.32)
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
        -0.08,
    )
    assert approved.surface_alignment_approved
    assert approved.probe_pose_approved


def test_aligned_pose_is_derived_opposite_probe_local_positive_x():
    probe = pose(
        x=0.04,
        y=0.10,
        z=0.20,
        orientation=yaw_quaternion(90.0),
    )

    aligned = derive_aligned_preapproach_pose(
        probe,
        target_surface_distance_m=0.04,
        aligned_preapproach_distance_m=0.14,
    )

    assert aligned.position.x == pytest.approx(0.04)
    assert aligned.position.y == pytest.approx(0.0)
    assert aligned.position.z == pytest.approx(0.20)
    assert aligned.orientation == probe.orientation


def test_later_approvals_preserve_the_approved_safe_pose():
    """Later stages cannot rewrite a previously approved safe pose."""
    setup = initialize_reference_probe_setup(target())
    safe_pose = pose(
        x=0.4,
        y=0.2,
        z=0.5,
        orientation=yaw_quaternion(30.0),
    )
    setup = approve_safe_approach_pose(setup, safe_pose)
    setup = approve_surface_alignment_pose(
        setup,
        pose(x=0.20, orientation=yaw_quaternion(15.0)),
    )
    setup = approve_probe_pose(
        setup,
        pose(x=0.04, orientation=yaw_quaternion(10.0)),
    )

    assert setup.safe_approach_pose_object == safe_pose


def test_reapproving_approach_preserves_independent_later_approvals():
    """Changing the independent safe pose cannot rewrite geometry."""
    setup = initialize_reference_probe_setup(target())
    setup = approve_surface_alignment_pose(setup, pose(x=0.20))
    setup = approve_probe_pose(setup, pose(x=0.04))

    updated = approve_safe_approach_pose(setup, pose(x=0.40))

    assert updated.safe_approach_approved
    assert updated.surface_alignment_approved
    assert updated.probe_pose_approved


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


def test_refinement_translation_uses_fixed_surface_axes():
    current = pose(x=0.4, y=0.2, z=0.5)

    refined = refine_probe_pose(
        current,
        yaw_quaternion(90.0),
        local_translation=Vector3Data(x=0.0, y=0.01, z=0.02),
    )

    assert refined.position.x == pytest.approx(0.39)
    assert refined.position.y == pytest.approx(0.20)
    assert refined.position.z == pytest.approx(0.52)


def test_refinement_rotation_keeps_the_sensor_tip_position_fixed():
    refined = refine_probe_pose(
        pose(),
        QuaternionData.identity(),
        pitch_rad=math.radians(-5.0),
        yaw_rad=math.radians(3.0),
    )

    assert refined.orientation != QuaternionData.identity()
    assert refined.position == Vector3Data.zero()
    assert math.isclose(
        math.sqrt(
            refined.orientation.x ** 2
            + refined.orientation.y ** 2
            + refined.orientation.z ** 2
            + refined.orientation.w ** 2
        ),
        1.0,
        abs_tol=1e-9,
    )


def test_refinement_invalidates_stage_and_dependent_approvals():
    setup = initialize_reference_probe_setup(target())
    setup = approve_safe_approach_pose(setup, pose(x=0.30))
    setup = approve_surface_alignment_pose(setup, pose(x=0.15))
    setup = approve_probe_pose(setup, pose(x=0.03))

    alignment_changed = invalidate_probe_setup_approvals(
        setup,
        "alignment",
    )
    probe_changed = invalidate_probe_setup_approvals(setup, "probe")

    assert alignment_changed.safe_approach_approved
    assert not alignment_changed.surface_alignment_approved
    assert not alignment_changed.probe_pose_approved
    assert probe_changed.safe_approach_approved
    assert probe_changed.surface_alignment_approved
    assert not probe_changed.probe_pose_approved
