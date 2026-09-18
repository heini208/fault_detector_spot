from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeSetupMotionCommandFactory,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)


def pose(x=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )


def setup():
    target = ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(0.03),
        aligned_preapproach_pose_object=pose(0.10),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.10,
        direction_source="test",
    )
    return initialize_reference_probe_setup(target)


def test_new_refinement_waits_for_ready_arm_candidate():
    session = ProbeRefinementSession.create(setup())

    assert not session.safe_approach_ready_seeded
    assert session.motion_states[RefinementStage.SAFE_APPROACH] is (
        RefinementMotionState.NOT_TESTED
    )


def test_ready_safe_approach_has_its_own_semantic_command():
    command = ProbeSetupMotionCommandFactory.ready_safe_approach()

    assert command.command_id is CommandID.READY_SAFE_APPROACH


def test_safe_approach_speed_is_faster_than_aligned_default():
    config = ArmMotionParameters()

    assert config.get("safe_approach_linear_speed_mps") > config.get(
        "motion.linear_speed_mps"
    )
    assert config.get("safe_approach_angular_speed_rad_s") > config.get(
        "motion.angular_speed_rad_s"
    )
