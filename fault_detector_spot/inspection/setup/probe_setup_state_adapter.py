"""Translate immutable probe setup snapshots into ROS state messages."""

from uuid import uuid4

from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    SurfaceVerificationState,
)
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose


class ProbeSetupStateAdapter:
    """Create public ROS messages from server-owned probe setup state."""

    def __init__(self, clock):
        self.clock = clock

    def message(
        self,
        snapshot: ProbeSetupSnapshot,
        operation: int,
        state_code: int,
        detail: str,
        request_id: str = "",
        motion_operation: int = 0,
    ) -> ProbeSetupState:
        message = ProbeSetupState()
        message.header.stamp = self.clock.now().to_msg()
        message.request_id = request_id or uuid4().hex
        message.client_id = snapshot.context.client_id
        message.context_id = snapshot.context.context_id
        message.revision = snapshot.context.revision
        message.operation = int(operation)
        message.motion_operation = int(motion_operation)
        message.state = int(state_code)
        message.detail = detail
        message.validation_error = snapshot.validation_error
        message.dirty = snapshot.dirty
        message.selected_object_id = snapshot.selected_object_id
        message.selected_routine_id = snapshot.selected_routine_id
        message.selected_reference_view_id = (
            snapshot.selected_reference_view_id
        )
        message.selected_reference_tag_id = (
            snapshot.selected_reference_tag_id
        )
        message.selected_reference_tag_family = (
            snapshot.selected_reference_tag_family
        )
        message.object_ids = list(snapshot.object_ids)
        message.routine_ids = list(snapshot.routine_ids)
        message.reference_view_ids = list(snapshot.reference_view_ids)
        message.reference_camera_ids = list(snapshot.reference_camera_ids)
        message.probe_point_ids = list(snapshot.probe_point_ids)
        self._write_geometry(message, snapshot)
        self._write_refinement(message, snapshot)
        self._write_surface_verification(message, snapshot)
        return message

    @staticmethod
    def _write_refinement(message, snapshot):
        refinement = snapshot.refinement
        if refinement is None:
            return
        message.refinement_active = True
        stages = {
            RefinementStage.SAFE_APPROACH: (
                ProbeSetupState.REFINEMENT_STAGE_SAFE_APPROACH
            ),
            RefinementStage.ALIGNMENT: (
                ProbeSetupState.REFINEMENT_STAGE_ALIGNMENT
            ),
            RefinementStage.PROBE: ProbeSetupState.REFINEMENT_STAGE_PROBE,
        }
        message.refinement_stage = stages[refinement.active_stage]
        message.safe_approach_candidate_pose_object = pose_data_to_pose(
            refinement.candidate_pose(RefinementStage.SAFE_APPROACH)
        )
        message.aligned_preapproach_candidate_pose_object = (
            pose_data_to_pose(
                refinement.candidate_pose(RefinementStage.ALIGNMENT)
            )
        )
        message.probe_candidate_pose_object = pose_data_to_pose(
            refinement.candidate_pose(RefinementStage.PROBE)
        )
        states = {
            RefinementMotionState.NOT_TESTED: (
                ProbeSetupState.MOTION_NOT_TESTED
            ),
            RefinementMotionState.MOVING: ProbeSetupState.MOTION_MOVING,
            RefinementMotionState.REACHED: ProbeSetupState.MOTION_REACHED,
            RefinementMotionState.FAILED: ProbeSetupState.MOTION_FAILED,
        }
        message.safe_approach_motion_state = states[
            refinement.motion_states[RefinementStage.SAFE_APPROACH]
        ]
        message.alignment_motion_state = states[
            refinement.motion_states[RefinementStage.ALIGNMENT]
        ]
        message.probe_motion_state = states[
            refinement.motion_states[RefinementStage.PROBE]
        ]
        message.refinement_recovery_required = refinement.recovery_required
        message.refinement_recovery_message = refinement.recovery_message
        pending = refinement.pending_motion
        if pending is not None:
            message.motion_pending = True
            message.motion_request_id = pending.request_id

    @staticmethod
    def _write_surface_verification(message, snapshot):
        verification = snapshot.surface_verification
        if verification is None:
            return
        states = {
            SurfaceVerificationState.IDLE: (
                ProbeSetupState.SURFACE_VERIFICATION_IDLE
            ),
            SurfaceVerificationState.SAMPLING: (
                ProbeSetupState.SURFACE_VERIFICATION_SAMPLING
            ),
            SurfaceVerificationState.MOVING: (
                ProbeSetupState.SURFACE_VERIFICATION_MOVING
            ),
            SurfaceVerificationState.SETTLING: (
                ProbeSetupState.SURFACE_VERIFICATION_SETTLING
            ),
            SurfaceVerificationState.CONVERGED: (
                ProbeSetupState.SURFACE_VERIFICATION_CONVERGED
            ),
            SurfaceVerificationState.FAILED: (
                ProbeSetupState.SURFACE_VERIFICATION_FAILED
            ),
            SurfaceVerificationState.CANCELLED: (
                ProbeSetupState.SURFACE_VERIFICATION_CANCELLED
            ),
            SurfaceVerificationState.RECOVERY_REQUIRED: (
                ProbeSetupState.SURFACE_VERIFICATION_RECOVERY_REQUIRED
            ),
        }
        message.surface_verification_active = verification.active
        message.surface_verification_state = states[verification.state]
        message.surface_verification_request_id = verification.request_id
        message.surface_distance_tolerance_m = verification.policy.tolerance_m
        if verification.measured_distance_m is not None:
            message.has_surface_distance_measurement = True
            message.measured_surface_distance_m = (
                verification.measured_distance_m
            )
            message.surface_distance_error_m = verification.error_m
        if verification.last_correction_m is not None:
            message.has_surface_correction = True
            message.last_surface_correction_m = verification.last_correction_m
        message.cumulative_surface_correction_m = (
            verification.cumulative_correction_m
        )
        message.surface_verification_iteration = verification.iteration_count
        message.surface_recovery_required = verification.recovery_required

    @classmethod
    def _write_geometry(cls, message, snapshot):
        cls._write_reference_geometry(message, snapshot)
        cls._write_probe_setup(message, snapshot)

    @classmethod
    def _write_reference_geometry(cls, message, snapshot):
        if snapshot.reference_pixel is not None:
            message.has_reference_pixel = True
            message.reference_pixel_u = snapshot.reference_pixel.u
            message.reference_pixel_v = snapshot.reference_pixel.v
        geometry = snapshot.geometry
        if geometry is None:
            return
        cls._write_projected_point(message, geometry.projected_point)
        cls._write_normal_and_approach(message, geometry)

    @staticmethod
    def _write_projected_point(message, projected):
        message.has_surface_point = True
        message.surface_point_camera.x = projected.point_camera.x
        message.surface_point_camera.y = projected.point_camera.y
        message.surface_point_camera.z = projected.point_camera.z
        message.surface_frame = projected.frame_id
        message.mapped_depth_u = projected.mapped_pixel.u
        message.mapped_depth_v = projected.mapped_pixel.v
        message.sampled_depth_u = projected.sampled_pixel.u
        message.sampled_depth_v = projected.sampled_pixel.v
        message.depth_m = projected.depth_m

    @staticmethod
    def _write_normal_and_approach(message, geometry):
        message.surface_normal_error = geometry.surface_normal_error
        if geometry.surface_normal is not None:
            normal = geometry.surface_normal
            message.has_surface_normal = True
            message.surface_normal_camera.x = normal.normal_camera.x
            message.surface_normal_camera.y = normal.normal_camera.y
            message.surface_normal_camera.z = normal.normal_camera.z
            message.surface_normal_sample_count = normal.sample_count
            message.surface_normal_rmse_m = normal.plane_rmse_m
        direction = geometry.approach_direction
        message.has_approach_direction = True
        message.approach_direction_camera.x = direction.direction_camera.x
        message.approach_direction_camera.y = direction.direction_camera.y
        message.approach_direction_camera.z = direction.direction_camera.z
        message.approach_source = direction.source

    @classmethod
    def _write_probe_setup(cls, message, snapshot):
        setup = snapshot.setup
        if setup is None:
            return
        message.has_probe_setup = True
        target = setup.surface_target
        cls._write_surface_target(message, target)
        calculated = snapshot.geometry.probe_setup
        message.calculated_safe_approach_pose_object = pose_data_to_pose(
            calculated.safe_approach_pose_object
        )
        message.calculated_aligned_preapproach_pose_object = (
            pose_data_to_pose(
                calculated.aligned_preapproach_pose_object
            )
        )
        message.calculated_probe_pose_object = pose_data_to_pose(
            calculated.probe_pose_object
        )
        cls._write_approved_poses(message, setup)
        message.target_surface_distance_m = (
            target.target_surface_distance_m
        )
        message.aligned_preapproach_distance_m = (
            target.aligned_preapproach_distance_m
        )

    @staticmethod
    def _write_surface_target(message, target):
        message.surface_point_object.x = target.surface_point_object.x
        message.surface_point_object.y = target.surface_point_object.y
        message.surface_point_object.z = target.surface_point_object.z
        message.outward_direction_object.x = (
            target.outward_direction_object.x
        )
        message.outward_direction_object.y = (
            target.outward_direction_object.y
        )
        message.outward_direction_object.z = (
            target.outward_direction_object.z
        )

    @staticmethod
    def _write_approved_poses(message, setup):
        message.safe_approach_pose_object = pose_data_to_pose(
            setup.safe_approach_pose_object
        )
        message.aligned_preapproach_pose_object = pose_data_to_pose(
            setup.aligned_preapproach_pose_object
        )
        message.probe_pose_object = pose_data_to_pose(
            setup.probe_pose_object
        )
        message.safe_approach_approved = setup.safe_approach_approved
        message.surface_alignment_approved = (
            setup.surface_alignment_approved
        )
        message.probe_pose_approved = setup.probe_pose_approved


__all__ = ["ProbeSetupStateAdapter"]
