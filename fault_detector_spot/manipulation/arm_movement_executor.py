"""Centralized Spot arm movement execution."""

from copy import deepcopy
import math
import time

from bosdyn.api import (
    arm_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
)
from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from spot_msgs.srv import RobotCommand as RobotCommandService
from synchros2.utilities import namespace_with
from fault_detector_spot.inspection.geometry.rotation import (
    multiply_quaternions,
    rotate_vector,
)
from fault_detector_spot.inspection.model.models import Vector3Data
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.geometry.alignment_orientation import (
    surface_aligned_probe_orientation,
    tag_aligned_probe_orientation,
)
from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
)
from fault_detector_spot.manipulation.guarded_probe_execution import (
    GuardedProbeExecution,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlanner,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data
from fault_detector_spot.shared.execution.movement_executor import (
    DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC,
    DEFAULT_RESULT_TIMEOUT_SEC,
    MovementExecutor,
)

from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)


class _ArmOperation:
    MOVEMENT = "movement"
    READY_WAIT = "ready_wait"
    READY_PREPARE = "ready_prepare"
    GUARDED_MOVEMENT = "guarded_movement"
    PREPARE = "prepare"
    STOW = "stow"


class ArmMovementExecutor(MovementExecutor):
    """Coordinate readiness, guarded probe motion, and low-level probe motion."""

    OUTCOME_TYPE = ArmMovementOutcome
    UPDATE_TYPE = ArmMovementUpdate
    MOVEMENT_NAME = "arm"

    def __init__(
        self,
        tf_listener,
        tag_state_source=None,
        robot_name: str = "",
        speed_policy=None,
        action_client=None,
        arm_stop_service_client=None,
        arm_state_source=None,
        surface_source=None,
        force_baseline_sampler=None,
        force_contact_policy=None,
        contact_evidence_analyzer=None,
        force_stale_timeout_sec=None,
        contact_retreat_distance_m=None,
        contact_retreat_speed_mps=None,
        ready_forward_distance_m=None,
        ready_lift_distance_m=None,
        ready_state_timeout_sec=None,
        ready_tf_timeout_sec=None,
        ready_deployed_timeout_sec=None,
        stow_state_timeout_sec=None,
        goal_response_timeout_sec: float = (
            DEFAULT_GOAL_RESPONSE_TIMEOUT_SEC
        ),
        result_timeout_sec: float = DEFAULT_RESULT_TIMEOUT_SEC,
        monotonic_clock=time.monotonic,
        logger=None,
        config=None,
    ):
        config = config if config is not None else ArmMotionParameters(
            getattr(arm_state_source, "node", None)
        )
        ready_forward_distance_m = config.get(
            "ready_forward_distance_m", ready_forward_distance_m
        )
        ready_lift_distance_m = config.get(
            "ready_lift_distance_m", ready_lift_distance_m
        )
        ready_state_timeout_sec = config.get(
            "ready_state_timeout_sec", ready_state_timeout_sec
        )
        ready_tf_timeout_sec = config.get(
            "ready_tf_timeout_sec", ready_tf_timeout_sec
        )
        ready_deployed_timeout_sec = config.get(
            "ready_deployed_timeout_sec", ready_deployed_timeout_sec
        )
        stow_state_timeout_sec = config.get(
            "stow_state_timeout_sec", stow_state_timeout_sec
        )
        force_stale_timeout_sec = config.get(
            "contact.force_stale_timeout_sec", force_stale_timeout_sec
        )
        contact_retreat_distance_m = config.get(
            "contact.retreat_distance_m", contact_retreat_distance_m
        )
        contact_retreat_speed_mps = config.get(
            "contact.retreat_speed_mps", contact_retreat_speed_mps
        )
        super().__init__(
            tf_listener,
            tag_state_source=tag_state_source,
            robot_name=robot_name,
            action_client=action_client,
            goal_response_timeout_sec=goal_response_timeout_sec,
            result_timeout_sec=result_timeout_sec,
            monotonic_clock=monotonic_clock,
            logger=logger,
        )
        self.speed_policy = (
            speed_policy
            if speed_policy is not None
            else ArmMotionSpeedPolicy.from_config(config)
        )
        self.arm_state_source = arm_state_source
        self.surface_source = surface_source
        self._owns_arm_stop_service_client = False
        self.arm_stop_service_client = arm_stop_service_client
        if (
            self.arm_stop_service_client is None
            and arm_state_source is not None
            and getattr(arm_state_source, "node", None) is not None
        ):
            self.arm_stop_service_client = arm_state_source.node.create_client(
                RobotCommandService,
                namespace_with(robot_name, "robot_command"),
            )
            self._owns_arm_stop_service_client = True
        self.force_contact_policy = force_contact_policy
        if contact_evidence_analyzer is not None:
            self.contact_evidence_analyzer = contact_evidence_analyzer
        else:
            self.contact_evidence_analyzer = ArmContactEvidenceAnalyzer(
                config=config
            )
        self.ready_forward_distance_m = self._ready_forward_distance(
            ready_forward_distance_m,
        )
        self.ready_lift_distance_m = self._positive_timeout(
            ready_lift_distance_m,
            "Ready arm lift distance",
        )
        self.ready_state_timeout_sec = self._positive_timeout(
            ready_state_timeout_sec,
            "Ready arm state timeout",
        )
        self.ready_tf_timeout_sec = self._positive_timeout(
            ready_tf_timeout_sec,
            "Ready arm TF timeout",
        )
        self.ready_deployed_timeout_sec = self._positive_timeout(
            ready_deployed_timeout_sec,
            "Ready arm deployed timeout",
        )
        self.stow_state_timeout_sec = self._positive_timeout(
            stow_state_timeout_sec,
            "Stow arm state timeout",
        )
        self.probe_motion_planner = ProbeMotionPlanner(
            tf_listener=tf_listener,
            speed_policy=self.speed_policy,
            build_pose_goal=self._build_pose_goal,
        )

        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None
        self._ready_probe_start = None
        self._guarded_plan_builder = None
        self._guarded_force_threshold_n = None
        self._arm_stop_service_future = None
        self._arm_stop_service_started = None

        self.guarded_probe_execution = None
        if (
            arm_state_source is not None
            and force_baseline_sampler is not None
            and force_contact_policy is not None
        ):
            self.guarded_probe_execution = GuardedProbeExecution(
                arm_state_source=arm_state_source,
                force_baseline_sampler=force_baseline_sampler,
                force_contact_policy=force_contact_policy,
                contact_evidence_analyzer=(
                    self.contact_evidence_analyzer
                ),
                start_goal=self._guard_start_goal,
                poll_goal=self._guard_poll_goal,
                cancel_goal=self._guard_cancel_goal,
                start_stop=self._guard_start_arm_stop,
                poll_stop=self._guard_poll_arm_stop,
                current_hand_pose=self.probe_motion_planner.current_hand_pose,
                build_motion_goal=self.probe_motion_planner.build_motion_goal,
                default_angular_speed_rad_s=(
                    self.speed_policy.default_speed.angular_speed_rad_s
                ),
                force_stale_timeout_sec=force_stale_timeout_sec,
                retreat_distance_m=contact_retreat_distance_m,
                retreat_speed_mps=contact_retreat_speed_mps,
                monotonic_clock=monotonic_clock,
            )

    def relative(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a relative hand target and execute it through the guard."""
        if self.probe_motion_planner.relative_command_is_noop(command):
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Skipped zero arm movement",
            )
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_relative(command),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def pose(
        self,
        target: PoseStamped,
        execution_frame: str = "",
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Execute an absolute bare-hand target through the guard."""
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_absolute(
                target,
                execution_frame,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def probe_pose(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Compatibility entry for guarded absolute probe motion."""
        return self.guarded_probe(
            probe_target,
            motion_sensor_id,
            speed,
            force_threshold_n=force_threshold_n,
        )

    def tag_probe(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a live tag target and execute it through the guard."""
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_tag(
                command,
                self.tag_state_source,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def probe_relative(
        self,
        offset: PoseStamped,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a probe-relative target and execute it through the guard."""
        if self.probe_motion_planner.pose_offset_is_noop(offset):
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Skipped zero arm movement",
            )
        return self.guarded_probe(
            lambda: self.probe_motion_planner.resolve_probe_relative(
                offset,
                motion_sensor_id,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def orient_to_surface(
        self,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Orient the active probe to the front-facing live surface."""
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Surface orientation requires active sensor geometry",
            )
        if self.surface_source is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Surface orientation source is not configured",
            )
        return self.guarded_probe(
            lambda: self._resolve_surface_orientation_target(sensor_id),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def orient_to_tag(
        self,
        tag_id: int,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Orient the active probe to a currently reachable tag."""
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Tag orientation requires active sensor geometry",
            )
        if self.tag_state_source is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Tag state source is not configured",
            )
        return self.guarded_probe(
            lambda: self._resolve_tag_orientation_target(
                int(tag_id),
                sensor_id,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )

    def guarded_probe(
        self,
        probe_target,
        motion_sensor_id: str = "",
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Execute one ready probe movement with force monitoring."""
        if self.active:
            return self._busy_update()
        if self.guarded_probe_execution is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement is not configured",
            )
        if self.force_contact_policy is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement requires a force contact policy",
            )

        if callable(probe_target):
            if str(motion_sensor_id).strip():
                raise ValueError(
                    "Guarded probe target builders must resolve their "
                    "own motion sensor ID"
                )
            target_builder = probe_target
        else:
            target = deepcopy(probe_target)
            sensor_id = str(motion_sensor_id).strip()
            target_builder = lambda: (target, sensor_id)

        self._guarded_force_threshold_n = force_threshold_n
        self._guarded_plan_builder = lambda: (
            self.probe_motion_planner.build_plan(
                target_builder,
                speed,
            )
        )
        return self.ready_probe(self._begin_guarded_probe)

    def ready_probe(
        self,
        start_probe,
    ) -> ArmMovementUpdate:
        """Ensure the arm is deployed, then start one probe operation."""
        if not callable(start_probe):
            raise TypeError("Ready probe requires a callable probe starter")
        if self.active:
            return self._busy_update()

        self._active = True
        self._operation = _ArmOperation.READY_WAIT
        self._ready_probe_start = start_probe
        return self._advance_ready_probe()

    def probe(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        """Start one low-level unguarded probe movement."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.MOVEMENT
        return self._submit_probe(
            probe_target,
            motion_sensor_id,
            speed,
        )

    def prepare(
        self,
        speed=None,
    ) -> ArmMovementUpdate:
        """Prepare a stowed arm with an unguarded controlled lift."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.PREPARE
        self._operation_speed = speed
        return self._advance_prepare_start()

    def stow(self) -> ArmMovementUpdate:
        """Stow a deployed arm through Spot's native stow command."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.STOW
        return self._advance_stow_start()

    def poll(self) -> ArmMovementUpdate:
        """Advance the active arm operation without blocking."""
        if not self.active or self._operation is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "No arm movement is active",
            )

        if self._verification_started is not None:
            return self._poll_state_confirmation()

        if self._operation == _ArmOperation.READY_WAIT:
            return self._advance_ready_probe()

        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            return self._poll_guarded_probe()

        if self._send_goal_future is None:
            if self._operation in (
                _ArmOperation.PREPARE,
                _ArmOperation.READY_PREPARE,
            ):
                return self._advance_prepare_start()
            if self._operation == _ArmOperation.STOW:
                return self._advance_stow_start()
            return super().poll()

        if self._goal_handle is None:
            return self._poll_goal_response()

        return self._poll_result()

    def cancel(self) -> None:
        if (
            self._operation == _ArmOperation.GUARDED_MOVEMENT
            and self.guarded_probe_execution is not None
        ):
            self.guarded_probe_execution.cancel()
        super().cancel()

    def shutdown(self) -> None:
        super().shutdown()
        if (
            self._owns_arm_stop_service_client
            and self.arm_stop_service_client is not None
        ):
            try:
                self.arm_stop_service_client.destroy()
            finally:
                self.arm_stop_service_client = None
                self._owns_arm_stop_service_client = False

    def _resolve_tag_orientation_target(
        self,
        tag_id: int,
        sensor_id: str,
    ):
        tag = self.tag_state_source.reachable_tag(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently reachable"
            )

        tag_pose = self.probe_motion_planner.normalize_target(
            tag.pose,
            GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        tag_orientation = pose_to_pose_data(tag_pose.pose).orientation
        target_orientation = multiply_quaternions(
            tag_orientation,
            tag_aligned_probe_orientation(),
        )

        current_probe = self.probe_motion_planner.current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )
        target_probe = deepcopy(current_probe)
        target_probe.pose.orientation.x = target_orientation.x
        target_probe.pose.orientation.y = target_orientation.y
        target_probe.pose.orientation.z = target_orientation.z
        target_probe.pose.orientation.w = target_orientation.w
        return target_probe, sensor_id

    def _resolve_surface_orientation_target(
        self,
        sensor_id: str,
    ):
        surface_normal = self.surface_source.surface_normal()
        projected = surface_normal.projected_point
        camera_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            projected.frame_id,
            timeout_sec=0.0,
        )
        surface_normal_execution = rotate_vector(
            transform_to_pose_data(camera_to_execution).orientation,
            surface_normal.normal_camera,
        )
        hand_to_probe_orientation = pose_to_pose_data(
            self.probe_motion_planner.hand_to_probe_pose(sensor_id)
        ).orientation
        target_orientation = surface_aligned_probe_orientation(
            surface_normal_execution,
            hand_to_probe_orientation,
            Vector3Data(x=0.0, y=0.0, z=1.0),
        )

        current_probe = self.probe_motion_planner.current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )
        target_probe = deepcopy(current_probe)
        target_probe.pose.orientation.x = target_orientation.x
        target_probe.pose.orientation.y = target_orientation.y
        target_probe.pose.orientation.z = target_orientation.z
        target_probe.pose.orientation.w = target_orientation.w
        return target_probe, sensor_id

    def _advance_ready_probe(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.ready_state_timeout_sec,
                "Waiting for manipulator stow state before probe movement",
            )

        self._state_wait_started = None
        if state is ArmStowState.STOWED:
            self._operation = _ArmOperation.READY_PREPARE
            self._operation_speed = None
            return self._advance_prepare_start()

        return self._begin_ready_probe()

    def _begin_ready_probe(self) -> ArmMovementUpdate:
        start_probe = self._ready_probe_start
        if start_probe is None:
            return super()._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Ready probe movement has no pending probe operation",
            )
        self._ready_probe_start = None
        try:
            return start_probe()
        except Exception as exception:
            return super()._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"Ready probe operation could not start: {exception}",
            )

    def _begin_guarded_probe(self) -> ArmMovementUpdate:
        self._operation = _ArmOperation.GUARDED_MOVEMENT
        builder = self._guarded_plan_builder
        if builder is None:
            return super()._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no pending target",
            )
        update = self.guarded_probe_execution.start(
            builder,
            force_threshold_n=self._guarded_force_threshold_n,
        )
        return self._finish_guarded_update(update)

    def _poll_guarded_probe(self) -> ArmMovementUpdate:
        update = self.guarded_probe_execution.poll()
        return self._finish_guarded_update(update)

    def _finish_guarded_update(
        self,
        update: ArmMovementUpdate,
    ) -> ArmMovementUpdate:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        if not self.active:
            return update
        return super()._finish(update.outcome, update.detail)

    def _finish(self, outcome, detail: str):
        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            # A goal ending must leave the guard active for stop confirmation.
            # Only _finish_guarded_update ends the enclosing arm operation.
            self._reset_goal_lifecycle()
            return ArmMovementUpdate(outcome, str(detail).strip())
        return super()._finish(outcome, detail)

    def _guard_start_goal(self, goal) -> ArmMovementUpdate:
        return self._submit_goal(lambda: goal)

    def _guard_poll_goal(self) -> ArmMovementUpdate:
        if self._send_goal_future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no active RobotCommand goal",
            )
        if self._goal_handle is None:
            return self._poll_goal_response()
        return self._poll_result()

    def _guard_cancel_goal(self) -> None:
        self._request_cancel()
        self._reset_goal_lifecycle()

    def _guard_start_arm_stop(self) -> ArmMovementUpdate:
        client = self.arm_stop_service_client
        if client is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "RobotCommand service client is unavailable",
            )

        try:
            if not client.wait_for_service(timeout_sec=0.0):
                return ArmMovementUpdate(
                    ArmMovementOutcome.ACTION_SERVER_UNAVAILABLE,
                    "RobotCommand service is unavailable",
                )
            request = self._build_arm_stop_request()
            future = client.call_async(request)
        except Exception as exception:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"ArmStopCommand service call failed: {exception}",
            )

        if future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "RobotCommand service returned no future for ArmStopCommand",
            )

        self._arm_stop_service_future = future
        self._arm_stop_service_started = self._monotonic_clock()
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "ArmStopCommand service request sent",
        )

    def _guard_poll_arm_stop(self) -> ArmMovementUpdate:
        future = self._arm_stop_service_future
        if future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "ArmStopCommand has no active service request",
            )

        if not future.done():
            if self._deadline_expired(
                self._arm_stop_service_started,
                self.goal_response_timeout_sec,
            ):
                self._reset_arm_stop_service_lifecycle(cancel=True)
                return ArmMovementUpdate(
                    ArmMovementOutcome.RESULT_TIMEOUT,
                    "ArmStopCommand service response timed out after "
                    f"{self.goal_response_timeout_sec:.1f} s",
                )
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for ArmStopCommand service response",
            )

        try:
            response = future.result()
        except Exception as exception:
            self._reset_arm_stop_service_lifecycle()
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"ArmStopCommand service response failed: {exception}",
            )

        self._reset_arm_stop_service_lifecycle()
        if response is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "ArmStopCommand service returned no response",
            )

        message = str(getattr(response, "message", "")).strip()
        if bool(getattr(response, "success", False)):
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                message or "ArmStopCommand accepted",
            )
        return ArmMovementUpdate(
            ArmMovementOutcome.MOTION_FAILED,
            message or "ArmStopCommand was rejected",
        )

    def _reset_arm_stop_service_lifecycle(self, cancel=False) -> None:
        future = self._arm_stop_service_future
        self._arm_stop_service_future = None
        self._arm_stop_service_started = None
        if cancel and future is not None and not future.done():
            try:
                future.cancel()
            except Exception:
                pass

    def _handle_successful_result(self, result):
        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            self._reset_goal_lifecycle()
            return ArmMovementUpdate(
                ArmMovementOutcome.SUCCESS,
                "Succeeded",
            )

        if self._operation in (
            _ArmOperation.PREPARE,
            _ArmOperation.READY_PREPARE,
            _ArmOperation.STOW,
        ):
            self._reset_goal_lifecycle()
            self._verification_started = self._monotonic_clock()
            return self._poll_state_confirmation()

        return super()._handle_successful_result(result)

    def _handle_failed_result(self, result):
        outcome, detail = self._arm_failure_result(result)
        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            self._reset_goal_lifecycle()
            return ArmMovementUpdate(outcome, detail)
        return self._finish(outcome, detail)

    def _arm_failure_result(self, result):
        feedback = self._cartesian_feedback(result)
        status = getattr(feedback, "status", None)
        value = getattr(status, "value", None)

        if self._status_matches(
            status,
            value,
            "STATUS_TRAJECTORY_STALLED",
        ):
            return (
                ArmMovementOutcome.TRAJECTORY_STALLED,
                self._cartesian_failure_detail(
                    "Cartesian arm trajectory stalled",
                    result,
                ),
            )

        if self._status_matches(
            status,
            value,
            "STATUS_TRAJECTORY_CANCELLED",
        ):
            return (
                ArmMovementOutcome.TRAJECTORY_CANCELLED,
                self._cartesian_failure_detail(
                    "Cartesian arm trajectory cancelled",
                    result,
                ),
            )

        return (
            ArmMovementOutcome.MOTION_FAILED,
            self._command_failure_detail(result),
        )

    @staticmethod
    def _cartesian_failure_detail(prefix: str, result) -> str:
        detail = str(
            getattr(result, "message", "")
            or getattr(result, "detail", "")
        ).strip()
        if not detail:
            return prefix
        return f"{prefix}; {detail}"

    @staticmethod
    def _status_matches(status, value, constant_name: str) -> bool:
        if status is None or value is None:
            return False
        expected = getattr(status, constant_name, None)
        return expected is not None and value == expected

    @staticmethod
    def _cartesian_feedback(result):
        command_feedback = getattr(result, "result", None)
        command = getattr(command_feedback, "command", None)
        synchronized = getattr(command, "synchronized_feedback", None)
        arm = getattr(synchronized, "arm_command_feedback", None)
        feedback = getattr(arm, "feedback", None)
        if feedback is None:
            return None

        cartesian_choice = getattr(
            feedback,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET",
            None,
        )
        feedback_choice = getattr(feedback, "feedback_choice", None)
        if (
            cartesian_choice is not None
            and feedback_choice != cartesian_choice
        ):
            return None

        return getattr(feedback, "arm_cartesian_feedback", None)

    def _reset_operation(self) -> None:
        super()._reset_operation()
        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None
        self._ready_probe_start = None
        self._guarded_plan_builder = None
        self._guarded_force_threshold_n = None
        self._reset_arm_stop_service_lifecycle(cancel=True)
        if self.guarded_probe_execution is not None:
            self.guarded_probe_execution.reset()

    def _advance_prepare_start(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.ready_state_timeout_sec,
                "Waiting for manipulator stow state",
            )

        self._state_wait_started = None
        if state is ArmStowState.DEPLOYED:
            if self._operation == _ArmOperation.READY_PREPARE:
                return self._begin_ready_probe()
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Arm is already deployed",
            )

        try:
            current_hand = self.probe_motion_planner.current_hand_pose()
        except Exception as exception:
            return self._wait_for_ready_transform(exception)

        self._tf_wait_started = None
        target_hand = deepcopy(current_hand)
        target_hand.pose.position.x += self.ready_forward_distance_m
        target_hand.pose.position.z += self.ready_lift_distance_m

        return self._submit_probe(
            target_hand,
            BARE_HAND_MOTION_ID,
            self._operation_speed,
        )

    def _advance_stow_start(self) -> ArmMovementUpdate:
        state = self._fresh_arm_state()
        if state is None or state is ArmStowState.UNKNOWN:
            return self._wait_for_arm_state(
                self.stow_state_timeout_sec,
                "Waiting for manipulator stow state",
            )

        self._state_wait_started = None
        if state is ArmStowState.STOWED:
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                "Arm is already stowed",
            )

        return self._submit_goal(self._build_stow_goal)

    def _poll_state_confirmation(self) -> ArmMovementUpdate:
        if self._operation in (
            _ArmOperation.PREPARE,
            _ArmOperation.READY_PREPARE,
        ):
            expected = ArmStowState.DEPLOYED
            timeout_sec = self.ready_deployed_timeout_sec
            success_detail = "Arm deployed"
            failure_detail = (
                "Ready arm movement completed, but Spot did not report "
                f"DEPLOYED within {timeout_sec:.1f} s"
            )
        elif self._operation == _ArmOperation.STOW:
            expected = ArmStowState.STOWED
            timeout_sec = self.stow_state_timeout_sec
            success_detail = "Arm stowed"
            failure_detail = (
                "Stow arm movement completed, but Spot did not report "
                f"STOWED within {timeout_sec:.1f} s"
            )
        else:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Arm state verification has no matching operation",
            )

        state = self._fresh_arm_state()
        if state is expected:
            if self._operation == _ArmOperation.READY_PREPARE:
                self._verification_started = None
                return self._begin_ready_probe()
            return self._finish(
                ArmMovementOutcome.SUCCESS,
                success_detail,
            )

        if not self._deadline_expired(
            self._verification_started,
            timeout_sec,
        ):
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                f"Waiting for Spot to report arm {expected.value}",
            )

        outcome = self._arm_state_failure_outcome(state)
        if (
            outcome is ArmMovementOutcome.ARM_STATE_UNKNOWN
            and state is not ArmStowState.UNKNOWN
        ):
            outcome = ArmMovementOutcome.MOTION_FAILED
        return self._finish(outcome, failure_detail)

    def _submit_probe(
        self,
        probe_target: PoseStamped,
        motion_sensor_id: str,
        speed=None,
    ) -> ArmMovementUpdate:
        return self._submit_goal(
            lambda: self.probe_motion_planner.build_probe_goal(
                probe_target,
                motion_sensor_id,
                speed,
            )
        )

    def _wait_for_arm_state(
        self,
        timeout_sec: float,
        detail: str,
    ) -> ArmMovementUpdate:
        now = self._monotonic_clock()
        if self._state_wait_started is None:
            self._state_wait_started = now

        if now - self._state_wait_started < timeout_sec:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                detail,
            )

        state = self._fresh_arm_state()
        outcome = self._arm_state_failure_outcome(state)
        return self._finish(
            outcome,
            "Fresh manipulator stow state was unavailable for "
            f"{timeout_sec:.1f} s",
        )

    def _wait_for_ready_transform(
        self,
        exception: Exception,
    ) -> ArmMovementUpdate:
        now = self._monotonic_clock()
        if self._tf_wait_started is None:
            self._tf_wait_started = now

        if now - self._tf_wait_started < self.ready_tf_timeout_sec:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for ready-arm hand pose transform "
                f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME}",
            )

        return self._finish(
            ArmMovementOutcome.EXECUTION_ERROR,
            "Ready arm hand transform "
            f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME} "
            f"was unavailable for {self.ready_tf_timeout_sec:.1f} s: "
            f"{exception}",
        )

    def _fresh_arm_state(self):
        if self.arm_state_source is None:
            return None
        return self.arm_state_source.stow_state()

    def _arm_state_failure_outcome(self, state):
        source = self.arm_state_source
        if source is None:
            return ArmMovementOutcome.ARM_STATE_UNAVAILABLE
        if getattr(source, "last_received_at", None) is None:
            return ArmMovementOutcome.ARM_STATE_UNAVAILABLE
        if source.is_stale():
            return ArmMovementOutcome.ARM_STATE_STALE
        if state is ArmStowState.UNKNOWN:
            return ArmMovementOutcome.ARM_STATE_UNKNOWN
        return ArmMovementOutcome.ARM_STATE_UNKNOWN

    def _build_arm_stop_request(self) -> RobotCommandService.Request:
        arm_stop = arm_command_pb2.ArmStopCommand.Request()
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_stop_command=arm_stop
        )
        synchronized = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        command = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized
        )
        request = RobotCommandService.Request()
        convert(command, request.command)
        return request

    @staticmethod
    def _ready_forward_distance(value) -> float:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                "Ready arm forward distance must be non-negative and finite"
            )
        return value

    def _build_stow_goal(self) -> RobotCommand.Goal:
        stow_command = RobotCommandBuilder.arm_stow_command()
        goal = RobotCommand.Goal()
        convert(stow_command, goal.command)
        return goal

    def _build_pose_goal(
        self,
        target: PoseStamped,
        duration_sec: float,
    ) -> RobotCommand.Goal:
        duration = float(duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError(
                "Arm movement duration must be positive and finite"
            )

        target_frame = target.header.frame_id.strip()
        if not target_frame:
            raise ValueError("Arm pose target frame must not be empty")

        pose = target.pose
        command = RobotCommandBuilder.arm_pose_command(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            namespace_with(self.robot_name, target_frame),
            duration,
        )
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        return goal


__all__ = [
    "ArmMovementExecutor",
    "ArmMovementOutcome",
    "ArmMovementUpdate",
]
