"""Centralized Spot arm movement execution."""

from copy import deepcopy
import math
from threading import RLock
import time

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
    ODOM_FRAME_NAME,
)
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import TransformException
from spot_msgs.action import RobotCommand
from spot_msgs.srv import RobotCommand as RobotCommandService
from synchros2.utilities import namespace_with
from fault_detector_spot.shared.geometry.rotation import (
    multiply_quaternions,
    rotate_vector,
)
from fault_detector_spot.shared.geometry.models import (
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.geometry.alignment_orientation import (
    surface_aligned_probe_orientation,
    tag_aligned_probe_orientation,
)
from fault_detector_spot.manipulation.arm_command_builder import (
    build_arm_stop_request,
    build_stow_goal,
    build_gripper_goal,
    build_moveit_joint_goal,
    build_pose_goal,
)
from fault_detector_spot.manipulation.arm_command_feedback import arm_failure_result
from fault_detector_spot.manipulation.surface_orientation_target import SurfaceOrientationTarget

from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
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
from fault_detector_spot.manipulation.guarded_probe_monitor import (
    GuardedProbeMonitor,
)
from fault_detector_spot.manipulation.moveit_arm_planner import (
    MoveItPlanOutcome,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    CartesianMotionPlan,
    ProbeMotionPlanner,
    ResolvedProbeTarget,
)
from fault_detector_spot.shared.geometry.movement_geometry import (
    MovementGeometryUnavailable,
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


TAG_POSITION_VERIFY_TIMEOUT_SEC = 2.0
CHECKPOINT_POSITION_TOLERANCE_M = 0.020


SURFACE_ORIENTATION_MAX_ERROR_RAD = math.radians(5.0)
SURFACE_ORIENTATION_TIMEOUT_SEC = 2.0
SURFACE_ORIENTATION_SENSING_TIMEOUT_SEC = 6.0
SURFACE_ORIENTATION_MAX_CORRECTIONS = 1


class _ArmOperation:
    MOVEMENT = "movement"
    READY_WAIT = "ready_wait"
    READY_PREPARE = "ready_prepare"
    GUARDED_MOVEMENT = "guarded_movement"
    TAG_POSITION_VERIFY = "tag_position_verify"
    SURFACE_ORIENTATION_VERIFY = "surface_orientation_verify"
    PREPARE = "prepare"
    STOW = "stow"
    GRIPPER = "gripper"
    CONFIRM_STOP = "confirm_stop"


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
        moveit_arm_planner=None,
        arm_stop_service_client=None,
        arm_state_source=None,
        surface_source=None,
        force_baseline_sampler=None,
        force_contact_policy=None,
        contact_evidence_analyzer=None,
        force_stale_timeout_sec=None,
        hard_force_delta_limit_n=None,
        stop_confirmation_linear_velocity_threshold_mps=None,
        stop_confirmation_angular_velocity_threshold_rad_s=None,
        stop_confirmation_stable_duration_sec=None,
        stop_confirmation_timeout_sec=None,
        contact_retreat_distance_m=None,
        contact_retreat_speed_mps=None,
        moveit_result_timeout_margin_sec=None,
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
        self._execution_lock = RLock()
        config = config if config is not None else ArmMotionParameters(
            getattr(arm_state_source, "node", None)
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
        self._configure_motion_speeds(
            config=config,
            speed_policy=speed_policy,
            moveit_result_timeout_margin_sec=moveit_result_timeout_margin_sec,
        )
        self.arm_state_source = arm_state_source
        self.surface_source = surface_source
        self.moveit_arm_planner = moveit_arm_planner
        self._configure_arm_stop_client(
            arm_stop_service_client=arm_stop_service_client,
            arm_state_source=arm_state_source,
            robot_name=robot_name,
        )
        self.force_contact_policy = force_contact_policy
        if contact_evidence_analyzer is not None:
            self.contact_evidence_analyzer = contact_evidence_analyzer
        else:
            self.contact_evidence_analyzer = ArmContactEvidenceAnalyzer(
                config=config
            )
        self._configure_readiness(
            config=config,
            ready_forward_distance_m=ready_forward_distance_m,
            ready_lift_distance_m=ready_lift_distance_m,
            ready_state_timeout_sec=ready_state_timeout_sec,
            ready_tf_timeout_sec=ready_tf_timeout_sec,
            ready_deployed_timeout_sec=ready_deployed_timeout_sec,
            stow_state_timeout_sec=stow_state_timeout_sec,
        )
        self.probe_motion_planner = ProbeMotionPlanner(
            tf_listener=tf_listener,
            speed_policy=self.speed_policy,
        )

        self._probe_continuation = object()
        self._clear_arm_operation_state()
        self._pending_moveit_plan_builder = None
        self._pending_moveit_cartesian_path = False
        self._moveit_cartesian_plan = None
        self._clear_surface_orientation_state()
        self._arm_stop_service_future = None
        self._arm_stop_service_started = None

        self._configure_contact_guard(
            config=config,
            arm_state_source=arm_state_source,
            force_baseline_sampler=force_baseline_sampler,
            force_contact_policy=force_contact_policy,
            force_stale_timeout_sec=force_stale_timeout_sec,
            hard_force_delta_limit_n=hard_force_delta_limit_n,
            stop_confirmation_linear_velocity_threshold_mps=(
                stop_confirmation_linear_velocity_threshold_mps
            ),
            stop_confirmation_angular_velocity_threshold_rad_s=(
                stop_confirmation_angular_velocity_threshold_rad_s
            ),
            stop_confirmation_stable_duration_sec=stop_confirmation_stable_duration_sec,
            stop_confirmation_timeout_sec=stop_confirmation_timeout_sec,
            contact_retreat_distance_m=contact_retreat_distance_m,
            contact_retreat_speed_mps=contact_retreat_speed_mps,
            monotonic_clock=monotonic_clock,
        )

    def _configure_motion_speeds(
        self,
        config,
        speed_policy,
        moveit_result_timeout_margin_sec,
    ):
        self._base_result_timeout_sec = self.result_timeout_sec
        self.moveit_result_timeout_margin_sec = self._positive_timeout(
            config.get(
                'motion.moveit_result_timeout_margin_sec',
                moveit_result_timeout_margin_sec,
            ),
            'MoveIt result timeout margin',
        )
        self.speed_policy = (
            speed_policy if speed_policy is not None
            else ArmMotionSpeedPolicy.from_config(config)
        )
        self.ready_speed = ArmMotionSpeed(
            linear_speed_mps=config.get('ready_linear_speed_mps'),
            angular_speed_rad_s=self.speed_policy.default_speed.angular_speed_rad_s,
        )
        self.safe_approach_speed = ArmMotionSpeed(
            linear_speed_mps=config.get('safe_approach_linear_speed_mps'),
            angular_speed_rad_s=config.get('safe_approach_angular_speed_rad_s'),
        )

    def _configure_arm_stop_client(
        self,
        arm_stop_service_client,
        arm_state_source,
        robot_name,
    ):
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

    def _configure_readiness(
        self,
        config,
        ready_forward_distance_m,
        ready_lift_distance_m,
        ready_state_timeout_sec,
        ready_tf_timeout_sec,
        ready_deployed_timeout_sec,
        stow_state_timeout_sec,
    ):
        self.ready_forward_distance_m = self._ready_forward_distance(
            config.get('ready_forward_distance_m', ready_forward_distance_m),
        )
        self.ready_lift_distance_m = self._positive_timeout(
            config.get('ready_lift_distance_m', ready_lift_distance_m),
            'Ready arm lift distance',
        )
        self.ready_state_timeout_sec = self._positive_timeout(
            config.get('ready_state_timeout_sec', ready_state_timeout_sec),
            'Ready arm state timeout',
        )
        self.ready_tf_timeout_sec = self._positive_timeout(
            config.get('ready_tf_timeout_sec', ready_tf_timeout_sec),
            'Ready arm TF timeout',
        )
        self.ready_deployed_timeout_sec = self._positive_timeout(
            config.get('ready_deployed_timeout_sec', ready_deployed_timeout_sec),
            'Ready arm deployed timeout',
        )
        self.stow_state_timeout_sec = self._positive_timeout(
            config.get('stow_state_timeout_sec', stow_state_timeout_sec),
            'Stow arm state timeout',
        )

    def _configure_contact_guard(
        self,
        config,
        arm_state_source,
        force_baseline_sampler,
        force_contact_policy,
        force_stale_timeout_sec,
        hard_force_delta_limit_n,
        stop_confirmation_linear_velocity_threshold_mps,
        stop_confirmation_angular_velocity_threshold_rad_s,
        stop_confirmation_stable_duration_sec,
        stop_confirmation_timeout_sec,
        contact_retreat_distance_m,
        contact_retreat_speed_mps,
        monotonic_clock,
    ):
        guard_settings = {
            "force_stale_timeout_sec": config.get(
                'contact.force_stale_timeout_sec',
                force_stale_timeout_sec,
            ),
            "hard_force_delta_limit_n": config.get(
                'contact.hard_force_delta_limit_n',
                hard_force_delta_limit_n,
            ),
            "stop_confirmation_linear_velocity_threshold_mps": config.get(
                'contact.stop_confirmation.linear_velocity_threshold_mps',
                stop_confirmation_linear_velocity_threshold_mps,
            ),
            "stop_confirmation_angular_velocity_threshold_rad_s": config.get(
                'contact.stop_confirmation.angular_velocity_threshold_rad_s',
                stop_confirmation_angular_velocity_threshold_rad_s,
            ),
            "stop_confirmation_stable_duration_sec": config.get(
                'contact.stop_confirmation.stable_duration_sec',
                stop_confirmation_stable_duration_sec,
            ),
            "stop_confirmation_timeout_sec": config.get(
                'contact.stop_confirmation.timeout_sec',
                stop_confirmation_timeout_sec,
            ),
            "contact_retreat_distance_m": config.get(
                'contact.retreat_distance_m',
                contact_retreat_distance_m,
            ),
            "contact_retreat_speed_mps": config.get(
                'contact.retreat_speed_mps',
                contact_retreat_speed_mps,
            ),
        }
        self.guarded_probe_execution = None
        self._guarded_probe_monitor = None
        if (
            arm_state_source is not None
            and force_baseline_sampler is not None
            and force_contact_policy is not None
        ):
            self.guarded_probe_execution = GuardedProbeExecution(
                arm_state_source=arm_state_source,
                force_baseline_sampler=force_baseline_sampler,
                force_contact_policy=force_contact_policy,
                contact_evidence_analyzer=self.contact_evidence_analyzer,
                start_motion=self._continue_probe,
                poll_goal=self._guard_poll_goal,
                cancel_goal=self._guard_cancel_goal,
                start_stop=self._guard_start_arm_stop,
                poll_stop=self._guard_poll_arm_stop,
                current_hand_pose=self.probe_motion_planner.current_hand_pose,
                build_motion_plan=self.probe_motion_planner.build_motion_plan,
                default_angular_speed_rad_s=(
                    self.speed_policy.default_speed.angular_speed_rad_s
                ),
                force_stale_timeout_sec=guard_settings["force_stale_timeout_sec"],
                hard_force_delta_limit_n=guard_settings["hard_force_delta_limit_n"],
                stop_confirmation_linear_velocity_threshold_mps=(
                    guard_settings["stop_confirmation_linear_velocity_threshold_mps"]
                ),
                stop_confirmation_angular_velocity_threshold_rad_s=(
                    guard_settings["stop_confirmation_angular_velocity_threshold_rad_s"]
                ),
                stop_confirmation_stable_duration_sec=(
                    guard_settings["stop_confirmation_stable_duration_sec"]
                ),
                stop_confirmation_timeout_sec=(
                    guard_settings["stop_confirmation_timeout_sec"]
                ),
                retreat_distance_m=guard_settings["contact_retreat_distance_m"],
                retreat_speed_mps=guard_settings["contact_retreat_speed_mps"],
                monotonic_clock=monotonic_clock,
                execution_lock=self._execution_lock,
            )
            node = getattr(arm_state_source, 'node', None)
            if node is not None:
                self._guarded_probe_monitor = GuardedProbeMonitor(
                    node,
                    arm_state_source,
                    self.guarded_probe_execution,
                )

    def relative(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Resolve a relative hand target and execute it through the guard."""
        scale = float(getattr(command, "arm_speed_scale", 1.0))
        if not math.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError("Arm speed scale must be in (0, 1]")
        if scale != 1.0:
            baseline = speed or self.speed_policy.default_speed
            speed = ArmMotionSpeed(
                baseline.linear_speed_mps * scale,
                baseline.angular_speed_rad_s * scale,
            )
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

    def capture_probe_checkpoint(self, sensor_id: str) -> ResolvedProbeTarget:
        """Snapshot an achieved probe pose in odom for path backtracking."""
        with self._execution_lock:
            if self.active:
                raise RuntimeError("Cannot capture a checkpoint while arm movement is active")
            if not str(sensor_id).strip():
                raise ValueError("Probe checkpoint requires attachment geometry")
            pose = self.probe_motion_planner.current_pose(
                ODOM_FRAME_NAME, sensor_probe_frame(sensor_id),
            )
            pose_to_pose_data(pose.pose).validate()
            return ResolvedProbeTarget(deepcopy(pose), sensor_id)

    def restore_probe_checkpoint(self, checkpoint: ResolvedProbeTarget) -> ArmMovementUpdate:
        """Guard and verify a return to a previously reached probe pose."""
        if not isinstance(checkpoint, ResolvedProbeTarget):
            raise TypeError("Expected a resolved probe checkpoint")
        if checkpoint.target.header.frame_id != ODOM_FRAME_NAME:
            raise ValueError("Probe checkpoint must be expressed in odom")
        pose_to_pose_data(checkpoint.target.pose).validate()
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            target = ResolvedProbeTarget(deepcopy(checkpoint.target), checkpoint.sensor_id)
            self._tag_accuracy = {
                "tolerance": CHECKPOINT_POSITION_TOLERANCE_M,
                "orientation_tolerance": math.radians(5.0),
                "verify_checkpoint": True,
                "target": target, "corrections": 0,
                "speed": self.safe_approach_speed, "force_threshold": None,
            }
            update = self.guarded_probe(lambda: target, speed=self.safe_approach_speed)
            if update.outcome is not ArmMovementOutcome.RUNNING:
                self._tag_accuracy = None
            return update

    def tag_probe(
        self,
        command,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Move to a tag, then verify and correct the achieved tip position."""
        scale = float(getattr(command, "arm_speed_scale", 1.0))
        if not math.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError("Arm speed scale must be in (0, 1]")
        baseline = speed or self.speed_policy.default_speed
        speed = ArmMotionSpeed(baseline.linear_speed_mps * scale,
                               baseline.angular_speed_rad_s * scale)
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            tolerance = float(command.tag_position_tolerance_m)
            if not math.isfinite(tolerance) or tolerance <= 0:
                return ArmMovementUpdate(
                    ArmMovementOutcome.EXECUTION_ERROR,
                    "Tag position tolerance must be positive and finite",
                )
            self._tag_accuracy = {
                "tolerance": tolerance, "target": None, "corrections": 0,
                "speed": speed or self.speed_policy.default_speed,
                "force_threshold": force_threshold_n,
            }
            update = self.guarded_probe(
                lambda: self._resolve_verified_tag_target(command),
                speed=speed,
                force_threshold_n=force_threshold_n,
            )
            if update.outcome is not ArmMovementOutcome.RUNNING:
                self._tag_accuracy = None
            return update

    def _resolve_verified_tag_target(self, command):
        resolved = self.probe_motion_planner.resolve_tag(command, self.tag_state_source)
        # Retain the ordinary resolved target without additional TF lookups or
        # changes to planning. Accuracy work starts only after motion succeeds.
        self._tag_accuracy["target"] = deepcopy(resolved)
        return resolved

    @staticmethod
    def _pose_stamp_ns(pose):
        return pose.header.stamp.sec * 1000000000 + pose.header.stamp.nanosec

    def _tag_current_pose(self):
        target = self._tag_accuracy["target"]
        return self.probe_motion_planner.current_pose(
            target.target.header.frame_id, sensor_probe_frame(target.sensor_id)
        )

    def _begin_tag_position_verification(self):
        self._operation = _ArmOperation.TAG_POSITION_VERIFY
        state = self._tag_accuracy
        state["deadline"] = self._monotonic_clock() + TAG_POSITION_VERIFY_TIMEOUT_SEC
        state["stamp"] = None
        try:
            state["stamp"] = self._pose_stamp_ns(self._tag_current_pose())
        except (ValueError, RuntimeError, TransformException):
            pass
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING, "Move-to-tag completed; waiting for fresh position feedback"
        )

    def _poll_tag_position_verification(self):
        state = self._tag_accuracy
        if self._monotonic_clock() >= state["deadline"]:
            return super()._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Move-to-tag accuracy check timed out waiting for fresh position feedback",
            )
        try:
            current = self._tag_current_pose()
            stamp = self._pose_stamp_ns(current)
            if state["stamp"] is None:
                state["stamp"] = stamp
            if stamp <= state["stamp"]:
                return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "Waiting for fresh position feedback")
            error = self.speed_policy._translation_distance(
                current.pose, state["target"].target.pose
            )
        except (ValueError, RuntimeError, TransformException) as exception:
            return ArmMovementUpdate(ArmMovementOutcome.RUNNING, f"Waiting for position feedback: {exception}")
        orientation_tolerance = state.get("orientation_tolerance")
        orientation_error = self.speed_policy._rotation_angle(
            current.pose, state["target"].target.pose,
        ) if orientation_tolerance is not None else 0.0
        orientation_ok = orientation_tolerance is None or orientation_error <= orientation_tolerance
        if error <= state["tolerance"] and orientation_ok:
            return super()._finish(
                ArmMovementOutcome.SUCCESS,
                f"Move-to-tag position verified: {error:.4f} m error "
                f"(tolerance {state['tolerance']:.4f} m)",
            )
        if state.get("verify_checkpoint") and state["corrections"]:
            return super()._finish(
                ArmMovementOutcome.CHECKPOINT_TOLERANCE_FAILED,
                "Checkpoint was not reached within position/orientation tolerance: "
                f"position {error:.4f} m (limit {state['tolerance']:.4f} m), "
                f"orientation {math.degrees(orientation_error):.2f} deg "
                f"(limit {math.degrees(orientation_tolerance):.2f} deg)",
            )
        state["corrections"] += 1
        baseline = state["speed"]
        speed = ArmMotionSpeed(baseline.linear_speed_mps * .5, baseline.angular_speed_rad_s * .5)
        self._guarded_force_threshold_n = state["force_threshold"]
        self._guarded_cartesian_path = False
        self._guarded_plan_builder = lambda: self.probe_motion_planner.build_plan(
            lambda: state["target"], speed
        )
        return self._begin_guarded_probe()

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
        """Orient the active probe and verify the achieved surface alignment."""
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
        if self.active:
            return self._busy_update()

        started_at = self._monotonic_clock()
        self._surface_orientation_sensor_id = sensor_id
        self._surface_orientation_speed = speed
        self._surface_orientation_force_threshold_n = force_threshold_n
        self._surface_orientation_corrections = 0
        update = self.guarded_probe(
            self._surface_orientation_target_builder(
                sensor_id,
                receipt_not_before=started_at,
            ),
            speed=speed,
            force_threshold_n=force_threshold_n,
        )
        if (
            isinstance(update, ArmMovementUpdate)
            and update.outcome is not ArmMovementOutcome.RUNNING
            and not self.active
        ):
            self._clear_surface_orientation_state()
        return update

    def _surface_orientation_target_builder(
        self,
        sensor_id: str,
        receipt_not_before: float,
    ):
        return SurfaceOrientationTarget(
            surface_source=self.surface_source,
            resolve_target=self._resolve_surface_orientation_target,
            sensor_id=sensor_id,
            receipt_not_before=receipt_not_before,
            clock=self._monotonic_clock,
            sensing_timeout_sec=SURFACE_ORIENTATION_SENSING_TIMEOUT_SEC,
            tf_timeout_sec=SURFACE_ORIENTATION_TIMEOUT_SEC,
        )

    def orient_to_tag(
        self,
        tag_id: int,
        motion_sensor_id: str,
        speed=None,
        force_threshold_n=None,
    ) -> ArmMovementUpdate:
        """Orient the active probe to a currently usable tag."""
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
        cartesian_path: bool = False,
        retreat_distance_m=None,
    ) -> ArmMovementUpdate:
        """Execute a force-guarded probe movement.

        retreat_distance_m overrides contact backoff for this movement only;
        None uses the configured arm.contact.retreat_distance_m default.
        """
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            error = self._guarded_probe_precondition_error(cartesian_path)
            if error is not None:
                return error
            target_builder = self._probe_target_builder(probe_target, motion_sensor_id)

            self._guarded_retreat_distance_m = retreat_distance_m
            self._guarded_force_threshold_n = force_threshold_n
            self._guarded_cartesian_path = bool(cartesian_path)
            self._guarded_plan_builder = lambda: (
                self.probe_motion_planner.build_plan(
                    target_builder,
                    speed,
                )
            )
            return self.ready_probe(self._begin_guarded_probe)

    def _guarded_probe_precondition_error(self, cartesian_path):
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
        if cartesian_path and self.moveit_arm_planner is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Cartesian guarded probe movement requires MoveIt planning",
            )

        return None

    @staticmethod
    def _probe_target_builder(probe_target, motion_sensor_id):
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

        return target_builder

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
        probe_target: PoseStamped | CartesianMotionPlan,
        motion_sensor_id: str = "",
        speed=None,
        *,
        _continuation=None,
        _cartesian_path: bool = False,
    ) -> ArmMovementUpdate:
        """Execute every normal arm motion through this unguarded boundary.

        Accept either a probe target or an already resolved hand motion plan.
        Internal continuations retain the enclosing readiness/guard lifecycle;
        public calls still require an idle executor. Trajectory planning and
        Spot command translation belong at this boundary.
        """
        continuing = _continuation is self._probe_continuation
        if self.active:
            if not continuing or any((
                self._send_goal_future is not None,
                self._pending_goal_builder is not None,
                self._verification_started is not None,
            )):
                return self._busy_update()
        elif continuing:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Probe continuation has no active arm operation",
            )
        else:
            self._active = True
            self._operation = _ArmOperation.MOVEMENT

        target = deepcopy(probe_target)

        def build_plan():
            if isinstance(target, CartesianMotionPlan):
                if str(motion_sensor_id).strip() or speed is not None:
                    raise ValueError(
                        "Resolved arm plans already contain target geometry "
                        "and timing"
                    )
                return target
            return self.probe_motion_planner.build_probe_plan(
                target,
                motion_sensor_id,
                speed,
            )

        if self._moveit_planning_required():
            self._pending_moveit_plan_builder = build_plan
            self._pending_moveit_cartesian_path = bool(_cartesian_path)
            return self._advance_moveit_planning_start()

        def build_goal():
            plan = build_plan()
            return self._build_pose_goal(plan.target_hand, plan.duration_sec)

        self._pending_goal_builder = build_goal
        return self._submit_goal(build_goal)

    def _moveit_planning_required(self) -> bool:
        return (
            self.moveit_arm_planner is not None
            and self._operation not in (
                _ArmOperation.PREPARE,
                _ArmOperation.READY_PREPARE,
            )
        )

    def _advance_moveit_planning_start(self) -> ArmMovementUpdate:
        planner = self.moveit_arm_planner
        builder = self._pending_moveit_plan_builder
        if planner is None or builder is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "MoveIt planning has no planner or pending arm plan",
            )

        try:
            plan = builder()
            target_hand = self.probe_motion_planner.normalize_target(
                plan.target_hand,
                planner.planning_frame,
            )
        except MovementGeometryUnavailable as exception:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                str(exception),
            )
        except Exception as exception:
            self._pending_moveit_plan_builder = None
            self._pending_moveit_cartesian_path = False
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                f"MoveIt target preparation failed: {exception}",
            )

        if self._pending_moveit_cartesian_path:
            update = planner.start_cartesian(target_hand)
        else:
            update = planner.start(target_hand)
        return self._handle_moveit_planning_start(update, plan)

    def _handle_moveit_planning_start(self, update, plan):
        self._pending_moveit_plan_builder = None
        self._pending_moveit_cartesian_path = False
        if update.outcome is MoveItPlanOutcome.RUNNING:
            self._moveit_cartesian_plan = plan
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                update.detail,
            )

        return self._finish(
            self._moveit_failure_outcome(update.outcome),
            update.detail,
        )

    def _poll_moveit_planning(self) -> ArmMovementUpdate:
        planner = self.moveit_arm_planner
        plan = self._moveit_cartesian_plan
        if planner is None or plan is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "MoveIt planning lost its planner or Cartesian arm plan",
            )

        update = planner.poll()
        if update.outcome is MoveItPlanOutcome.RUNNING:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                update.detail,
            )

        self._moveit_cartesian_plan = None
        if update.outcome is not MoveItPlanOutcome.SUCCESS:
            return self._finish(
                self._moveit_failure_outcome(update.outcome),
                update.detail,
            )

        trajectory = update.trajectory
        if trajectory is None:
            return self._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "MoveIt reported success without an arm trajectory",
            )

        def build_goal():
            return self._build_moveit_joint_goal(
                trajectory,
                minimum_duration_sec=plan.duration_sec,
            )

        self._pending_goal_builder = build_goal
        return self._submit_goal(build_goal)

    @staticmethod
    def _moveit_failure_outcome(_outcome):
        return ArmMovementOutcome.PLANNING_FAILED

    def _reset_moveit_planning(self, cancel=False) -> None:
        planner = self.moveit_arm_planner
        self._pending_moveit_plan_builder = None
        self._pending_moveit_cartesian_path = False
        self._moveit_cartesian_plan = None
        if cancel and planner is not None:
            planner.cancel()

    def _continue_probe(
        self,
        probe_target: PoseStamped | CartesianMotionPlan,
        motion_sensor_id: str = "",
        speed=None,
    ) -> ArmMovementUpdate:
        """Start a physical step without replacing its enclosing operation."""
        cartesian_path = self._next_probe_cartesian_path
        self._next_probe_cartesian_path = False
        return self.probe(
            probe_target,
            motion_sensor_id,
            speed,
            _continuation=self._probe_continuation,
            _cartesian_path=cartesian_path,
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
        self._operation_speed = (
            speed if speed is not None else self.ready_speed
        )
        return self._advance_prepare_start()

    def toggle_gripper(self) -> ArmMovementUpdate:
        """Toggle toward the opposite endpoint of the measured opening."""
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            percentage = (
                self.arm_state_source.gripper_open_percentage()
                if self.arm_state_source is not None else None
            )
            if percentage is None:
                return ArmMovementUpdate(
                    ArmMovementOutcome.ARM_STATE_UNAVAILABLE,
                    "Fresh gripper opening feedback is required to toggle",
                )
            # The midpoint selects the opposite endpoint for a partial opening.
            return self._start_gripper(0.0 if percentage > 50.0 else 1.0)

    def close_gripper(self) -> ArmMovementUpdate:
        """Request closing explicitly without guessing the current position."""
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            return self._start_gripper(0.0)

    def _start_gripper(self, open_fraction):
        self._operation = _ArmOperation.GRIPPER
        return self._start_goal(lambda: build_gripper_goal(open_fraction))

    def stow(self) -> ArmMovementUpdate:
        """Stow a deployed arm through Spot's native stow command."""
        if self.active:
            return self._busy_update()
        self._active = True
        self._operation = _ArmOperation.STOW
        return self._advance_stow_start()

    def confirm_stop(self) -> ArmMovementUpdate:
        """Issue a stop and require fresh stationary feedback before recovery."""
        with self._execution_lock:
            if self.active:
                return self._busy_update()
            if self.guarded_probe_execution is None:
                return ArmMovementUpdate(ArmMovementOutcome.STOP_UNCONFIRMED,
                                         "Stop confirmation monitor is unavailable")
            self._active = True
            self._operation = _ArmOperation.CONFIRM_STOP
            self.guarded_probe_execution.reset()
            if self._guarded_probe_monitor is not None:
                self._guarded_probe_monitor.cancel(None)
                update = self._guarded_probe_monitor.poll()
            else:
                update = self.guarded_probe_execution.cancel()
            return self._finish_stop_confirmation(update)

    def _finish_stop_confirmation(self, update):
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        outcome = (ArmMovementOutcome.SUCCESS
                   if update.outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
                   else ArmMovementOutcome.STOP_UNCONFIRMED)
        return super()._finish(outcome, update.detail)

    def poll(self) -> ArmMovementUpdate:
        """Advance the active arm operation without blocking."""
        with self._execution_lock:
            if self.cancelling:
                if self._guarded_probe_monitor is None and self.guarded_probe_execution is not None:
                    self._cancel_stop_finished(self.guarded_probe_execution.poll())
                return ArmMovementUpdate(ArmMovementOutcome.RUNNING, self._cancellation_detail)
            if not self.active or self._operation is None:
                return ArmMovementUpdate(
                    ArmMovementOutcome.EXECUTION_ERROR,
                    "No arm movement is active",
                )

            if self._operation == _ArmOperation.CONFIRM_STOP:
                owner = self._guarded_probe_monitor or self.guarded_probe_execution
                return self._finish_stop_confirmation(owner.poll())

            if (
                self._operation
                == _ArmOperation.SURFACE_ORIENTATION_VERIFY
            ):
                return self._poll_surface_orientation_verification()

            if self._operation == _ArmOperation.TAG_POSITION_VERIFY:
                return self._poll_tag_position_verification()

            # The guard must consume planning results and retain force monitoring.
            if self._operation == _ArmOperation.GUARDED_MOVEMENT:
                return self._poll_guarded_probe()

            if self._pending_moveit_plan_builder is not None:
                return self._advance_moveit_planning_start()

            if self._moveit_cartesian_plan is not None:
                return self._poll_moveit_planning()

            if self._verification_started is not None:
                return self._poll_state_confirmation()

            if self._operation == _ArmOperation.READY_WAIT:
                return self._advance_ready_probe()

            return self._poll_arm_goal()

    def _poll_arm_goal(self):
        if self._send_goal_future is None:
            if self._pending_goal_builder is not None:
                return super().poll()
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
        with self._execution_lock:
            if self.cancelling or not self.active:
                return
            self._cancel_goal_done = False
            self._reset_moveit_planning(cancel=True)
            super().cancel()
            if self.cancelling and not self._cancel_goal_done:
                self._monitor_cancel_stop()

    def _cancellation_goal_finished(self):
        with self._execution_lock:
            self._cancel_goal_done = True
            # A late goal acceptance could follow the first stop request.
            # Confirm another stop after the action has actually terminated.
            self._monitor_cancel_stop()

    def _monitor_cancel_stop(self):
        guard = self.guarded_probe_execution
        if guard is None:
            self._cancellation_detail = "Stop unconfirmed: arm stop monitor unavailable"
            return
        guard.reset()
        if self._guarded_probe_monitor is not None:
            self._guarded_probe_monitor.cancel(self._cancel_stop_finished)
        else:
            self._cancel_stop_finished(guard.cancel())

    def _cancel_stop_finished(self, update):
        with self._execution_lock:
            if update.outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED and self._cancel_goal_done:
                self._reset_operation()
            elif update.outcome is not ArmMovementOutcome.RUNNING:
                self._cancellation_detail = update.detail

    def shutdown(self) -> None:
        with self._execution_lock:
            super().shutdown()
            if self._guarded_probe_monitor is not None:
                self._guarded_probe_monitor.close()
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
        tag = self.tag_state_source.usable_tag(tag_id)
        if tag is None:
            raise RuntimeError(
                f"Tag {tag_id} is not currently usable"
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
        surface_normal=None,
    ):
        if surface_normal is None:
            surface_normal = self.surface_source.surface_normal()
        surface_normal_execution = (
            self._surface_normal_execution(surface_normal)
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

    def _surface_normal_execution(self, surface_normal) -> Vector3Data:
        projected = surface_normal.projected_point
        if surface_normal.stamp_nanoseconds <= 0:
            raise ValueError("Surface orientation depth timestamp is empty")
        camera_to_execution = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            projected.frame_id,
            transform_time=Time(
                nanoseconds=surface_normal.stamp_nanoseconds
            ),
            timeout_sec=0.0,
        )
        return rotate_vector(
            transform_to_pose_data(camera_to_execution).orientation,
            surface_normal.normal_camera,
        )

    def _surface_orientation_error_rad(
        self,
        sensor_id: str,
        surface_normal,
    ) -> float:
        outward = self._surface_normal_execution(surface_normal)
        current_probe = self.probe_motion_planner.current_pose(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            sensor_probe_frame(sensor_id),
        )
        probe_axis = rotate_vector(
            pose_to_pose_data(current_probe.pose).orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        alignment = -(
            probe_axis.x * outward.x
            + probe_axis.y * outward.y
            + probe_axis.z * outward.z
        )
        return math.acos(max(-1.0, min(1.0, alignment)))

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
            self._operation_speed = self.ready_speed
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
        self._next_probe_cartesian_path = self._guarded_cartesian_path
        owner = self._guarded_probe_monitor or self.guarded_probe_execution
        options = {}
        if self._guarded_retreat_distance_m is not None:
            options["retreat_distance_m"] = self._guarded_retreat_distance_m
        update = owner.start(
            builder,
            **options,
            force_threshold_n=self._guarded_force_threshold_n,
        )
        return self._finish_guarded_update(update)

    def _poll_guarded_probe(self) -> ArmMovementUpdate:
        owner = self._guarded_probe_monitor or self.guarded_probe_execution
        update = owner.poll()
        return self._finish_guarded_update(update)

    def _finish_guarded_update(
        self,
        update: ArmMovementUpdate,
    ) -> ArmMovementUpdate:
        if update.outcome is ArmMovementOutcome.RUNNING:
            return update
        if not self.active:
            return update
        if update.outcome is ArmMovementOutcome.SUCCESS and self._tag_accuracy is not None:
            if self._tag_accuracy["corrections"] and not self._tag_accuracy.get("verify_checkpoint"):
                return super()._finish(
                    ArmMovementOutcome.SUCCESS,
                    "Move-to-tag completed with one slow position adjustment",
                )
            return self._begin_tag_position_verification()
        if (
            update.outcome is ArmMovementOutcome.SUCCESS
            and self._surface_orientation_sensor_id is not None
        ):
            now = self._monotonic_clock()
            self._operation = _ArmOperation.SURFACE_ORIENTATION_VERIFY
            self._surface_orientation_verify_estimate = None
            self._surface_orientation_verify_not_before = now
            self._surface_orientation_verify_deadline = (
                now + SURFACE_ORIENTATION_SENSING_TIMEOUT_SEC
            )
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Surface orientation movement completed; verifying "
                "alignment from a fresh depth frame",
            )
        return super()._finish(update.outcome, update.detail)

    def _poll_surface_orientation_verification(
        self,
    ) -> ArmMovementUpdate:
        sensor_id = self._surface_orientation_sensor_id
        receipt_not_before = self._surface_orientation_verify_not_before
        deadline = self._surface_orientation_verify_deadline
        if (
            sensor_id is None
            or receipt_not_before is None
            or deadline is None
        ):
            return super()._finish(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Surface orientation verification lost its state",
            )

        try:
            estimate = getattr(self, "_surface_orientation_verify_estimate", None)
            if estimate is None:
                estimate = self.surface_source.surface_normal(
                    receipt_not_before=receipt_not_before,
                )
                self._surface_orientation_verify_estimate = estimate
                deadline = self._monotonic_clock() + SURFACE_ORIENTATION_TIMEOUT_SEC
                self._surface_orientation_verify_deadline = deadline
            # Retain this fresh post-motion observation while its TF catches up.
            error_rad = self._surface_orientation_error_rad(
                sensor_id,
                estimate,
            )
        except (ValueError, TransformException) as exception:
            return self._surface_orientation_verification_wait(exception, deadline)

        return self._handle_surface_orientation_error(sensor_id, estimate, error_rad)

    def _surface_orientation_verification_wait(self, exception, deadline):
        if self._monotonic_clock() < deadline:
            return ArmMovementUpdate(
                ArmMovementOutcome.RUNNING,
                "Waiting for fresh post-orientation surface "
                f"verification: {exception}",
            )
        return super()._finish(
            ArmMovementOutcome.EXECUTION_ERROR,
            "Surface orientation verification timed out while waiting for "
            + ("capture-time TF" if getattr(
                self, "_surface_orientation_verify_estimate", None,
            ) is not None else "fresh depth")
            + f": {exception}",
        )


    def _handle_surface_orientation_error(self, sensor_id, estimate, error_rad):
        error_deg = math.degrees(error_rad)
        if error_rad <= SURFACE_ORIENTATION_MAX_ERROR_RAD:
            return super()._finish(
                ArmMovementOutcome.SUCCESS,
                "Surface orientation verified at "
                f"{error_deg:.2f} deg axis error",
            )

        if (
            self._surface_orientation_corrections
            >= SURFACE_ORIENTATION_MAX_CORRECTIONS
        ):
            return super()._finish(
                ArmMovementOutcome.MOTION_FAILED,
                "Surface orientation remains outside tolerance after "
                f"{SURFACE_ORIENTATION_MAX_CORRECTIONS} correction: "
                f"{error_deg:.2f} deg > "
                f"{math.degrees(SURFACE_ORIENTATION_MAX_ERROR_RAD):.2f} deg",
            )

        self._surface_orientation_corrections += 1
        self._guarded_force_threshold_n = (
            self._surface_orientation_force_threshold_n
        )
        self._guarded_cartesian_path = False
        self._guarded_plan_builder = lambda: (
            self.probe_motion_planner.build_plan(
                lambda: self._resolve_surface_orientation_target(
                    sensor_id,
                    estimate,
                ),
                self._surface_orientation_speed,
            )
        )
        return self._begin_guarded_probe()

    def _finish(self, outcome, detail: str):
        if self._operation == _ArmOperation.GUARDED_MOVEMENT:
            # A goal ending must leave the guard active for stop confirmation.
            # Only _finish_guarded_update ends the enclosing arm operation.
            self._reset_moveit_planning(cancel=True)
            self._pending_goal_builder = None
            self._reset_goal_lifecycle()
            return ArmMovementUpdate(outcome, str(detail).strip())
        return super()._finish(outcome, detail)

    def _guard_poll_goal(self) -> ArmMovementUpdate:
        if self._pending_moveit_plan_builder is not None:
            return self._advance_moveit_planning_start()
        if self._moveit_cartesian_plan is not None:
            return self._poll_moveit_planning()
        if self._pending_goal_builder is not None:
            return super().poll()
        if self._send_goal_future is None:
            return ArmMovementUpdate(
                ArmMovementOutcome.EXECUTION_ERROR,
                "Guarded probe movement has no active RobotCommand goal",
            )
        if self._goal_handle is None:
            return self._poll_goal_response()
        return self._poll_result()

    def _guard_cancel_goal(self) -> None:
        self._reset_moveit_planning(cancel=True)
        self._pending_goal_builder = None
        if not self.cancelling:
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
        return arm_failure_result(result, self._command_failure_detail)


    def _reset_operation(self) -> None:
        with self._execution_lock:
            if self._guarded_probe_monitor is not None:
                self._guarded_probe_monitor.stop()
            self._reset_moveit_planning(cancel=True)
            super()._reset_operation()
            if hasattr(self, "_base_result_timeout_sec"):
                self.result_timeout_sec = self._base_result_timeout_sec
            self._clear_arm_operation_state()
            self._pending_moveit_cartesian_path = False
            self._clear_surface_orientation_state()
            self._reset_arm_stop_service_lifecycle(cancel=True)
            if self.guarded_probe_execution is not None:
                self.guarded_probe_execution.reset()

    def _clear_arm_operation_state(self) -> None:
        self._tag_accuracy = None
        self._operation = None
        self._operation_speed = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._verification_started = None
        self._ready_probe_start = None
        self._guarded_plan_builder = None
        self._guarded_retreat_distance_m = None
        self._guarded_force_threshold_n = None
        self._guarded_cartesian_path = False
        self._next_probe_cartesian_path = False

    def _clear_surface_orientation_state(self) -> None:
        self._surface_orientation_sensor_id = None
        self._surface_orientation_speed = None
        self._surface_orientation_force_threshold_n = None
        self._surface_orientation_corrections = 0
        self._surface_orientation_verify_not_before = None
        self._surface_orientation_verify_deadline = None
        self._surface_orientation_verify_estimate = None

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

        return self._continue_probe(
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
        return build_arm_stop_request()

    @staticmethod
    def _ready_forward_distance(value) -> float:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                "Ready arm forward distance must be non-negative and finite"
            )
        return value

    def _build_stow_goal(self) -> RobotCommand.Goal:
        return build_stow_goal()

    def _build_moveit_joint_goal(self, trajectory, minimum_duration_sec=None):
        goal, duration = build_moveit_joint_goal(trajectory, minimum_duration_sec)
        self.result_timeout_sec = max(
            self._base_result_timeout_sec,
            duration + self.moveit_result_timeout_margin_sec,
        )
        return goal

    def _build_pose_goal(self, target: PoseStamped, duration_sec: float):
        return build_pose_goal(target, duration_sec, self.robot_name)


__all__ = [
    "ArmMovementExecutor",
    "ArmMovementOutcome",
    "ArmMovementUpdate",
    "SURFACE_ORIENTATION_MAX_CORRECTIONS",
    "SURFACE_ORIENTATION_MAX_ERROR_RAD",
    "SURFACE_ORIENTATION_TIMEOUT_SEC",
]
