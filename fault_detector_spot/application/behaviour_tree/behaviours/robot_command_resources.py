"""Own process-wide ROS resources used by Spot robot commands."""

from threading import Event, RLock
from concurrent.futures import Future
from types import SimpleNamespace
import math
import time

from fault_detector_spot.navigation.walking_profile import WalkingProfiles

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from fault_detector_spot.navigation.base_correction_policy import (
    BaseCorrectionConfig,
    BaseCorrectionPolicy,
)
from fault_detector_spot.navigation.base_goal_verifier import (
    BaseGoalVerificationConfig,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource,
)
from fault_detector_spot.manipulation.arm_contact_telemetry import (
    ArmContactTelemetry,
)
from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineSampler,
)
from fault_detector_spot.manipulation.arm_joint_state_source import (
    ArmJointStateSource,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor, PLANNING_FRAME_MAX_AGE_SEC,
)
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.moveit_arm_planner import (
    MoveItArmPlanner,
)
from fault_detector_spot.manipulation.moveit_collision_scene import (
    MoveItCollisionScene,
)
from fault_detector_spot.manipulation.moveit_sensor_head import MoveItSensorHead
from fault_detector_spot.manipulation.moveit_scene_refresh import MoveItSceneRefresh
from fault_detector_spot.manipulation.arm_collision_control import ArmCollisionControl
from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStateSource,
)
from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BASE_READY_STANDING_TIMEOUT_PARAMETER,
    BASE_READY_STATE_TIMEOUT_PARAMETER,
    BASE_TAG_OBSERVATION_TIMEOUT_PARAMETER,
    DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC,
    DEFAULT_BASE_READY_STATE_TIMEOUT_SEC,
    DEFAULT_BASE_TAG_OBSERVATION_TIMEOUT_SEC,
    BaseMovementExecutor,
)
from fault_detector_spot.navigation.posture_state_source import (
    PostureStateSource,
)
from fault_detector_spot.sensing.observations.tag_observation_stability import (
    TagObservationStabilityConfig,
)


class _StopAwareActionClient:
    """Fence late planning results at the last action submission boundary."""

    def __init__(self, client, stopped, motion_started):
        self._client = client
        self._stopped = stopped
        self._motion_started = motion_started

    def __getattr__(self, name):
        return getattr(self._client, name)

    def send_goal_async(self, *args, **kwargs):
        if self._stopped():
            # This request never reached the wire. Return definite rejection,
            # not an exception that would correctly be treated as lost delivery.
            rejected = Future()
            rejected.set_result(SimpleNamespace(accepted=False))
            return rejected
        self._motion_started()
        future = self._client.send_goal_async(*args, **kwargs)
        # A stop can race the nonblocking send. Never let a late acceptance
        # escape cancellation merely because the owner has not ticked again.
        if future is not None:
            def cancel_if_stopped(completed):
                if not self._stopped():
                    return
                try:
                    handle = completed.result()
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    # The execution owner retains uncertain submission evidence.
                    pass
            future.add_done_callback(cancel_if_stopped)
        return future


class RobotCommandResources:
    """Share RobotCommand clients, TF, state, and movement executors."""

    def __init__(self, monotonic_clock=time.monotonic):
        self._lock = RLock()
        self._node = None
        self._clients = {}
        self._tf_listener = None
        self._arm_state_source = None
        self._arm_joint_state_source = None
        self._arm_contact_telemetry = None
        self._probe_surface_source = None
        self._posture_state_source = None
        self._arm_motion_speed_policy = None
        self._moveit_arm_planner = None
        self._moveit_scene_refresh = None
        self._arm_collision_control = None
        self._arm_movement_executors = {}
        self._base_movement_executors = {}
        self._waypoint_executor = None
        self._stop_arms = {}
        self._stop_bases = ()
        self._stop_retry_after = {}
        self._monotonic_clock = monotonic_clock
        self._software_stop = Event()
        self._latch_lock = RLock()
        self._stop_generation = 0
        self._reset_generation = None
        self._stop_started = False
        self._stop_confirmed = False
        self._stop_request_error = ""
        self._velocity_publisher = None
        self._velocity_subscription = None
        self._last_velocity_at = None
        self._stop_requested_at = None
        self._velocity_quiet_sec = 0.5
        self._closing = False
        self._shutdown_unconfirmed = False
        self._shutdown_confirmed = False

    def close_admission(self):
        """Fence late submissions before tree dispatch is paused."""
        self._closing = True
        # Shutdown never resets this latch. Avoid waiting behind a reset's
        # workflow locks before the coordinator can start its deadline.
        self._software_stop.set()

    def shutdown_owners(self):
        """Snapshot the existing owners; do not create resources at teardown."""
        owners = {f"arm/{name or 'default'}": owner
                  for name, owner in self._arm_movement_executors.copy().items()}
        owners.update({f"base/{name or 'default'}": owner
                       for name, owner in self._base_movement_executors.copy().items()})
        if self._waypoint_executor is not None:
            owners["nav2"] = self._waypoint_executor
        return owners

    def begin_shutdown(self):
        self.close_admission()
        self._stop_requested_at = self._monotonic_clock()
        if self._velocity_publisher is not None:
            self._velocity_publisher.publish(Twist())

    def poll_shutdown(self):
        return self.stop_sources_quiet(), "Waiting for base velocity command sources to become quiet"

    def report_shutdown_unconfirmed(self):
        """Permit teardown only after the coordinator has reported uncertainty."""
        self._shutdown_unconfirmed = True

    def report_shutdown_confirmed(self):
        """Authorize finalization using the coordinator's complete stop proof."""
        self._shutdown_confirmed = True

    @property
    def stop_latched(self):
        return self._closing or self._software_stop.is_set()

    def _invalidate_stop_proof(self):
        self._stop_confirmed = False

    def prepare_stop(self, node):
        """Create stop resources before advertising command readiness."""
        self.get_arm_movement_executor(node)
        self.get_base_movement_executor(node)
        # Dispatch must never lazily construct a physical owner after the
        # shutdown snapshot. This creates only a client and state subscription.
        self.get_waypoint_navigation_executor(node)
        if self._velocity_publisher is None:
            self._velocity_publisher = node.create_publisher(Twist, "cmd_vel", 10)
            self._velocity_subscription = node.create_subscription(
                Twist, "cmd_vel", self._receive_velocity, 10,
                callback_group=ReentrantCallbackGroup(),
            )

    def _receive_velocity(self, _message):
        self._last_velocity_at = self._monotonic_clock()
        for owner in tuple(self._base_movement_executors.values()):
            owner.invalidate_shutdown_stop()

    def stop_sources_quiet(self):
        if self._stop_requested_at is None:
            return False
        boundary = self._stop_requested_at
        if self._last_velocity_at is not None:
            boundary = max(boundary, self._last_velocity_at)
        return self._monotonic_clock() - boundary >= self._velocity_quiet_sec

    def latch_software_stop(self):
        """Fence a new operator stop without waiting for any execution owner."""
        with self._latch_lock:
            self._stop_generation += 1
            self._software_stop.set()

    def request_software_stop(self):
        """Latch before touching any owner or ordinary resource/workflow lock."""
        if not self.stop_latched:
            self.latch_software_stop()
        if self._stop_started:
            return
        self._stop_started = True
        self._stop_confirmed = False
        self._stop_requested_at = self._monotonic_clock()
        self._stop_request_error = ""
        # Resource construction is complete before command subscription starts.
        # Read snapshots without waiting for the construction/workflow lock.
        self._stop_arms = dict.fromkeys(tuple(self._arm_movement_executors.values()), False)
        self._stop_bases = tuple(self._base_movement_executors.values())
        self._stop_retry_after = {}
        operations = []
        if self._velocity_publisher is not None:
            operations.append(lambda: self._velocity_publisher.publish(Twist()))
        navigation = self._waypoint_executor
        # First ask every active physical source to cancel through its async
        # transport. A busy preparation lock in one owner must not delay the
        # cancellation request reaching another owner.
        owners = (*self._stop_arms, *self._stop_bases)
        if navigation is not None:
            owners = (navigation, *owners)
        operations.extend(executor.request_cancel for executor in owners)
        if navigation is not None:
            operations.append(navigation.cancel)
        operations.extend(executor.cancel for executor in self._stop_arms)
        operations.extend(executor.begin_stop_confirmation for executor in self._stop_bases)
        errors = []
        for operation in operations:
            try:
                operation()
            except Exception as exception:
                errors.append(str(exception))
        if errors:
            self._stop_request_error = "; ".join(errors)

    def begin_reset_confirmation(self):
        """Demand new post-request telemetry before clearing an existing latch."""
        if not self.stop_latched or not self._stop_confirmed:
            return False
        self._reset_generation = self._stop_generation
        self._stop_confirmed = False
        self._stop_arms = dict.fromkeys(tuple(self._arm_movement_executors.values()), False)
        self._stop_bases = tuple(self._base_movement_executors.values())
        self._stop_retry_after = {}
        for executor in self._stop_bases:
            executor.begin_stop_confirmation()
        return True

    def validate_recovery(self, node):
        """Validate explicit recovery without moving or clearing the latch."""
        from bosdyn.client.frame_helpers import BODY_FRAME_NAME
        from fault_detector_spot.inspection.model.sensor_models import BARE_HAND_MOTION_ID
        if self.stop_latched or not self._stop_confirmed or not self.execution_idle():
            return False, "Recovery requires an explicitly reset, confirmed software stop"
        if not self.stop_sources_quiet():
            return False, "Base command sources are not quiet"
        for executor in self._stop_bases:
            confirmed, detail = executor.poll_stop_confirmation()
            if not confirmed:
                return False, detail
        try:
            source = self.get_probe_surface_source(node)
            sensor_id, _revision = source.active_attachment()
            if sensor_id != BARE_HAND_MOTION_ID:
                return False, "Recovery stow requires confirmed bare hand; attached-sensor clearance is not proven"
            if not self.get_arm_collision_control(node).state().enabled:
                return False, "Recovery requires environment collision checking enabled"
            arm = self.get_arm_movement_executor(node)
            pose = arm.probe_motion_planner.current_hand_pose(BODY_FRAME_NAME)
            stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            if not math.isfinite(age) or not 0.0 <= age <= PLANNING_FRAME_MAX_AGE_SEC:
                return False, "Recovery requires a fresh measured arm pose"
            from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
            pose_to_pose_data(pose.pose).validate()
        except Exception as exception:
            return False, f"Recovery validation failed: {exception}"
        return True, "Fresh stop, bare-hand attachment, pose and collision policy validated"

    def reset_software_stop(self, clear_ledger_latch):
        """A reset changes admission only; it never submits movement."""
        if self._closing:
            return False, "Shutdown has closed motion admission"
        if not self._stop_confirmed or not self.execution_idle() or not self.stop_sources_quiet():
            return False, "Software stop is not confirmed; reset remains blocked"
        for executor in self._stop_bases:
            confirmed, detail = executor.poll_stop_confirmation()
            if not confirmed:
                return False, detail
        with self._latch_lock:
            if self._reset_generation != self._stop_generation:
                return False, "A newer software stop superseded this reset"
            clear_ledger_latch()
            self._software_stop.clear()
            self._stop_started = False
        return True, "Software stop reset; discarded work will not replay"

    def execution_idle(self):
        """Use retained action evidence as well as operation ownership."""
        with self._lock:
            executors = tuple(self._arm_movement_executors.values()) + tuple(
                self._base_movement_executors.values()
            )
            navigation = self._waypoint_executor
        return (all(executor.execution_idle for executor in executors)
                and (navigation is None or navigation.execution_idle))

    def begin_stop_confirmation(self, node):
        """Request cancellation through the existing movement owners."""
        if not self._stop_started:
            self.prepare_stop(node)
        self.request_software_stop()

    def poll_stop_confirmation(self):
        """Wait for terminal actions and fresh physical stop evidence."""
        navigation = self._waypoint_executor
        if navigation is not None and navigation.active:
            navigation.cancel()
            update = navigation.poll()
            if navigation.active:
                return False, update.detail
        if navigation is not None and not navigation.execution_idle:
            return False, "Nav2 action termination is unconfirmed"
        if not self.stop_sources_quiet():
            return False, "Waiting for all base velocity command sources to become quiet"
        for executor in self._stop_bases:
            confirmed, detail = executor.poll_stop_confirmation()
            if not confirmed:
                return False, detail
        for executor, started in tuple(self._stop_arms.items()):
            if not started:
                if self._monotonic_clock() < self._stop_retry_after.get(executor, 0.0):
                    return False, "Waiting to retry arm stop confirmation"
                if executor.active:
                    executor.cancel()
                    update = executor.poll()
                    if executor.active:
                        return False, update.detail
                if not executor.goals_settled():
                    return False, "Arm action termination is unconfirmed"
                update = executor.confirm_stop()
                self._stop_arms[executor] = True
            else:
                update = executor.poll()
            if update.outcome is ArmMovementOutcome.RUNNING:
                return False, update.detail
            if update.outcome is not ArmMovementOutcome.SUCCESS:
                self._stop_arms[executor] = False
                self._stop_retry_after[executor] = self._monotonic_clock() + 1.0
                return False, update.detail
            del self._stop_arms[executor]
        self._stop_confirmed = self.execution_idle()
        if self._stop_confirmed:
            # Request failures are uncertainty, not permanent proof of motion.
            # Only later terminal action and fresh physical evidence reconcile
            # them; neither elapsed time nor another request clears this record.
            self._stop_request_error = ""
            return True, "All owned movements have confirmed stopping; software stop remains latched"
        return False, "Waiting for every execution owner to confirm idle"

    def get_action_client(self, node, robot_name: str = ""):
        """Return the single RobotCommand client for a robot namespace."""
        with self._lock:
            self._bind_node(node)
            action_ns = namespace_with(robot_name, "robot_command")
            client = self._clients.get(action_ns)
            if client is None:
                client = ActionClientWrapper(
                    RobotCommand,
                    action_ns,
                    node,
                    wait_for_server=False,
                )
                client = _StopAwareActionClient(client, lambda: self.stop_latched, self._invalidate_stop_proof)
                self._clients[action_ns] = client
            return client

    def get_tf_listener(self, node):
        """Own one TF listener; all BT consumers borrow its buffer."""
        with self._lock:
            self._bind_node(node)
            if self._tf_listener is None:
                self._tf_listener = TFListenerWrapper(node)
            return self._tf_listener

    def get_arm_state_source(self, node):
        """Return the authoritative manipulator state source."""
        with self._lock:
            self._bind_node(node)
            if self._arm_state_source is None:
                self._arm_state_source = ArmStateSource(node)
            return self._arm_state_source

    def get_arm_joint_state_source(self, node):
        """Return the shared six-axis arm joint telemetry source."""
        with self._lock:
            self._bind_node(node)
            if self._arm_joint_state_source is None:
                self._arm_joint_state_source = ArmJointStateSource(node)
            return self._arm_joint_state_source

    def get_arm_contact_telemetry(self, node):
        """Return shared shadow telemetry for guarded arm movement."""
        with self._lock:
            self._bind_node(node)
            if self._arm_contact_telemetry is None:
                self._arm_contact_telemetry = ArmContactTelemetry.from_node(
                    node,
                    self.get_arm_state_source(node),
                    self.get_arm_joint_state_source(node),
                )
            return self._arm_contact_telemetry

    def get_probe_surface_source(self, node):
        """Return shared live hand-depth and attachment state."""
        with self._lock:
            self._bind_node(node)
            if self._probe_surface_source is None:
                self._probe_surface_source = ProbeSurfaceSource(
                    node, self.get_tf_listener(node).buffer,
                )
            return self._probe_surface_source

    def get_posture_state_source(self, node):
        """Return the authoritative base posture state source."""
        with self._lock:
            self._bind_node(node)
            if self._posture_state_source is None:
                self._posture_state_source = PostureStateSource(node)
            return self._posture_state_source

    def get_arm_collision_control(self, node):
        """Share the startup collision preference independently of mapping or planning."""
        with self._lock:
            self._bind_node(node)
            if self._arm_collision_control is None:
                self._arm_collision_control = ArmCollisionControl(node)
            return self._arm_collision_control

    def get_moveit_arm_planner(self, node):
        """Return the shared MoveIt planning client for the Spot arm."""
        with self._lock:
            self._bind_node(node)
            if self._moveit_arm_planner is None:
                config = ArmMotionParameters(node)
                sensor_head = MoveItSensorHead(node)
                collision_scene = MoveItCollisionScene(
                    node, self.get_arm_collision_control(node),
                    sensor_head=sensor_head,
                )
                self._moveit_arm_planner = MoveItArmPlanner(
                    node,
                    allowed_planning_time_sec=config.get(
                        "motion.moveit_allowed_planning_time_sec"
                    ),
                    velocity_scaling=config.get(
                        "motion.moveit_velocity_scaling"
                    ),
                    acceleration_scaling=config.get(
                        "motion.moveit_acceleration_scaling"
                    ),
                    min_arm_sh1_rad=config.get(
                        "motion.arm_sh1_safe_min_rad"
                    ),
                    collision_scene=collision_scene,
                )
                self._moveit_scene_refresh = MoveItSceneRefresh(
                    node, self._moveit_arm_planner, sensor_head, self._arm_motion_idle,
                )
            return self._moveit_arm_planner

    def _arm_motion_idle(self):
        with self._lock:
            return not any(executor.active for executor in self._arm_movement_executors.values())

    def get_arm_movement_executor(
        self,
        node,
        tag_state_source=None,
        robot_name: str = "",
    ):
        """Return the shared Cartesian arm movement executor."""
        with self._lock:
            self._bind_node(node)
            executor = self._arm_movement_executors.get(robot_name)
            if executor is None:
                if self._arm_motion_speed_policy is None:
                    self._arm_motion_speed_policy = (
                        ArmMotionSpeedPolicy.from_node(node)
                    )
                executor = ArmMovementExecutor(
                    self.get_tf_listener(node),
                    config=ArmMotionParameters(node),
                    tag_state_source=tag_state_source,
                    surface_source=self.get_probe_surface_source(node),
                    robot_name=robot_name,
                    speed_policy=self._arm_motion_speed_policy,
                    action_client=self.get_action_client(
                        node,
                        robot_name,
                    ),
                    moveit_arm_planner=self.get_moveit_arm_planner(node),
                    arm_state_source=self.get_arm_state_source(node),
                    force_baseline_sampler=ForceBaselineSampler.from_node(
                        node,
                        self.get_arm_state_source(node),
                    ),
                    force_contact_policy=(
                        SpeedAwareForceContactPolicy.from_node(node)
                    ),
                    logger=node.get_logger(),
                )
                if executor.guarded_probe_execution is not None:
                    executor.guarded_probe_execution.contact_telemetry = (
                        self.get_arm_contact_telemetry(node)
                    )
                self._arm_movement_executors[robot_name] = executor
            elif tag_state_source is not None:
                if executor.tag_state_source is None:
                    executor.tag_state_source = tag_state_source
                elif executor.tag_state_source is not tag_state_source:
                    raise RuntimeError(
                        "Arm movement executor already uses another "
                        "tag state source"
                    )
            return executor

    def get_waypoint_navigation_executor(self, node):
        """Share navigation ownership with emergency cancellation."""
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from fault_detector_spot.navigation.waypoint_navigation_executor import WaypointNavigationExecutor

        with self._lock:
            self._bind_node(node)
            if self._waypoint_executor is None:
                self._waypoint_executor = WaypointNavigationExecutor(
                    self.get_arm_movement_executor(node),
                    self.get_base_movement_executor(node),
                    _StopAwareActionClient(
                        ActionClient(node, NavigateToPose, "/navigate_to_pose"),
                        lambda: self.stop_latched, self._invalidate_stop_proof,
                    ),
                    stamp_now=lambda: node.get_clock().now().to_msg(),
                    node=node,
                )
            return self._waypoint_executor

    def navigation_stopping(self):
        """Expose outstanding Nav2 stop confirmation to emergency orchestration."""
        return self._waypoint_executor is not None and self._waypoint_executor.cancelling

    def get_base_movement_executor(
        self,
        node,
        tag_state_source=None,
        robot_name: str = "",
    ):
        """Return the shared base movement executor."""
        with self._lock:
            self._bind_node(node)
            executor = self._base_movement_executors.get(robot_name)
            if executor is None:
                executor = BaseMovementExecutor(
                    self.get_tf_listener(node),
                    arm_state_source=self.get_arm_state_source(node),
                    tag_state_source=tag_state_source,
                    robot_name=robot_name,
                    action_client=self.get_action_client(
                        node,
                        robot_name,
                    ),
                    posture_state_source=self.get_posture_state_source(
                        node
                    ),
                    correction_policy=BaseCorrectionPolicy(
                        BaseCorrectionConfig.from_node(node)
                    ),
                    ready_state_timeout_sec=self._positive_parameter(
                        node,
                        BASE_READY_STATE_TIMEOUT_PARAMETER,
                        DEFAULT_BASE_READY_STATE_TIMEOUT_SEC,
                    ),
                    ready_standing_timeout_sec=self._positive_parameter(
                        node,
                        BASE_READY_STANDING_TIMEOUT_PARAMETER,
                        DEFAULT_BASE_READY_STANDING_TIMEOUT_SEC,
                    ),
                    walking_profiles=WalkingProfiles.from_node(node),
                    goal_verification_config=(
                        BaseGoalVerificationConfig.from_node(node)
                    ),
                    tag_stability_config=(
                        TagObservationStabilityConfig.from_node(node)
                    ),
                    tag_observation_timeout_sec=self._positive_parameter(
                        node,
                        BASE_TAG_OBSERVATION_TIMEOUT_PARAMETER,
                        DEFAULT_BASE_TAG_OBSERVATION_TIMEOUT_SEC,
                    ),
                    ros_time_sec=lambda: (
                        node.get_clock().now().nanoseconds * 1e-9
                    ),
                    logger=node.get_logger(),
                )
                self._base_movement_executors[robot_name] = executor
            elif tag_state_source is not None:
                if executor.tag_state_source is None:
                    executor.tag_state_source = tag_state_source
                elif executor.tag_state_source is not tag_state_source:
                    raise RuntimeError(
                        "Base movement executor already uses another "
                        "tag state source"
                    )
            return executor

    def cleanup_operations(self):
        """Retain each handle until its cleanup succeeds, independently."""
        operations = {}

        def attribute(name, method):
            resource = getattr(self, name)
            if resource is None:
                return

            def release():
                result = method(resource)
                if result is False:
                    raise RuntimeError(f"{name} cleanup reported incomplete")
                if getattr(self, name) is resource:
                    setattr(self, name, None)
            operations[name] = release

        def executor_cleanup(owner):
            return owner.shutdown(stop_unconfirmed=self._shutdown_unconfirmed,
                                  stop_confirmed=self._shutdown_confirmed)

        attribute('_waypoint_executor', executor_cleanup)
        for collection_name in ('_arm_movement_executors', '_base_movement_executors'):
            collection = getattr(self, collection_name)
            for name, owner in tuple(collection.items()):
                def release(collection=collection, name=name, owner=owner):
                    executor_cleanup(owner)
                    if collection.get(name) is owner:
                        del collection[name]
                operations[f"{collection_name}/{name or 'default'}"] = release
        attribute('_moveit_scene_refresh', lambda resource: resource.destroy())
        attribute('_moveit_arm_planner', lambda resource: resource.destroy())
        attribute('_arm_collision_control', lambda resource: resource.destroy())
        attribute('_probe_surface_source', lambda resource: resource.destroy())
        attribute('_arm_contact_telemetry', lambda resource: resource.close())
        attribute('_arm_state_source', lambda resource: resource.destroy())
        attribute('_arm_joint_state_source', lambda resource: resource.destroy())
        attribute('_posture_state_source', lambda resource: resource.destroy())
        attribute('_tf_listener', lambda resource: resource.shutdown())
        attribute('_velocity_subscription', lambda resource: self._node.destroy_subscription(resource))
        attribute('_velocity_publisher', lambda resource: self._node.destroy_publisher(resource))
        for name, client in tuple(self._clients.items()):
            def release(name=name, client=client):
                client.destroy()
                if self._clients.get(name) is client:
                    del self._clients[name]
            operations[f"action_client/{name}"] = release
        return operations

    def close(self):
        """Release independent resources and retain failures for another close."""
        if (self.shutdown_owners()
                and not (self._shutdown_confirmed or self._shutdown_unconfirmed)):
            raise RuntimeError("Ordered shutdown confirmation or explicit timeout is required before teardown")
        errors = []
        for name, release in self.cleanup_operations().items():
            try:
                release()
            except Exception as exception:
                errors.append(f"{name}: {exception}")
        if errors:
            raise RuntimeError("; ".join(errors))

    @staticmethod
    def _positive_parameter(node, name: str, default: float) -> float:
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        value = float(node.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(
                f"Parameter '{name}' must be positive"
            )
        return value

    def _bind_node(self, node):
        if self._closing:
            raise RuntimeError("Shutdown has closed robot resource admission")
        if node is None:
            raise RuntimeError(
                "RobotCommandResources requires a ROS node"
            )
        if self._node is None:
            self._node = node
            return
        if self._node is not node:
            raise RuntimeError(
                "RobotCommandResources cannot be shared across ROS nodes"
            )
