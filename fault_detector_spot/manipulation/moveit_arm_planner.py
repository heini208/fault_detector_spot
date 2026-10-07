"""Nonblocking MoveIt planning client for Spot hand-pose targets."""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty

from fault_detector_spot.manipulation.moveit_collision_policy import (
    occupancy_collision_matrix,
)

ARM_JOINT_NAMES = (
    "arm_sh0",
    "arm_sh1",
    "arm_el0",
    "arm_el1",
    "arm_wr0",
    "arm_wr1",
)
DEFAULT_SERVICE_NAME = "/plan_kinematic_path"
DEFAULT_CARTESIAN_SERVICE_NAME = "/compute_cartesian_path"
DEFAULT_PLANNING_FRAME = "body"
DEFAULT_GROUP_NAME = "arm"
DEFAULT_END_EFFECTOR_LINK = "hand"
DEFAULT_PLANNER_ID = "RRTConnectkConfigDefault"
DEFAULT_ALLOWED_PLANNING_TIME_SEC = 5.0
DEFAULT_RESPONSE_TIMEOUT_SEC = 7.0
DEFAULT_POSITION_TOLERANCE_M = 0.005
DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.01
DEFAULT_VELOCITY_SCALING = 0.5
DEFAULT_ACCELERATION_SCALING = 0.4
DEFAULT_CARTESIAN_MAX_STEP_M = 0.002
DEFAULT_CARTESIAN_JUMP_THRESHOLD = 2.0
DEFAULT_CARTESIAN_MIN_FRACTION = 0.999
MIN_ARM_SH1_RAD = math.radians(-178.0)

class MoveItPlanOutcome(Enum):
    RUNNING = "running"
    SUCCESS = "success"
    SERVICE_UNAVAILABLE = "service_unavailable"
    TIMEOUT = "timeout"
    FAILURE = "failure"
    ERROR = "error"


@dataclass(frozen=True)
class MoveItPlanUpdate:
    outcome: MoveItPlanOutcome
    detail: str
    trajectory: object | None = None


class MoveItArmPlanner:
    """Plan normal and straight Cartesian hand paths without executing them.

    The optional occupancy policy requires one serialized planning client and
    ACM writer. It changes the shared scene, so independent concurrent planning
    clients must not use that scene. Existing callers keep the direct path.
    """

    def __init__(
        self,
        node,
        service_name: str = DEFAULT_SERVICE_NAME,
        cartesian_service_name: str = DEFAULT_CARTESIAN_SERVICE_NAME,
        planning_frame: str = DEFAULT_PLANNING_FRAME,
        group_name: str = DEFAULT_GROUP_NAME,
        end_effector_link: str = DEFAULT_END_EFFECTOR_LINK,
        planner_id: str = DEFAULT_PLANNER_ID,
        allowed_planning_time_sec: float = (
            DEFAULT_ALLOWED_PLANNING_TIME_SEC
        ),
        response_timeout_sec: float = DEFAULT_RESPONSE_TIMEOUT_SEC,
        position_tolerance_m: float = DEFAULT_POSITION_TOLERANCE_M,
        orientation_tolerance_rad: float = (
            DEFAULT_ORIENTATION_TOLERANCE_RAD
        ),
        velocity_scaling: float = DEFAULT_VELOCITY_SCALING,
        acceleration_scaling: float = DEFAULT_ACCELERATION_SCALING,
        cartesian_max_step_m: float = DEFAULT_CARTESIAN_MAX_STEP_M,
        cartesian_jump_threshold: float = DEFAULT_CARTESIAN_JUMP_THRESHOLD,
        cartesian_min_fraction: float = DEFAULT_CARTESIAN_MIN_FRACTION,
        min_arm_sh1_rad: float = MIN_ARM_SH1_RAD,
        monotonic_clock=time.monotonic,
        environment_collision_policy_enabled: bool = False,
        get_scene_service_name: str = "/get_planning_scene",
        apply_scene_service_name: str = "/apply_planning_scene",
        environment_source=None,
        clear_octomap_service_name: str = "/clear_octomap",
    ):
        if node is None:
            raise ValueError("MoveItArmPlanner requires a ROS node")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        if not isinstance(environment_collision_policy_enabled, bool):
            raise TypeError("Environmental collision policy enabled must be a boolean")
        self.environment_collision_policy_enabled = environment_collision_policy_enabled
        if environment_source is not None and not environment_collision_policy_enabled:
            raise ValueError("Environment observations require collision policy handling")
        self._environment_source = environment_source
        self.get_scene_service_name = str(get_scene_service_name).strip()
        self.apply_scene_service_name = str(apply_scene_service_name).strip()
        if not self.get_scene_service_name or not self.apply_scene_service_name:
            raise ValueError("MoveIt scene service names must not be empty")

        self.node = node
        self.service_name = str(service_name).strip()
        self.cartesian_service_name = str(cartesian_service_name).strip()
        self.planning_frame = str(planning_frame).strip()
        self.group_name = str(group_name).strip()
        self.end_effector_link = str(end_effector_link).strip()
        self.planner_id = str(planner_id).strip()
        self.allowed_planning_time_sec = self._positive(
            allowed_planning_time_sec,
            "Allowed planning time",
        )
        self.response_timeout_sec = self._positive(
            response_timeout_sec,
            "MoveIt response timeout",
        )
        self.position_tolerance_m = self._positive(
            position_tolerance_m,
            "Position tolerance",
        )
        self.orientation_tolerance_rad = self._positive(
            orientation_tolerance_rad,
            "Orientation tolerance",
        )
        self.velocity_scaling = self._scaling(
            velocity_scaling,
            "Velocity scaling",
        )
        self.acceleration_scaling = self._scaling(
            acceleration_scaling,
            "Acceleration scaling",
        )
        self.cartesian_max_step_m = self._positive(
            cartesian_max_step_m,
            "Cartesian maximum step",
        )
        self.cartesian_jump_threshold = self._nonnegative(
            cartesian_jump_threshold,
            "Cartesian jump threshold",
        )
        self.cartesian_min_fraction = self._fraction(
            cartesian_min_fraction,
            "Cartesian minimum fraction",
        )
        self.min_arm_sh1_rad = self._finite(
            min_arm_sh1_rad,
            "arm_sh1 safety floor",
        )
        if not self.service_name:
            raise ValueError("MoveIt planning service name must not be empty")
        if not self.cartesian_service_name:
            raise ValueError(
                "MoveIt Cartesian planning service name must not be empty"
            )
        if not self.planning_frame:
            raise ValueError("MoveIt planning frame must not be empty")
        if not self.group_name:
            raise ValueError("MoveIt planning group must not be empty")
        if not self.end_effector_link:
            raise ValueError("MoveIt end-effector link must not be empty")

        self._monotonic_clock = monotonic_clock
        self._logger = node.get_logger()
        self._client = node.create_client(
            GetMotionPlan,
            self.service_name,
        )
        self._cartesian_client = node.create_client(
            GetCartesianPath,
            self.cartesian_service_name,
        )
        self._get_scene_client = None
        self._apply_scene_client = None
        self._clear_octomap_client = None
        if self.environment_collision_policy_enabled:
            self._get_scene_client = node.create_client(
                GetPlanningScene, self.get_scene_service_name,
            )
            self._apply_scene_client = node.create_client(
                ApplyPlanningScene, self.apply_scene_service_name,
            )
            if environment_source is not None:
                self._clear_octomap_client = node.create_client(
                    Empty, clear_octomap_service_name,
                )
        self._future = None
        self._started_at = None
        self._planning_mode = None
        self._stage = "plan"
        self._pending_request = None
        self._ignore_environment_collisions = False
        self._discard_result = False
        self._service_fault = ""

    @property
    def active(self) -> bool:
        self._release_discarded_request()
        return (self._future is not None or self._pending_request is not None
                or bool(self._service_fault))

    def start(
        self, target_hand: PoseStamped, *, ignore_environment_collisions=False,
    ) -> MoveItPlanUpdate:
        return self._start(target_hand, "motion", ignore_environment_collisions)

    def start_cartesian(
        self, target_hand: PoseStamped, *, ignore_environment_collisions=False,
    ) -> MoveItPlanUpdate:
        return self._start(target_hand, "cartesian", ignore_environment_collisions)

    def _start(self, target_hand, mode, ignore_environment_collisions):
        error = self._target_error(target_hand)
        if error is not None:
            return error
        if not isinstance(ignore_environment_collisions, bool):
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "Ignore environmental collisions must be a boolean",
            )
        if ignore_environment_collisions and not self.environment_collision_policy_enabled:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "Environmental collision policy is not enabled for this planner",
            )

        try:
            client = self._cartesian_client if mode == "cartesian" else self._client
            clients = [client]
            if self.environment_collision_policy_enabled:
                clients.extend([self._get_scene_client, self._apply_scene_client])
            if self._environment_source is not None and not ignore_environment_collisions:
                clients.append(self._clear_octomap_client)
            if any(not item.wait_for_service(timeout_sec=0.0) for item in clients):
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.SERVICE_UNAVAILABLE,
                    "Required MoveIt planning or scene service is unavailable",
                )
            self._pending_request = (
                self._build_cartesian_request(target_hand)
                if mode == "cartesian" else self._build_request(target_hand)
            )
            self._planning_mode = mode
            self._ignore_environment_collisions = ignore_environment_collisions
            if self.environment_collision_policy_enabled:
                if self._environment_source is not None and not ignore_environment_collisions:
                    problem = self._environment_source.begin_refresh()
                    if problem:
                        # The shared source may have been created for this first
                        # command. Let its subscriptions receive data before failing.
                        self._stage = "prepare_map"
                        self._started_at = self._monotonic_clock()
                        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, problem)
                    try:
                        return self._begin_future(
                            self._clear_octomap_client.call_async(Empty.Request()), "clear_map",
                        )
                    except Exception as exception:
                        return self._request_error(exception)
                return self._read_scene()
            return self._submit_plan()
        except Exception as exception:
            return self._request_error(exception, uncertain=False)

    def _submit_plan(self):
        client = self._cartesian_client if self._planning_mode == "cartesian" else self._client
        return self._begin_future(client.call_async(self._pending_request), "plan")

    def _read_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        if self._environment_source is not None and not self._ignore_environment_collisions:
            request.components.components |= PlanningSceneComponents.OCTOMAP
        return self._begin_future(self._get_scene_client.call_async(request), "read_scene")

    def _begin_future(self, future, stage):
        if future is None:
            raise RuntimeError("MoveIt service returned no future")
        self._future = future
        self._stage = stage
        self._started_at = self._monotonic_clock()
        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, self._waiting_detail())

    def _waiting_detail(self):
        if self._stage == "prepare_map":
            return "Waiting for fresh stationary base observations"
        if self._stage == "clear_map":
            return "Clearing the previous local obstacle map"
        if self._stage == "observe":
            return "Waiting for fresh filtered depth and an occupied local map"
        if self._stage == "read_scene":
            return "Reading MoveIt collision rules"
        if self._stage == "apply_scene":
            return "Applying MoveIt environmental collision policy"
        detail = (
            "Waiting for MoveIt Cartesian path"
            if self._planning_mode == "cartesian" else "Waiting for MoveIt arm plan"
        )
        if self.environment_collision_policy_enabled:
            detail += (
                " (sensor obstacles ignored)" if self._ignore_environment_collisions
                else " (sensor obstacles checked)"
            )
        return detail

    def poll(self) -> MoveItPlanUpdate:
        if self._discard_result:
            self._release_discarded_request()
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "MoveIt request was cancelled or timed out; its result is discarded",
            )
        if self._stage == "prepare_map":
            problem = self._environment_source.begin_refresh()
            if self._monotonic_clock() - self._started_at >= self.response_timeout_sec:
                self.cancel()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.TIMEOUT, problem or "Environment preparation timed out",
                )
            if problem:
                return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, problem)
            try:
                return self._begin_future(
                    self._clear_octomap_client.call_async(Empty.Request()), "clear_map",
                )
            except Exception as exception:
                return self._request_error(exception)
        checked_environment = (
            self._environment_source is not None and self._pending_request is not None
            and not self._ignore_environment_collisions
        )
        if checked_environment:
            problem = self._environment_source.motion_problem()
            if problem:
                self.cancel()
                return MoveItPlanUpdate(MoveItPlanOutcome.FAILURE, problem)
        if self._stage == "observe":
            if self._monotonic_clock() - self._started_at >= self.response_timeout_sec:
                self.cancel()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.TIMEOUT,
                    "No fresh usable environment map; checking remains unavailable",
                )
            if self._environment_source.has_fresh_cloud():
                try:
                    return self._read_scene()
                except Exception as exception:
                    return self._request_error(exception, uncertain=False)
            return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, self._waiting_detail())
        if checked_environment and self._stage in ("apply_scene", "plan"):
            if not self._environment_source.has_fresh_cloud():
                self.cancel()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.FAILURE, "Environment observations became stale during planning",
                )
        future = self._future
        if future is None:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                self._service_fault or "No MoveIt arm plan is active",
            )
        if not future.done():
            if self._monotonic_clock() - self._started_at >= self.response_timeout_sec:
                detail = self._waiting_detail()
                self.cancel()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.TIMEOUT,
                    f"{detail} timed out after {self.response_timeout_sec:.1f} s",
                )
            return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, self._waiting_detail())

        try:
            response = future.result()
            if response is None:
                raise RuntimeError("MoveIt service returned no response")
        except Exception as exception:
            return self._request_error(exception, uncertain=self._stage != "read_scene")

        if self._stage == "clear_map":
            self._environment_source.map_cleared()
            self._future = None
            self._stage = "observe"
            self._started_at = self._monotonic_clock()
            return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, self._waiting_detail())

        if self._stage == "read_scene":
            if checked_environment and not response.scene.world.octomap.octomap.data:
                # A filtered cloud alone cannot establish that move_group has
                # a populated scene. Refuse an empty tree, without fallback.
                self._reset()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.SERVICE_UNAVAILABLE,
                    "MoveIt has no occupied environment map yet; retry after depth updates",
                )
            if checked_environment and response.scene.world.octomap.header.frame_id != self.planning_frame:
                self._reset()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.ERROR, "Local obstacle map is not in the arm planning frame",
                )
            try:
                request = ApplyPlanningScene.Request()
                request.scene.is_diff = True
                request.scene.robot_state.is_diff = True
                request.scene.allowed_collision_matrix = occupancy_collision_matrix(
                    response.scene.allowed_collision_matrix,
                    ignore_environment_collisions=self._ignore_environment_collisions,
                )
            except Exception as exception:
                self._reset()
                return MoveItPlanUpdate(MoveItPlanOutcome.ERROR, str(exception))
            try:
                return self._begin_future(
                    self._apply_scene_client.call_async(request), "apply_scene",
                )
            except Exception as exception:
                return self._request_error(exception)

        if self._stage == "apply_scene":
            if not response.success:
                self._reset()
                return MoveItPlanUpdate(
                    MoveItPlanOutcome.FAILURE,
                    "MoveIt rejected the environmental collision policy; no plan submitted",
                )
            try:
                return self._submit_plan()
            except Exception as exception:
                return self._request_error(exception)

        mode = self._planning_mode
        self._reset()
        if mode == "cartesian":
            return self._cartesian_result(response)
        return self._motion_plan_result(response)

    def _request_error(self, exception, *, uncertain=True):
        detail = f"MoveIt service request failed: {exception}"
        self._reset()
        if self.environment_collision_policy_enabled and uncertain:
            # A transport error does not establish that server work has ended.
            self._service_fault = (
                detail + "; server state is uncertain; verify MoveIt is idle "
                "before reinitializing the planner"
            )
        return MoveItPlanUpdate(MoveItPlanOutcome.ERROR, self._service_fault or detail)

    def cancel(self) -> None:
        if not self.environment_collision_policy_enabled:
            # Preserve existing cancellation when no shared policy is in use.
            future = self._future
            self._reset()
            if future is not None:
                try:
                    future.cancel()
                except Exception:
                    pass
            return
        if self._future is not None:
            # ROS services cannot cancel server computation. Keep the future
            # until its reply, but never consume it as a plan or a next stage.
            self._discard_result = True
            self._pending_request = None
            self._release_discarded_request()
        else:
            self._reset()

    def _release_discarded_request(self):
        if self._discard_result and self._future is not None and self._future.done():
            try:
                if self._future.result() is None:
                    raise RuntimeError("MoveIt service returned no response")
            except Exception as exception:
                self._request_error(exception, uncertain=self._stage != "read_scene")
            else:
                self._reset()

    def destroy(self) -> None:
        self.cancel()
        clients = (
            self._client, self._cartesian_client,
            self._get_scene_client, self._apply_scene_client,
            self._clear_octomap_client,
        )
        self._client = self._cartesian_client = None
        self._get_scene_client = self._apply_scene_client = None
        self._clear_octomap_client = None
        self._service_fault = "MoveIt planner has been destroyed"
        for client in clients:
            if client is not None:
                self.node.destroy_client(client)
        if self._environment_source is not None:
            self._environment_source.destroy()

    def _target_error(self, target_hand: PoseStamped):
        busy = self.active
        if self._service_fault:
            return MoveItPlanUpdate(MoveItPlanOutcome.SERVICE_UNAVAILABLE, self._service_fault)
        if busy:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "Another MoveIt service request is still outstanding",
            )
        if not isinstance(target_hand, PoseStamped):
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "MoveIt arm target must be a PoseStamped",
            )
        if target_hand.header.frame_id.strip() != self.planning_frame:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "MoveIt arm target must be expressed in planning frame "
                f"'{self.planning_frame}'",
            )
        return None

    def _motion_plan_result(self, response) -> MoveItPlanUpdate:
        result = response.motion_plan_response
        error_code = int(result.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.FAILURE,
                self._planning_failure_detail(error_code),
            )

        trajectory = deepcopy(result.trajectory.joint_trajectory)
        try:
            detail = self._validate_and_describe(trajectory)
        except Exception as exception:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                f"MoveIt returned an invalid arm trajectory: {exception}",
            )

        self._logger.info(detail)
        return MoveItPlanUpdate(
            MoveItPlanOutcome.SUCCESS,
            detail,
            trajectory=trajectory,
        )

    def _cartesian_result(self, response) -> MoveItPlanUpdate:
        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.FAILURE,
                "MoveIt Cartesian path failed: "
                f"{self._planning_failure_detail(error_code)}",
            )

        fraction = float(response.fraction)
        if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                f"MoveIt Cartesian path returned invalid fraction {fraction}",
            )
        trajectory = response.solution.joint_trajectory
        if fraction + 1e-12 < self.cartesian_min_fraction:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.FAILURE,
                "MoveIt Cartesian path is incomplete: "
                f"fraction {fraction:.6f} < "
                f"{self.cartesian_min_fraction:.6f}; "
                f"{self._partial_trajectory_detail(trajectory)}; "
                "cause is not reported by the Cartesian service "
                "(IK, collision, or jump rejection); inspect move_group logs "
                "for the rejected segment",
            )

        trajectory = deepcopy(trajectory)
        try:
            trajectory_detail = self._validate_and_describe(trajectory)
        except Exception as exception:
            return MoveItPlanUpdate(
                MoveItPlanOutcome.ERROR,
                "MoveIt returned an invalid Cartesian arm trajectory: "
                f"{exception}",
            )

        detail = (
            f"MoveIt Cartesian path ready: fraction {fraction:.6f}; "
            f"{trajectory_detail}"
        )
        self._logger.info(detail)
        return MoveItPlanUpdate(
            MoveItPlanOutcome.SUCCESS,
            detail,
            trajectory=trajectory,
        )

    def _build_request(self, target_hand: PoseStamped):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.group_name
        motion.planner_id = self.planner_id
        motion.num_planning_attempts = 4
        motion.allowed_planning_time = self.allowed_planning_time_sec
        motion.max_velocity_scaling_factor = self.velocity_scaling
        motion.max_acceleration_scaling_factor = self.acceleration_scaling
        motion.start_state.is_diff = True
        motion.goal_constraints = [
            self._pose_goal_constraints(target_hand)
        ]
        return request

    def _build_cartesian_request(self, target_hand: PoseStamped):
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.end_effector_link
        request.waypoints = [deepcopy(target_hand.pose)]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        # Keep custom joint floors out of Cartesian interpolation. The complete
        # returned trajectory is validated before it is sent to Spot.
        # Humble's Cartesian service has no velocity/acceleration scaling fields.
        # The executor stretches trajectory timing to the requested duration.
        return request

    @staticmethod
    def _planning_failure_detail(error_code: int) -> str:
        name = next((
            key for key in dir(MoveItErrorCodes)
            if key.isupper() and getattr(MoveItErrorCodes, key) == error_code
        ), "UNKNOWN")
        detail = (
            "MoveIt could not compute an arm plan "
            f"({name}, error code {error_code})"
        )
        if error_code == MoveItErrorCodes.GOAL_STATE_INVALID:
            detail += (
                ": no valid goal state found for the requested hand pose; "
                "check move_group logs for IK, collision, or joint-limit failures"
            )
        return detail

    def _pose_goal_constraints(
        self,
        target_hand: PoseStamped,
    ) -> Constraints:
        goal = Constraints()
        goal.name = "hand_pose"

        position = PositionConstraint()
        position.header.frame_id = self.planning_frame
        position.link_name = self.end_effector_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [
            2.0 * self.position_tolerance_m,
            2.0 * self.position_tolerance_m,
            2.0 * self.position_tolerance_m,
        ]
        region_pose = Pose()
        region_pose.position = deepcopy(target_hand.pose.position)
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [region_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.planning_frame
        orientation.link_name = self.end_effector_link
        orientation.orientation = deepcopy(target_hand.pose.orientation)
        orientation.absolute_x_axis_tolerance = (
            self.orientation_tolerance_rad
        )
        orientation.absolute_y_axis_tolerance = (
            self.orientation_tolerance_rad
        )
        orientation.absolute_z_axis_tolerance = (
            self.orientation_tolerance_rad
        )
        orientation.weight = 1.0

        goal.position_constraints = [position]
        goal.orientation_constraints = [orientation]
        return goal

    def _partial_trajectory_detail(self, trajectory) -> str:
        names = tuple(trajectory.joint_names)
        points = tuple(trajectory.points)
        prefix = f"returned {len(points)} trajectory points"
        if not points:
            return prefix
        if len(names) != len(ARM_JOINT_NAMES) or set(names) != set(
            ARM_JOINT_NAMES
        ):
            return f"{prefix} with joints {list(names)}"

        ranges = []
        for name in ARM_JOINT_NAMES:
            joint_index = names.index(name)
            values = []
            for point in points:
                if len(point.positions) != len(names):
                    continue
                value = float(point.positions[joint_index])
                if math.isfinite(value):
                    values.append(value)
            if not values:
                ranges.append(f"{name}=unavailable")
                continue
            ranges.append(
                f"{name} start={values[0]:.5f}, end={values[-1]:.5f}, "
                f"min={min(values):.5f}, max={max(values):.5f}"
            )
        return (
            f"{prefix}; "
            + "; ".join(ranges)
            + f"; arm_sh1 safety floor={self.min_arm_sh1_rad:.5f} rad"
        )

    def _validate_and_describe(self, trajectory) -> str:
        names = tuple(trajectory.joint_names)
        if len(names) != len(ARM_JOINT_NAMES) or set(names) != set(
            ARM_JOINT_NAMES
        ):
            raise ValueError(
                "expected exactly the six Spot arm joints, got "
                f"{list(names)}"
            )

        points = tuple(trajectory.points)
        if not points:
            raise ValueError("trajectory contains no points")

        previous_time = None
        ranges = {
            name: [math.inf, -math.inf]
            for name in ARM_JOINT_NAMES
        }
        for index, point in enumerate(points):
            if len(point.positions) != len(names):
                raise ValueError(
                    f"point {index} has {len(point.positions)} positions "
                    f"for {len(names)} joints"
                )
            time_sec = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )
            if not math.isfinite(time_sec) or time_sec < 0.0:
                raise ValueError(
                    f"point {index} has invalid time_from_start"
                )
            if (
                previous_time is not None
                and time_sec <= previous_time
            ):
                raise ValueError(
                    "trajectory timestamps are not strictly increasing"
                )
            previous_time = time_sec

            for joint_index, name in enumerate(names):
                value = float(point.positions[joint_index])
                if not math.isfinite(value):
                    raise ValueError(
                        f"point {index} has non-finite position for {name}"
                    )
                ranges[name][0] = min(ranges[name][0], value)
                ranges[name][1] = max(ranges[name][1], value)

        sh1_min = ranges["arm_sh1"][0]
        if sh1_min < self.min_arm_sh1_rad - 1e-6:
            raise ValueError(
                f"arm_sh1 reaches {sh1_min:.5f} rad below configured "
                f"safety floor {self.min_arm_sh1_rad:.5f} rad"
            )

        duration_sec = previous_time if previous_time is not None else 0.0
        joint_ranges = ", ".join(
            f"{name}=[{ranges[name][0]:.3f}, {ranges[name][1]:.3f}]"
            for name in ARM_JOINT_NAMES
        )
        return (
            f"MoveIt plan ready: {len(points)} points, "
            f"{duration_sec:.3f} s; {joint_ranges}"
        )

    def _reset(self) -> None:
        self._future = None
        self._started_at = None
        self._planning_mode = None
        self._stage = "plan"
        self._pending_request = None
        self._ignore_environment_collisions = False
        self._discard_result = False

    @staticmethod
    def _positive(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{label} must be positive and finite")
        return normalized

    @staticmethod
    def _nonnegative(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(f"{label} must be non-negative and finite")
        return normalized

    @staticmethod
    def _fraction(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or not 0.0 < normalized <= 1.0:
            raise ValueError(f"{label} must be in (0, 1]")
        return normalized

    @staticmethod
    def _finite(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized):
            raise ValueError(f"{label} must be finite")
        return normalized

    @staticmethod
    def _scaling(value, label: str) -> float:
        normalized = float(value)
        if (
            not math.isfinite(normalized)
            or normalized <= 0.0
            or normalized > 1.0
        ):
            raise ValueError(f"{label} must be in (0, 1]")
        return normalized


__all__ = [
    "ARM_JOINT_NAMES",
    "DEFAULT_CARTESIAN_JUMP_THRESHOLD",
    "DEFAULT_CARTESIAN_MAX_STEP_M",
    "DEFAULT_CARTESIAN_MIN_FRACTION",
    "DEFAULT_CARTESIAN_SERVICE_NAME",
    "MoveItArmPlanner",
    "MoveItPlanOutcome",
    "MoveItPlanUpdate",
]
