"""Own process-wide ROS resources used by Spot robot commands."""

from threading import RLock

from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    DEFAULT_READY_DEPLOYED_TIMEOUT_SEC,
    DEFAULT_READY_LIFT_DISTANCE_M,
    DEFAULT_READY_STATE_TIMEOUT_SEC,
    DEFAULT_READY_TF_TIMEOUT_SEC,
    DEFAULT_STOW_STATE_TIMEOUT_SEC,
    READY_DEPLOYED_TIMEOUT_PARAMETER,
    READY_LIFT_DISTANCE_PARAMETER,
    READY_STATE_TIMEOUT_PARAMETER,
    READY_TF_TIMEOUT_PARAMETER,
    STOW_STATE_TIMEOUT_PARAMETER,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStateSource,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor,
)


class RobotCommandResources:
    """Share RobotCommand clients, TF, and movement executors."""

    def __init__(self):
        self._lock = RLock()
        self._node = None
        self._clients = {}
        self._tf_listener = None
        self._arm_state_source = None
        self._arm_motion_speed_policy = None
        self._arm_movement_executors = {}
        self._base_movement_executors = {}

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
                self._clients[action_ns] = client
            return client

    def get_tf_listener(self, node):
        """Return the single TF listener used to prepare move commands."""
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
                    tag_state_source=tag_state_source,
                    robot_name=robot_name,
                    speed_policy=self._arm_motion_speed_policy,
                    action_client=self.get_action_client(
                        node,
                        robot_name,
                    ),
                    arm_state_source=self.get_arm_state_source(node),
                    ready_lift_distance_m=self._positive_parameter(
                        node,
                        READY_LIFT_DISTANCE_PARAMETER,
                        DEFAULT_READY_LIFT_DISTANCE_M,
                    ),
                    ready_state_timeout_sec=self._positive_parameter(
                        node,
                        READY_STATE_TIMEOUT_PARAMETER,
                        DEFAULT_READY_STATE_TIMEOUT_SEC,
                    ),
                    ready_tf_timeout_sec=self._positive_parameter(
                        node,
                        READY_TF_TIMEOUT_PARAMETER,
                        DEFAULT_READY_TF_TIMEOUT_SEC,
                    ),
                    ready_deployed_timeout_sec=self._positive_parameter(
                        node,
                        READY_DEPLOYED_TIMEOUT_PARAMETER,
                        DEFAULT_READY_DEPLOYED_TIMEOUT_SEC,
                    ),
                    stow_state_timeout_sec=self._positive_parameter(
                        node,
                        STOW_STATE_TIMEOUT_PARAMETER,
                        DEFAULT_STOW_STATE_TIMEOUT_SEC,
                    ),
                    logger=node.get_logger(),
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
                    tag_state_source=tag_state_source,
                    robot_name=robot_name,
                    action_client=self.get_action_client(
                        node,
                        robot_name,
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

    def close(self):
        """Destroy every ROS entity owned by this resource container."""
        with self._lock:
            node = self._node
            tf_listener = self._tf_listener
            arm_state_source = self._arm_state_source
            arm_executors = tuple(
                self._arm_movement_executors.values()
            )
            base_executors = tuple(
                self._base_movement_executors.values()
            )
            clients = tuple(self._clients.values())
            self._tf_listener = None
            self._arm_state_source = None
            self._arm_motion_speed_policy = None
            self._arm_movement_executors.clear()
            self._base_movement_executors.clear()
            self._clients.clear()
            self._node = None

        resources = []
        resources.extend(
            ("arm movement executor", executor.shutdown)
            for executor in arm_executors
        )
        resources.extend(
            ("base movement executor", executor.shutdown)
            for executor in base_executors
        )
        if tf_listener is not None:
            resources.append(
                ("TF listener", tf_listener.shutdown)
            )
        if arm_state_source is not None:
            resources.append(
                ("arm state source", arm_state_source.destroy)
            )
        resources.extend(
            ("RobotCommand action client", client.destroy)
            for client in clients
        )
        for resource_name, destroy in resources:
            try:
                destroy()
            except Exception as exception:
                if node is not None:
                    node.get_logger().warning(
                        f"Could not close shared {resource_name}: "
                        f"{exception}"
                    )

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
