"""Own process-wide ROS resources used by Spot robot commands."""

from threading import RLock

from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStateSource


class RobotCommandResources:
    """Share robot-command clients, TF, arm state, and arm movement helpers."""

    def __init__(self):
        self._lock = RLock()
        self._node = None
        self._clients = {}
        self._tf_listener = None
        self._arm_state_source = None
        self._arm_movement_executors = {}

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
        """Return the process-wide authoritative manipulator state source."""
        with self._lock:
            self._bind_node(node)
            if self._arm_state_source is None:
                self._arm_state_source = ArmStateSource(node)
            return self._arm_state_source

    def get_arm_movement_executor(self, node, robot_name: str = ""):
        """Return the shared Cartesian arm movement executor."""
        with self._lock:
            self._bind_node(node)
            executor = self._arm_movement_executors.get(robot_name)
            if executor is None:
                executor = ArmMovementExecutor(
                    self.get_tf_listener(node),
                    robot_name=robot_name,
                )
                self._arm_movement_executors[robot_name] = executor
            return executor

    def close(self):
        """Destroy every ROS entity owned by this resource container."""
        with self._lock:
            node = self._node
            tf_listener = self._tf_listener
            arm_state_source = self._arm_state_source
            clients = tuple(self._clients.values())
            self._tf_listener = None
            self._arm_state_source = None
            self._arm_movement_executors.clear()
            self._clients.clear()
            self._node = None

        resources = []
        if tf_listener is not None:
            resources.append(("TF listener", tf_listener.shutdown))
        if arm_state_source is not None:
            resources.append(("arm state source", arm_state_source.destroy))
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
                        f"Could not close shared {resource_name}: {exception}"
                    )

    def _bind_node(self, node):
        if node is None:
            raise RuntimeError("RobotCommandResources requires a ROS node")
        if self._node is None:
            self._node = node
            return
        if self._node is not node:
            raise RuntimeError(
                "RobotCommandResources cannot be shared across ROS nodes"
            )
