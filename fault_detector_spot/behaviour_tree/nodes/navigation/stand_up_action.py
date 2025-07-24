# fault_detector_spot/behaviour_tree/nodes/stand_up_action.py

import py_trees
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope


class StandUpAction(py_trees.behaviour.Behaviour):
    """
    Sends a Spot stand command via RobotCommand action asynchronously.
    Returns RUNNING while in flight, SUCCESS on completion, FAILURE on error.
    """

    def __init__(self, name: str = "StandUpAction", robot_name: str = ""):
        super().__init__(name)
        self.robot_name = robot_name
        self._client: ActionClientWrapper = None
        self._initialized = False

        # Async‐tracking
        self._send_goal_future = None
        self._goal_handle       = None
        self._get_result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if not self.node:
            raise RuntimeError("StandUpAction requires a ROS node")

    def initialize(self) -> bool:
        """Lazy‐init the action client."""
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
            self._initialized = True
            return True
        except Exception as e:
            self.logger.error(f"[StandUpAction] init failed: {e}")
            return False

    def update(self) -> py_trees.common.Status:
        if not self._initialized:
            return self._run_initialization()

        if not self._send_goal_future:
            return self._send_goal()

        if not self._goal_handle:
            return self._await_acceptance()

        return self._await_result()

    def _run_initialization(self) -> py_trees.common.Status:
        if self.initialize():
            self.feedback_message = "Stand‑up client initialized"
        else:
            self.feedback_message = "Initializing stand‑up client..."
        return py_trees.common.Status.RUNNING

    def _build_goal(self) -> RobotCommand.Goal:
        """Builds the synchro stand command goal message."""
        stand_cmd = RobotCommandBuilder.synchro_stand_command()
        goal = RobotCommand.Goal()
        convert(stand_cmd, goal.command)
        return goal

    def _send_goal(self) -> py_trees.common.Status:
        """Sends the stand command asynchronously once."""
        goal = self._build_goal()
        self.logger.info("[StandUpAction] Sending stand goal")
        self._send_goal_future = self._client.send_goal_async(goal)
        self.feedback_message = "Stand goal sent"
        return py_trees.common.Status.RUNNING

    def _await_acceptance(self) -> py_trees.common.Status:
        """Waits for the action server to accept or reject the goal."""
        if not self._send_goal_future.done():
            return py_trees.common.Status.RUNNING

        self._goal_handle = self._send_goal_future.result()
        if not self._goal_handle.accepted:
            self.feedback_message = "Stand goal rejected"
            self._reset_state()
            return py_trees.common.Status.FAILURE

        self._get_result_future = self._goal_handle.get_result_async()
        self.feedback_message = "Stand accepted, awaiting result"
        return py_trees.common.Status.RUNNING

    def _await_result(self) -> py_trees.common.Status:
        """Checks for the final result of the stand action."""
        if not self._get_result_future.done():
            return py_trees.common.Status.RUNNING

        result = self._get_result_future.result().result
        if result.success:
            self.feedback_message = "Stood up successfully"
            status = py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Stand failed: status {result}"
            status = py_trees.common.Status.FAILURE

        self._reset_state()
        return status

    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle:
            self._client.cancel_goal_async(self._goal_handle)
        self._reset_state()

    def _reset_state(self):
        """Clears internal futures so this behaviour can be re‑used."""
        self._send_goal_future = None
        self._goal_handle       = None
        self._get_result_future = None
        self._initialized       = False
