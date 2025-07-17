# fault_detector_spot/behaviour_tree/nodes/stow_arm_action.py

import py_trees
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope


class StowArmAction(py_trees.behaviour.Behaviour):
    """
    Sends a Spot arm_stow command via RobotCommand action asynchronously.
    Returns RUNNING while in flight, SUCCESS on completion, FAILURE on error.
    """

    def __init__(self, name: str = "StowArmAction", robot_name: str = ""):
        super().__init__(name)
        self.robot_name = robot_name
        self.action_client: ActionClientWrapper = None
        self.initialized = False

        # Async tracking
        self.send_goal_future = None
        self.goal_handle       = None
        self.get_result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if self.node is None:
            raise RuntimeError("StowArmAction requires a ROS node")

    def initialize(self):
        """Lazy init"""
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self.action_client = ActionClientWrapper(
                RobotCommand, action_ns, self.node
            )
            self.initialized = True
            return True
        except Exception as e:
            self.logger.error(f"StowArmAction init failed: {e}")
            return False

    def update(self) -> py_trees.common.Status:
        if not self.initialized:
            if not self.initialize():
                self.feedback_message = "Initializing stow client..."
                return py_trees.common.Status.RUNNING

        if self.send_goal_future is None:
            cmd = RobotCommandBuilder.arm_stow_command()
            goal = RobotCommand.Goal()
            convert(cmd, goal.command)
            self.logger.info("StowArmAction: sending arm_stow goal")
            self.send_goal_future = self.action_client.send_goal_async(goal)
            self.feedback_message = "Goal sent, waiting for acceptance"
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            if self.send_goal_future.done():
                self.goal_handle = self.send_goal_future.result()
                if not self.goal_handle.accepted:
                    self.feedback_message = "Stow goal rejected"
                    return py_trees.common.Status.FAILURE
                self.get_result_future = self.goal_handle.get_result_async()
                self.feedback_message = "Goal accepted, waiting for result"
            return py_trees.common.Status.RUNNING

        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.feedback_message = "Arm stowed successfully"
                # reset so this node can be reused later
                self._reset_state()
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Stow failed: status {result}"
                self._reset_state()
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.action_client.cancel_goal_async(self.goal_handle)
        self._reset_state()

    def _reset_state(self):
        self.send_goal_future = None
        self.goal_handle       = None
        self.get_result_future = None
        self.sent              = False
