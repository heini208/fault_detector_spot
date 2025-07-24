# fault_detector_spot/behaviour_tree/nodes/ready_arm_action.py

import py_trees
from spot_msgs.action import RobotCommand
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope


class ReadyArmAction(py_trees.behaviour.Behaviour):
    """
    Sends a Spot arm_ready command via RobotCommand action asynchronously.
    Returns RUNNING while in flight, SUCCESS on completion, FAILURE on error.
    """

    def __init__(self, name: str = "ReadyArmAction", robot_name: str = ""):
        super().__init__(name)
        self.robot_name = robot_name
        self.action_client: ActionClientWrapper = None
        self.initialized = False

        # Async tracking
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if self.node is None:
            raise RuntimeError("ReadyArmAction requires a ROS node")

    def initialize(self):
        """Lazy init of the RobotCommand action client."""
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self.action_client = ActionClientWrapper(RobotCommand, action_ns, self.node)
            self.initialized = True
            return True
        except Exception as e:
            self.logger.error(f"ReadyArmAction init failed: {e}")
            return False

    def update(self) -> py_trees.common.Status:
        # 1) Ensure client is initialized
        if not self.initialized:
            if not self.initialize():
                self.feedback_message = "Initializing ready_arm client..."
                return py_trees.common.Status.RUNNING

        for phase in (self.send_goal, self.wait_for_goal_acceptance, self.terminate_if_goal_result):
            status = phase()
            if status is not None:
                return status

        # 3) Otherwise still in flight
        return py_trees.common.Status.RUNNING

    def send_goal(self) -> py_trees.common.Status or None:
        if self.send_goal_future is None:
            cmd = RobotCommandBuilder.arm_ready_command()
            goal = RobotCommand.Goal()
            convert(cmd, goal.command)
            self.logger.info("ReadyArmAction: sending arm_ready goal")
            self.send_goal_future = self.action_client.send_goal_async(goal)
            self.feedback_message = "Goal sent, waiting for acceptance"
            return py_trees.common.Status.RUNNING
        return None

    def wait_for_goal_acceptance(self) -> py_trees.common.Status or None:
        if self.goal_handle is None:
            if self.send_goal_future.done():
                self.goal_handle = self.send_goal_future.result()
                if not self.goal_handle.accepted:
                    self.feedback_message = "Ready goal rejected"
                    return py_trees.common.Status.FAILURE
                self.get_result_future = self.goal_handle.get_result_async()
                self.feedback_message = "Goal accepted, waiting for result"
            return py_trees.common.Status.RUNNING
        return None

    def terminate_if_goal_result(self) -> py_trees.common.Status or None:
        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.feedback_message = "Arm ready succeeded"
                self._reset_state()
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Ready failed: status {result}"
                self._reset_state()
                return py_trees.common.Status.FAILURE
        return None

    def terminate(self, new_status: py_trees.common.Status):
        # cancel if we're interrupted
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.action_client.cancel_goal_async(self.goal_handle)
        self._reset_state()

    def _reset_state(self):
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self.initialized = False
