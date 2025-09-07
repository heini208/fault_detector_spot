import py_trees
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from fault_detector_msgs.msg import ComplexCommand

class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    Behaviour to navigate to the PoseStamped stored in last_command.goal_pose
    using Nav2's NavigateToPose action.
    """

    def __init__(self, name="NavigateToGoalPose"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self._action_client = None
        self._goal_handle = None
        self._result_future = None

    def setup(self, **kwargs):
        """
        Expects 'node' in kwargs to create action client.
        """
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Setup requires a ROS node passed as 'node' kwarg")

        # Register blackboard key
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)

        # Create NavigateToPose action client
        self._action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def initialise(self):
        """
        Called at the start of the tick. Send the goal if available.
        """
        last_command: ComplexCommand = self.blackboard.last_command

        if last_command is None or last_command.goal_pose is None:
            self.feedback_message = "No goal_pose in last_command"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = last_command.goal_pose

        self._action_client.wait_for_server(timeout_sec=5.0)
        self._goal_handle = self._action_client.send_goal_async(goal_msg)
        self._goal_handle.add_done_callback(self._goal_response_callback)

        self.feedback_message = f"Sent goal to Nav2"

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.feedback_message = "Goal rejected by Nav2"
            self._result_future = None
            return
        self.feedback_message = "Goal accepted by Nav2"
        self._result_future = goal_handle.get_result_async()

    def update(self):
        last_command: ComplexCommand = self.blackboard.last_command
        if last_command is None or last_command.goal_pose is None:
            self.feedback_message = "No goal_pose to navigate to"
            return py_trees.common.Status.FAILURE

        if self._result_future is None:
            # Goal not yet accepted
            self.feedback_message = "Waiting for Nav2 to accept goal"
            return py_trees.common.Status.RUNNING

        # Check if result is ready
        rclpy.spin_once(self.node, timeout_sec=0)
        if self._result_future.done():
            result = self._result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.feedback_message = "Navigation succeeded"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Navigation failed with status {result.status}"
                return py_trees.common.Status.FAILURE

        self.feedback_message = "Navigating..."
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Cancel goal if behaviour is stopped or aborted.
        """
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
