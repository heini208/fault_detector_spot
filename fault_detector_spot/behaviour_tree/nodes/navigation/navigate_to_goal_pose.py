import py_trees
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from fault_detector_msgs.msg import ComplexCommand

# GoalStatus constants for ROS 2 Humble
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_ABORTED = 5
STATUS_REJECTED = 6
STATUS_PREEMPTED = 7
STATUS_RECALLING = 8
STATUS_RECALLED = 9
STATUS_LOST = 10

class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    Behaviour to navigate to the PoseStamped stored in last_command.goal_pose
    using Nav2's NavigateToPose action.
    """

    def __init__(self, name="NavigateToGoalPose"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self._action_client: ActionClient | None = None
        self._goal_handle = None
        self._result_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Setup requires a ROS node passed as 'node' kwarg")

        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        self._action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def initialise(self):
        last_command: ComplexCommand = self.blackboard.last_command
        if last_command is None or last_command.goal_pose is None:
            self.feedback_message = "No goal_pose in last_command"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose = last_command.goal_pose.pose

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.feedback_message = "Nav2 action server not available"
            return

        future_goal_handle = self._action_client.send_goal_async(goal_msg)
        future_goal_handle.add_done_callback(self._goal_response_callback)
        self.feedback_message = "Sent goal to Nav2"

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.feedback_message = "Goal rejected by Nav2"
            self._goal_handle = None
            self._result_future = None
            return

        self.feedback_message = "Goal accepted by Nav2"
        self._goal_handle = goal_handle  # store actual GoalHandle
        self._result_future = goal_handle.get_result_async()

    def update(self):
        last_command: ComplexCommand = self.blackboard.last_command
        if last_command is None or last_command.goal_pose is None:
            self.feedback_message = "No goal_pose to navigate to"
            return py_trees.common.Status.FAILURE

        if self._result_future is None:
            self.feedback_message = "Waiting for Nav2 to accept goal"
            return py_trees.common.Status.RUNNING

        rclpy.spin_once(self.node, timeout_sec=0)

        if self._result_future.done():
            result = self._result_future.result()
            status = result.status
            if status == STATUS_SUCCEEDED:
                self.feedback_message = "Navigation succeeded"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Navigation failed with status {status}"
                return py_trees.common.Status.FAILURE

        self.feedback_message = "Navigating..."
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self._result_future = None
