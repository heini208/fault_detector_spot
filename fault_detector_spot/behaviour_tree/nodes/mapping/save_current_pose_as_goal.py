import py_trees
import rclpy
from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper
from geometry_msgs.msg import PoseWithCovarianceStamped


class SaveCurrentPoseAsGoal(py_trees.behaviour.Behaviour):
    """
    Saves the next published robot pose from /amcl_pose
    (PoseWithCovarianceStamped) into last_command.goal_pose on the blackboard.
    Waits until a new message is received.
    """

    def __init__(self, slam_helper : SlamToolboxHelper, name: str = "SaveCurrentPoseAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.slam_helper = slam_helper
        self.last_stamp = None

    def setup(self, **kwargs):
        """
        Setup ROS subscription and register blackboard key.
        Expects 'node' in kwargs.
        """
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Setup requires a ROS node passed as 'node' kwarg")

        # Register blackboard key
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )

    def update(self) -> py_trees.common.Status:
        # Ensure last_command exists
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        # Get the latest pose from Nav2Helper
        pose_msg = self.slam_helper.get_last_localization_pose()
        if pose_msg is None:
            self.feedback_message = "No AMCL pose available yet"
            return py_trees.common.Status.RUNNING

        # Only update if it's new
        if self.last_stamp is None or pose_msg.header.stamp != self.last_stamp:
            self.blackboard.last_command.goal_pose.header = pose_msg.header
            self.blackboard.last_command.goal_pose.pose = pose_msg.pose
            self.last_stamp = pose_msg.header.stamp
            self.feedback_message = "Saved new AMCL pose as goal_pose"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Waiting for next AMCL pose update..."
            return py_trees.common.Status.RUNNING
