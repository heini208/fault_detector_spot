import py_trees
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


class SaveCurrentPoseAsGoal(py_trees.behaviour.Behaviour):
    """
    Saves the next published robot pose from /localization_pose
    (PoseWithCovarianceStamped) into last_command.goal_pose on the blackboard.
    Waits until a new message is received.
    """

    def __init__(self, name: str = "SaveCurrentPoseAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.pose_received = None
        self.subscription = None

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

        # Subscribe to localization pose
        self.subscription = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Called whenever a new pose is published.
        Stores the pose for update() to save.
        """
        self.pose_received = msg

    def update(self) -> py_trees.common.Status:
        # Ensure last_command exists
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        # Check if a new pose was received
        if self.pose_received is None:
            self.feedback_message = "Waiting for /localization_pose message..."
            return py_trees.common.Status.RUNNING

        # Save received pose to blackboard (extract PoseStamped)
        pose_stamped = self.pose_received
        self.blackboard.last_command.goal_pose.header = pose_stamped.header
        self.blackboard.last_command.goal_pose.pose = pose_stamped.pose.pose

        self.feedback_message = "Saved current pose from /localization_pose"
        # Clear it so next execution waits for a new message
        self.pose_received = None
        return py_trees.common.Status.SUCCESS
