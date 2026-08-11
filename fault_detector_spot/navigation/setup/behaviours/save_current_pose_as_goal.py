import py_trees
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class SaveCurrentPoseAsGoal(py_trees.behaviour.Behaviour):
    """
    Copies the last_pose_estimation from the blackboard and sets it
    as last_command.goal_pose every tick.
    """

    def __init__(self, name: str = "SavePoseFromBlackboardAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        """
        Register blackboard keys.
        """
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_pose_estimation", access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        # Ensure last_command exists
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        # Get the latest pose from blackboard
        pose_msg: PoseWithCovarianceStamped = getattr(self.blackboard, "last_pose_estimation", None)
        if pose_msg is None:
            self.feedback_message = "No last_pose_estimation available yet"
            return py_trees.common.Status.FAILURE

        # Copy pose directly to goal_pose
        pose = PoseStamped()
        pose.header = pose_msg.header
        pose.pose = pose_msg.pose.pose
        self.blackboard.last_command.goal_pose = pose
        self.feedback_message = "Copied last_pose_estimation to goal_pose"
        return py_trees.common.Status.SUCCESS
