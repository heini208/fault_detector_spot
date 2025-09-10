import py_trees
from geometry_msgs.msg import PoseStamped


class AddGoalPoseAsWaypoint(py_trees.behaviour.Behaviour):
    """
    Saves the last_command.goal_pose as a waypoint in the map's JSON file
    """

    def __init__(self, name: str = "AddGoalPoseAsWaypoint"):
        super(AddGoalPoseAsWaypoint, self).__init__(name)
        self.bb = self.attach_blackboard_client()

    def setup(self, **kwargs):
        """Register required blackboard keys."""
        self.bb.register_key("last_command", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        """Try to save last_command.goal_pose into the waypoint JSON file."""
        if not self.bb.exists("last_command") or self.bb.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        cmd = self.bb.last_command
        if not hasattr(cmd, "goal_pose") or not isinstance(cmd.goal_pose, PoseStamped):
            self.feedback_message = "last_command has no valid goal_pose"
            return py_trees.common.Status.FAILURE

        if not hasattr(cmd, "map_name") or cmd.map_name is None:
            self.feedback_message = "last_command has no map_name"
            return py_trees.common.Status.FAILURE

        if not hasattr(cmd, "waypoint_name") or cmd.waypoint_name is None:
            self.feedback_message = "last_command has no waypoint_name"
            return py_trees.common.Status.FAILURE

        try:
            self.helper.add_pose_as_waypoint(
                waypoint_name=cmd.waypoint_name,
                map_name=cmd.map_name,
                pose=cmd.goal_pose,
            )
            self.feedback_message = (
                f"Saved waypoint '{cmd.waypoint_name}' in map '{cmd.map_name}'"
            )
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Failed to save waypoint: {e}"
            return py_trees.common.Status.FAILURE
