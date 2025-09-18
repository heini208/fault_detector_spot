import py_trees
from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper
from geometry_msgs.msg import PoseStamped


class AddGoalPoseAsLandmark(py_trees.behaviour.Behaviour):
    """
    Saves the last_command.goal_pose as a landmark in the map's JSON file
    """

    def __init__(self, slam_helper: SlamToolboxHelper, name: str = "AddGoalPoseAsLandmark"):
        super(AddGoalPoseAsLandmark, self).__init__(name)
        self.bb = self.attach_blackboard_client()
        self.helper = slam_helper

    def setup(self, **kwargs):
        """Register required blackboard keys."""
        self.bb.register_key("last_command", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        """Try to save last_command.goal_pose into the landmark JSON file."""
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

        self.helper.add_pose_as_landmark(
            landmark_name=cmd.waypoint_name,
            map_name=cmd.map_name,
            pose=cmd.goal_pose,
        )
        self.feedback_message = (
            f"Saved landmark '{cmd.waypoint_name}' in map '{cmd.map_name}'"
        )
        return py_trees.common.Status.SUCCESS